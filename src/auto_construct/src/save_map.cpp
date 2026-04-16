#include "auto_construct/save_map.h"
#include <chrono>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

SaveMap::SaveMap() : Node("save_map_node") {

    // 1. 声明默认保存路径 (可通过 Launch 或小程序的参数覆写)
    this->declare_parameter("save_path", "/home/nic/ROS/ROS/map/maps");

    // 2. 注册给前端小程序的服务 API
    save_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/save_map", std::bind(&SaveMap::save_callback, this, _1, _2));
        
    //用来给octomap清零，暂时没用
    reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/reset_map", std::bind(&SaveMap::reset_callback, this, _1, _2));

    //fast_lio_save_client_ = this->create_client<std_srvs::srv::Trigger>("/map_save");
    pgo_save_client_ = this->create_client<interface::srv::SaveMaps>("/pgo/save_maps");  //存储3D图像

    // 3. 初始化向底层节点发送指令的客户端
    octomap_reset_client_ = this->create_client<std_srvs::srv::Empty>("/octomap_server/reset");
    nav2_save_client_ = this->create_client<nav2_msgs::srv::SaveMap>("/map_saver_server/save_map");

    RCLCPP_INFO(this->get_logger(), "🚀 SaveMap 纯异步业务网关已启动！");
    RCLCPP_INFO(this->get_logger(), "等待前端指令：/save_map (存图) 或 /reset_map (清空重扫)");


    // 

    declareParameters();
    getParameters();

    rclcpp::QoS map_qos(10);
    map_qos.transient_local();
    map_qos.reliable();
    map_qos.keep_last(1);
    pcd_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_name_, map_qos); //发布 2D 栅格地图
    pcd_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("save/filter_pcd_cloud", 10);         //发布滤波后的 3D 点云


}

void SaveMap::reset_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    if (!octomap_reset_client_->wait_for_service(2s)) {
        response->success = false;
        response->message = "重置失败：未检测到底层的 Octomap 服务！";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    auto empty_req = std::make_shared<std_srvs::srv::Empty::Request>();
    octomap_reset_client_->async_send_request(empty_req);

    response->success = true;
    response->message = "底层地图已清空，正从当前位置重新构建...";
    RCLCPP_INFO(this->get_logger(), "🧹 已通过官方接口清空 Octomap 缓存！");
}




void SaveMap::save_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    // 1. 检查底层服务是否在线 (PGO 存 3D，Nav2 存 2D)
    if (!pgo_save_client_->wait_for_service(3s) || !nav2_save_client_->wait_for_service(3s)) {
        response->success = false;
        response->message = "同步失败：底层 PGO(3D) 或 Nav2(2D) 存图服务未在线！";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    // 2. 生成当前时间戳文件夹路径
    // 基础目录：/home/nic/ROS/ROS/map/maps/
    // 目标目录：/home/nic/ROS/ROS/map/maps/map_20260322_231005/
    std::string base_dir = "/home/nic/ROS/ROS/map/maps/";
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << base_dir << "map_" << std::put_time(&tm, "%Y%m%d_%H%M%S");
    std::string folder_path = oss.str();

    // 3. 物理创建这个文件夹 (关键步骤！)
    try {
        if (!std::filesystem::exists(folder_path)) {
            std::filesystem::create_directories(folder_path);
            RCLCPP_INFO(this->get_logger(), "📂 已创建新地图文件夹: %s", folder_path.c_str());
        }
    } catch (const std::exception &e) {
        response->success = false;
        response->message = "文件夹创建失败: " + std::string(e.what());
        return;
    }

    // 4. 准备 3D 存图请求 (PGO)
    // 注意：PGO 内部会自动在 folder_path 下生成 map.pcd
    auto pgo_req = std::make_shared<interface::srv::SaveMaps::Request>();
    pgo_req->file_path = folder_path; 
    pgo_req->save_patches = false; 
    pgo_save_client_->async_send_request(pgo_req);

    // 5. 准备 2D 存图请求 (Nav2 Map Saver)
    // 注意：Nav2 需要的是文件名前缀，我们存为 folder_path/map_2d
    auto nav2_req = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
    nav2_req->map_url = folder_path + "/octomap_map"; 
    nav2_req->map_topic = "/projected_map"; // 或者是你的 /map 话题
    nav2_req->image_format = "pgm";
    nav2_req->map_mode = "trinary";
    nav2_save_client_->async_send_request(nav2_req);


    //pcd 2 pgm 的异步处理线程，避免阻塞主线程
    std::thread(&SaveMap::async_save_task, this, folder_path).detach();


    // 6. 返回结果给小程序
    response->success = true;
    response->message = "✅ 3D/2D 同步保存指令已发送！文件夹：" + folder_path;
    RCLCPP_INFO(this->get_logger(), "🚀 同步存图任务已下发至底层节点");
}



//////////////////// pcd 2 pgm 相关的成员函数实现 ////////////////////

// pcd 2 pgm  实现
void SaveMap::async_save_task(const std::string folder_path) {
    // 等待 5 秒，确保 PGO 已经把 map.pcd 彻底写进硬盘
    std::this_thread::sleep_for(std::chrono::seconds(5));

    pcd_file_ = folder_path + "/map.pcd";
    pcd_cloud_->clear();
    if (cloud_after_pass_through_) cloud_after_pass_through_->clear();
    if (cloud_after_radius_) cloud_after_radius_->clear();


    // 🌟 原封不动调用搬运过来的执行逻辑
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *pcd_cloud_) == -1) {
        RCLCPP_ERROR(get_logger(), "Couldn't read file: %s", pcd_file_.c_str());
        return;
    }
    RCLCPP_INFO(get_logger(), "Initial point cloud size: %lu", pcd_cloud_->points.size());

    applyTransform();
    passThroughFilter(thre_z_min_, thre_z_max_, flag_pass_through_);
    radiusOutlierFilter(cloud_after_pass_through_, thre_radius_, thres_point_count_);
    setMapTopicMsg(cloud_after_radius_, map_topic_msg_);
    publishCallback(); // 这一步发布到 map_topic_name_ 话题

    // 5. 原封不动的 2D 存图请求 (Nav2 Map Saver)
    auto nav2_req = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
    nav2_req->map_url = folder_path + "/pcd2pgm_map"; 
    nav2_req->map_topic = map_topic_name_; // 对接 pcd2pgm 发布的话题
    nav2_req->image_format = "pgm";
    nav2_req->map_mode = "trinary";
    nav2_save_client_->async_send_request(nav2_req);

}




void SaveMap::declareParameters()
{
  declare_parameter("thre_z_min", 0.2); 
  declare_parameter("thre_z_max", 1.5);
  declare_parameter("flag_pass_through", true);
  declare_parameter("thre_radius", 0.5);
  declare_parameter("map_resolution", 0.05);
  declare_parameter("thres_point_count", 1);
  declare_parameter("map_topic_name", "/pcd2map"); 
  declare_parameter("odom_to_lidar_odom", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});  
}

void SaveMap::getParameters()
{
  get_parameter("thre_z_min", thre_z_min_);
  get_parameter("thre_z_max", thre_z_max_);
  get_parameter("flag_pass_through", flag_pass_through_);
  get_parameter("thre_radius", thre_radius_);
  get_parameter("map_resolution", map_resolution_);
  get_parameter("thres_point_count", thres_point_count_);
  get_parameter("map_topic_name", map_topic_name_);
  get_parameter("odom_to_lidar_odom", odom_to_lidar_odom_);  
}

//滤波
void SaveMap::passThroughFilter(double thre_low, double thre_high, bool flag_in)
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(pcd_cloud_);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(thre_low, thre_high);
  passthrough.setNegative(flag_in);
  passthrough.filter(*filtered_cloud);

  cloud_after_pass_through_ = filtered_cloud;
  RCLCPP_INFO(get_logger(), "After PassThrough filtering: %lu points", cloud_after_pass_through_->points.size());
}

//滤波
void SaveMap::radiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud, double radius, int thre_count)
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier;
  radius_outlier.setInputCloud(input_cloud);
  radius_outlier.setRadiusSearch(radius);
  radius_outlier.setMinNeighborsInRadius(thre_count);
  radius_outlier.filter(*filtered_cloud);

  cloud_after_radius_ = filtered_cloud;
  RCLCPP_INFO(get_logger(), "After RadiusOutlier filtering: %lu points", cloud_after_radius_->points.size());
}


void SaveMap::setMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & msg)
{
  msg.header.stamp = now();
  msg.header.frame_id = "map";
  msg.info.map_load_time = now();
  msg.info.resolution = map_resolution_;

  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  if (cloud->points.empty()) {
    RCLCPP_WARN(get_logger(), "Point cloud is empty!");
    return;
  }

  for (const auto & point : cloud->points) {
    x_min = std::min(x_min, static_cast<double>(point.x));
    x_max = std::max(x_max, static_cast<double>(point.x));
    y_min = std::min(y_min, static_cast<double>(point.y));
    y_max = std::max(y_max, static_cast<double>(point.y));
  }

  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.info.width = std::ceil((x_max - x_min) / map_resolution_);
  msg.info.height = std::ceil((y_max - y_min) / map_resolution_);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  for (const auto & point : cloud->points) {
    int i = std::floor((point.x - x_min) / map_resolution_);
    int j = std::floor((point.y - y_min) / map_resolution_);

    if (i >= 0 && i < msg.info.width && j >= 0 && j < msg.info.height) {
      msg.data[i + j * msg.info.width] = 100;
    }
  }
  RCLCPP_INFO(get_logger(), "Map data size: %lu", msg.data.size());
}

void SaveMap::publishCallback()
{
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*cloud_after_radius_, output);
  output.header.frame_id = "map";
  pcd_publisher_->publish(output);
  map_publisher_->publish(map_topic_msg_);
}


void SaveMap::applyTransform()
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << odom_to_lidar_odom_[0], odom_to_lidar_odom_[1], odom_to_lidar_odom_[2];
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[3], Eigen::Vector3f::UnitX()));
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[4], Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[5], Eigen::Vector3f::UnitZ()));

  pcl::transformPointCloud(*pcd_cloud_, *pcd_cloud_, transform.inverse());
}







int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto save_map_node = std::make_shared<SaveMap>();
    
    rclcpp::spin(save_map_node);
    
    rclcpp::shutdown();
    return 0;
}
