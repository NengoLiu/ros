#ifndef SAVE_MAP_H
#define SAVE_MAP_H

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <memory>
#include <ctime>     // 用于获取系统时间
#include <iomanip>   // 用于格式化时间 (比如 %Y%m%d)
#include <sstream>   // 用于字符串拼接

#include <filesystem> // 必须包含这个头文件

#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav2_msgs/srv/save_map.hpp"

#include "interface/srv/save_maps.hpp"


#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl/filters/passthrough.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include "sensor_msgs/msg/point_cloud2.hpp"



class SaveMap : public rclcpp::Node {
public:
    SaveMap();
    ~SaveMap() = default;

private:
    // 暴露给微信小程序后端的两个核心服务回调
    void save_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void reset_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // ROS 2 服务端 (对外提供 API)
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    
    // ROS 2 客户端 (对内异步调用底层算法库)
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr octomap_reset_client_;
    rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr nav2_save_client_;

    rclcpp::Client<interface::srv::SaveMaps>::SharedPtr pgo_save_client_;


////////

    void async_save_task(const std::string folder_path);

    void declareParameters();
    void getParameters();
    void passThroughFilter(double thre_low, double thre_high, bool flag_in);
    void radiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud, double radius, int thre_count);
    void setMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & msg);
    void publishCallback();
    void applyTransform();

    // pcd2pgm 的成员变量
    float thre_z_min_;
    float thre_z_max_;
    float thre_radius_;
    bool flag_pass_through_;
    float map_resolution_;
    int thres_point_count_;
    std::string pcd_file_;
    std::string map_topic_name_;
    std::vector<double> odom_to_lidar_odom_;



    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcd_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_pass_through_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_radius_;
    nav_msgs::msg::OccupancyGrid map_topic_msg_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_publisher_;

};

#endif // WEB_CONNECT_SAVE_MAP_H
