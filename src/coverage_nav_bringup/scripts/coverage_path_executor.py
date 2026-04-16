#!/usr/bin/env python3
"""
coverage_path_executor.py  —  覆盖路径执行节点

从 YAML 文件加载覆盖路径，驱动 Nav2 NavigateThroughPoses 完成覆盖作业。

支持的 YAML 格式 (你当前的格式):
    boundary: [...]
    pillars:  [...]
    path:
      frame_id: map          # 如果是 "undefined" 会自动用 frame_id 参数替代
      poses:
        - position:    {x, y, z}
          orientation: {x, y, z, w}
        - ...

发布 Topic:
    ~/status   (std_msgs/String)   — IDLE / RUNNING / PAUSED / DONE / FAILED / CANCELLED
    ~/progress (std_msgs/Float32)  — 0.0 ~ 100.0
    ~/done     (std_msgs/Bool)     — True=成功, False=失败/取消

服务接口:
    ~/start    (std_srvs/Trigger)  — 开始执行
    ~/pause    (std_srvs/Trigger)  — 暂停 (保留当前进度)
    ~/resume   (std_srvs/Trigger)  — 继续
    ~/cancel   (std_srvs/Trigger)  — 取消整个任务

用法示例:
    # 启动节点
    ros2 run coverage_nav_bringup coverage_path_executor \
        --ros-args -p path_file:=/path/to/coverage.yaml -p frame_id:=map

    # 开始执行
    ros2 service call /coverage_path_executor/start std_srvs/srv/Trigger {}

    # 暂停
    ros2 service call /coverage_path_executor/pause std_srvs/srv/Trigger {}

    # 继续
    ros2 service call /coverage_path_executor/resume std_srvs/srv/Trigger {}

    # 取消
    ros2 service call /coverage_path_executor/cancel std_srvs/srv/Trigger {}

    # 查看进度
    ros2 topic echo /coverage_path_executor/progress
"""

import time
import threading
import yaml
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger


# ──────────────────────────────────────────────────────────────────────────────
# 工具函数
# ──────────────────────────────────────────────────────────────────────────────

def _progress_bar(pct: float, width: int = 25) -> str:
    filled = int(width * pct / 100.0)
    bar = '█' * filled + '░' * (width - filled)
    return f'[{bar}] {pct:5.1f}%'


def _fmt_time(seconds: float) -> str:
    s = int(seconds)
    if s < 60:
        return f'{s}s'
    elif s < 3600:
        return f'{s // 60}m{s % 60:02d}s'
    else:
        return f'{s // 3600}h{(s % 3600) // 60:02d}m'


# ──────────────────────────────────────────────────────────────────────────────
# 执行节点
# ──────────────────────────────────────────────────────────────────────────────

class CoveragePathExecutor(Node):

    # ── 初始化 ─────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('coverage_path_executor')

        # ---- 参数 ----
        self.declare_parameter('path_file',       '')
        self.declare_parameter('frame_id',        'map')
        self.declare_parameter('skip_on_failure', True)
        self.declare_parameter('autostart',       False)

        self._path_file      = self.get_parameter('path_file').value
        self._frame_id       = self.get_parameter('frame_id').value
        self._skip_on_failure = self.get_parameter('skip_on_failure').value
        self._autostart      = self.get_parameter('autostart').value

        # ---- ROS 接口 ----
        # 允许服务和 action 回调并发执行
        self._cb = ReentrantCallbackGroup()

        self._nav_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses',
            callback_group=self._cb,
        )

        # 控制服务
        self.create_service(Trigger, '~/start',  self._svc_start,  callback_group=self._cb)
        self.create_service(Trigger, '~/pause',  self._svc_pause,  callback_group=self._cb)
        self.create_service(Trigger, '~/resume', self._svc_resume, callback_group=self._cb)
        self.create_service(Trigger, '~/cancel', self._svc_cancel, callback_group=self._cb)

        # 状态发布
        self._pub_status   = self.create_publisher(String,  '~/status',   10)
        self._pub_progress = self.create_publisher(Float32, '~/progress', 10)
        self._pub_done     = self.create_publisher(Bool,    '~/done',     10)

        # ---- 内部状态 ----
        self._all_poses: List[PoseStamped] = []
        self._total: int = 0

        self._resume_index: int  = 0     # 下次从哪个 index 发送
        self._sent_count: int    = 0     # 本次 action 共发出多少 pose
        self._last_remaining: int = 0    # 最新 feedback 中的 remaining 数

        self._goal_handle = None
        self._goal_lock   = threading.Lock()

        self._paused    = False
        self._cancelled = False
        self._running   = False
        self._start_time: float = 0.0

        # ---- 加载路径 ----
        if self._path_file:
            self._do_load(self._path_file)

        # ---- 启动提示 ----
        sep = '─' * 56
        self.get_logger().info(sep)
        self.get_logger().info('  CoveragePathExecutor 已启动')
        self.get_logger().info(f'  路径文件:   {self._path_file or "(未指定)"}')
        self.get_logger().info(f'  总路径点:   {self._total}')
        self.get_logger().info(f'  坐标系:     {self._frame_id}')
        self.get_logger().info(f'  跳过失败:   {self._skip_on_failure}')
        self.get_logger().info('  控制命令:')
        self.get_logger().info('    ros2 service call ~/start  std_srvs/srv/Trigger {}')
        self.get_logger().info('    ros2 service call ~/pause  std_srvs/srv/Trigger {}')
        self.get_logger().info('    ros2 service call ~/resume std_srvs/srv/Trigger {}')
        self.get_logger().info('    ros2 service call ~/cancel std_srvs/srv/Trigger {}')
        self.get_logger().info(sep)

        if self._autostart and self._all_poses:
            self.get_logger().info('autostart=true，3 秒后自动开始...')
            self._autostart_timer = self.create_timer(3.0, self._autostart_once)

    # ── YAML 加载 ──────────────────────────────────────────────────────────────

    def _do_load(self, path_file: str) -> None:
        """加载路径文件，成功则更新 _all_poses / _total / _frame_id。"""
        try:
            poses, frame_id = _parse_coverage_yaml(
                path_file, fallback_frame=self._frame_id, logger=self.get_logger()
            )
            self._all_poses = poses
            self._total     = len(poses)
            self._frame_id  = frame_id
            self.get_logger().info(
                f'路径加载成功: {self._total} 个路径点 (frame_id={frame_id})'
            )
        except Exception as e:
            self.get_logger().error(f'路径加载失败: {e}')

    # ── 服务回调 ───────────────────────────────────────────────────────────────

    def _svc_start(self, req, res):
        if self._running:
            res.success = False
            res.message = '任务已在执行中'
            return res
        if not self._all_poses:
            res.success = False
            res.message = '路径为空，请检查 path_file 参数'
            return res
        self._reset_state()
        threading.Thread(target=self._run, daemon=True).start()
        res.success = True
        res.message = f'开始覆盖导航，共 {self._total} 个路径点'
        return res

    def _svc_pause(self, req, res):
        if not self._running or self._paused:
            res.success = False
            res.message = '当前未运行或已暂停'
            return res
        self._paused = True
        self._cancel_current_goal()
        self._publish_status('PAUSED')
        res.success = True
        res.message = f'已暂停，当前进度 {self._resume_index}/{self._total}'
        return res

    def _svc_resume(self, req, res):
        if not self._running or not self._paused:
            res.success = False
            res.message = '当前未暂停'
            return res
        self._paused = False
        self._publish_status('RUNNING')
        res.success = True
        res.message = f'继续执行，从第 {self._resume_index} 个路径点继续'
        return res

    def _svc_cancel(self, req, res):
        self._cancelled = True
        self._paused    = False
        self._cancel_current_goal()
        self._publish_status('CANCELLED')
        res.success = True
        res.message = '任务已取消'
        return res

    # ── 执行主循环 ─────────────────────────────────────────────────────────────

    def _run(self) -> None:
        """覆盖路径执行主体，运行在独立线程中。"""
        self._running    = True
        self._start_time = time.monotonic()
        self._publish_status('RUNNING')

        # 等待 Nav2 action server 就绪
        self.get_logger().info('等待 navigate_through_poses action server...')
        if not self._nav_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('等待超时，请确认 Nav2 已启动并激活')
            self._finish(success=False)
            return
        self.get_logger().info('Nav2 已就绪，开始导航')

        while self._resume_index < self._total and not self._cancelled:
            # 等待 resume
            while self._paused and not self._cancelled:
                time.sleep(0.1)

            if self._cancelled:
                break

            # 发送本次 action：从 resume_index 到末尾
            remaining_poses = self._all_poses[self._resume_index:]
            self._sent_count    = len(remaining_poses)
            self._last_remaining = self._sent_count

            result = self._send_and_wait(remaining_poses)

            if result == 'SUCCEEDED':
                self._resume_index = self._total     # 全部完成
                break

            elif result == 'PAUSED':
                # 计算实际走到哪里了
                done = self._sent_count - self._last_remaining
                self._resume_index += done
                self.get_logger().info(
                    f'已暂停，进度 {self._resume_index}/{self._total}，等待继续指令...'
                )
                # 继续外层循环 → 等待 _paused=False

            elif result == 'CANCELLED':
                break

            elif result == 'FAILED':
                done = self._sent_count - self._last_remaining
                failed_idx = self._resume_index + done
                if self._skip_on_failure:
                    self.get_logger().warn(
                        f'路径点 #{failed_idx} 导航失败，跳过继续执行...'
                    )
                    self._resume_index = failed_idx + 1
                    # 继续外层循环
                else:
                    self.get_logger().error(
                        f'路径点 #{failed_idx} 导航失败，任务终止'
                    )
                    self._finish(success=False)
                    return

        success = (self._resume_index >= self._total) and not self._cancelled
        self._finish(success=success)

    # ── Nav2 动作交互 ──────────────────────────────────────────────────────────

    def _send_and_wait(self, poses: List[PoseStamped]) -> str:
        """
        发送 NavigateThroughPoses 目标，阻塞直到结果返回。
        返回: 'SUCCEEDED' | 'FAILED' | 'PAUSED' | 'CANCELLED'
        """
        done_event = threading.Event()
        result_box = ['FAILED']

        def _on_goal_response(future):
            handle = future.result()
            if not handle or not handle.accepted:
                self.get_logger().error('目标被 Nav2 拒绝 (action server 未就绪？)')
                result_box[0] = 'FAILED'
                done_event.set()
                return
            with self._goal_lock:
                self._goal_handle = handle
            handle.get_result_async().add_done_callback(_on_result)

        def _on_result(future):
            r = future.result()
            status = r.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                result_box[0] = 'SUCCEEDED'
            elif status == GoalStatus.STATUS_CANCELED:
                # 区分"我们主动暂停"和"取消"
                result_box[0] = 'PAUSED' if self._paused else 'CANCELLED'
            else:
                result_box[0] = 'FAILED'
            done_event.set()

        def _on_feedback(msg):
            fb = msg.feedback
            self._last_remaining = fb.number_of_poses_remaining
            self._print_progress()

        # 发送目标
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        future = self._nav_client.send_goal_async(goal_msg, feedback_callback=_on_feedback)
        future.add_done_callback(_on_goal_response)

        # 阻塞等待，每 100ms 检查暂停/取消
        while not done_event.wait(timeout=0.1):
            if (self._paused or self._cancelled) and not done_event.is_set():
                self._cancel_current_goal()

        return result_box[0]

    def _cancel_current_goal(self) -> None:
        """取消当前正在执行的 goal，等待确认。"""
        with self._goal_lock:
            handle = self._goal_handle
            self._goal_handle = None          # 防止重复取消

        if handle is None:
            return

        cancelled_event = threading.Event()
        future = handle.cancel_goal_async()
        future.add_done_callback(lambda _: cancelled_event.set())
        cancelled_event.wait(timeout=5.0)

    # ── 进度显示 ───────────────────────────────────────────────────────────────

    def _print_progress(self) -> None:
        if self._total == 0:
            return

        done = self._resume_index + (self._sent_count - self._last_remaining)
        pct  = min(100.0, done / self._total * 100.0)
        elapsed = time.monotonic() - self._start_time

        eta_str = ''
        if pct > 0.5:
            remaining_time = elapsed / pct * (100.0 - pct)
            eta_str = f' | 预计剩余: {_fmt_time(remaining_time)}'

        bar_str = _progress_bar(pct)
        line = (
            f'\r{bar_str} | {done}/{self._total} pts'
            f' | 耗时: {_fmt_time(elapsed)}{eta_str}   '
        )
        print(line, end='', flush=True)

        # 发布到 ROS topic
        msg = Float32()
        msg.data = float(pct)
        self._pub_progress.publish(msg)

    # ── 完成处理 ───────────────────────────────────────────────────────────────

    def _finish(self, success: bool) -> None:
        print()   # 结束进度条行，换行
        self._running = False
        elapsed = time.monotonic() - self._start_time

        if success:
            self.get_logger().info(
                f'✓ 覆盖导航完成！{self._total} 个路径点，耗时 {_fmt_time(elapsed)}'
            )
            self._publish_status('DONE')
        else:
            self.get_logger().warn(
                f'✗ 覆盖导航结束 (失败/取消)，耗时 {_fmt_time(elapsed)}'
            )
            self._publish_status('FAILED')

        msg = Bool()
        msg.data = success
        self._pub_done.publish(msg)

    # ── 工具 ───────────────────────────────────────────────────────────────────

    def _reset_state(self) -> None:
        self._resume_index   = 0
        self._sent_count     = 0
        self._last_remaining = 0
        self._paused    = False
        self._cancelled = False
        self._running   = False
        self._goal_handle = None

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._pub_status.publish(msg)
        self.get_logger().info(f'[状态] {status}')

    def _autostart_once(self) -> None:
        self._autostart_timer.cancel()
        resp = Trigger.Response()
        self._svc_start(None, resp)


# ──────────────────────────────────────────────────────────────────────────────
# YAML 解析（独立函数，便于测试）
# ──────────────────────────────────────────────────────────────────────────────

def _parse_coverage_yaml(
    path_file: str,
    fallback_frame: str = 'map',
    logger=None,
) -> Tuple[List[PoseStamped], str]:
    """
    解析覆盖路径 YAML 文件，返回 (poses, frame_id)。

    支持格式:
        path:
          frame_id: map        # "undefined" 时回退到 fallback_frame
          poses:
            - position:    {x, y, z}
              orientation: {x, y, z, w}
    """
    p = Path(path_file)
    if not p.exists():
        raise FileNotFoundError(f'文件不存在: {path_file}')

    with open(p, 'r') as f:
        data = yaml.safe_load(f)

    # 定位 path 节 (兼容顶层 path 键或整个文件就是路径)
    path_section = data.get('path', data)
    poses_raw    = path_section.get('poses', [])

    if not poses_raw:
        raise ValueError('YAML 中未找到 poses 列表')

    # frame_id 处理
    yaml_frame = str(path_section.get('frame_id', 'undefined')).strip()
    if yaml_frame in ('undefined', '', 'null', 'None'):
        frame_id = fallback_frame
        if logger:
            logger.warn(
                f'YAML frame_id="{yaml_frame}"，使用参数 frame_id="{frame_id}"'
            )
    else:
        frame_id = yaml_frame

    poses: List[PoseStamped] = []
    for raw in poses_raw:
        ps  = PoseStamped()
        ps.header.frame_id = frame_id

        pos = raw.get('position', {})
        ori = raw.get('orientation', {})

        ps.pose.position.x    = float(pos.get('x', 0.0))
        ps.pose.position.y    = float(pos.get('y', 0.0))
        ps.pose.position.z    = float(pos.get('z', 0.0))
        ps.pose.orientation.x = float(ori.get('x', 0.0))
        ps.pose.orientation.y = float(ori.get('y', 0.0))
        ps.pose.orientation.z = float(ori.get('z', 0.0))
        ps.pose.orientation.w = float(ori.get('w', 1.0))

        poses.append(ps)

    return poses, frame_id


# ──────────────────────────────────────────────────────────────────────────────
# 入口
# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CoveragePathExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
