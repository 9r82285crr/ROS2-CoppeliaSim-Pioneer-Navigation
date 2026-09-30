#!/usr/bin/env python3
import math
import json
from collections import deque
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def finite_min(arr):
    vals = [x for x in arr if math.isfinite(x)]
    return min(vals) if vals else float('inf')


class GridStepNavigator(Node):
    def __init__(self):
        super().__init__('grid_step_navigator_bfs_v2')

        # =========================
        # Topics
        # =========================
        self.cmd_topic = '/cmd_vel'
        self.odom_topic = '/odom'
        self.scan_topic = '/scan'

        # =========================
        # 状态机
        # PLAN_NEXT / ALIGN_TO_STEP / MOVE_TO_STEP
        # =========================
        self.state = 'PLAN_NEXT'

        # =========================
        # 原始目标 waypoint（世界坐标, 单位 m）
        # =========================
        self.base_waypoints: List[Tuple[float, float]] = [
            (1.0, 0.0),
            (1.0, 1.0),
            (2.0, 1.0),
        ]
        self.current_wp_idx = 0

        # 当前一步格点目标
        self.next_cell_goal: Optional[Tuple[int, int]] = None

        # =========================
        # 栅格地图
        # =========================
        self.resolution = 0.22
        self.grid_width = 220
        self.grid_height = 220

        self.grid_unknown = -1
        self.grid_free = 0
        self.grid_occ = 100

        self.grid = [
            [self.grid_unknown for _ in range(self.grid_width)]
            for _ in range(self.grid_height)
        ]

        # 出发点作为逻辑原点，放在地图中心
        self.grid_origin_i = self.grid_width // 2
        self.grid_origin_j = self.grid_height // 2

        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        # 地图保存
        self.map_save_path = 'grid_map.json'
        self.last_map_save_time = 0.0
        self.map_save_interval = 5.0

        # =========================
        # 运动参数
        # =========================
        self.forward_speed = 0.10
        self.turn_kp = 1.8
        self.turn_max_speed = 0.85
        self.turn_min_speed = 0.12
        self.turn_stop_tolerance = 0.02

        self.axis_yaw_tolerance = 0.05
        self.cell_center_tolerance = 0.06

        # =========================
        # 前方障碍检测
        # front_half_angle_rad 现在主要用于建图扇区
        # 前方测距已改为 scan.py 的中心切片逻辑
        # =========================
        self.front_half_angle_rad = math.radians(5.0)
        self.stop_threshold = 0.29
        self.resume_threshold = 0.30

        self.front_filtered = float('inf')
        self.front_filter_alpha = 0.65
        self.latest_front_min = float('inf')
        self.obstacle_blocked = False

        # =========================
        # 当前机器人状态
        # =========================
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        # 当前目标朝向（对准下一格）
        self.target_yaw = None

        # 上一步格点，防止来回振荡
        self.last_cell: Optional[Tuple[int, int]] = None

        # 最近失败格点记忆
        self.recently_blocked_cells = {}
        self.block_memory_duration = 55.0

        # 最近经过路径，作为 BFS 失败时的贪心惩罚项
        self.recent_cells: List[Tuple[int, int]] = []
        self.recent_cells_maxlen = 8

        # =========================
        # ROS2
        # =========================
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Grid step navigator BFS v2 started.')

    # ======================================================
    # 基础函数
    # ======================================================
    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def set_start_pose_if_needed(self):
        if self.start_x is None and self.robot_x is not None:
            self.start_x = self.robot_x
            self.start_y = self.robot_y
            self.start_yaw = self.robot_yaw
            self.get_logger().info(
                f'Start pose set as grid origin: ({self.start_x:.3f}, {self.start_y:.3f})'
            )

    # ======================================================
    # 网格映射
    # ======================================================
    def world_to_grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if self.start_x is None or self.start_y is None:
            return None

        gx = int(round((x - self.start_x) / self.resolution)) + self.grid_origin_i
        gy = int(round((y - self.start_y) / self.resolution)) + self.grid_origin_j

        if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
            return gx, gy
        return None

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        x = self.start_x + (gx - self.grid_origin_i) * self.resolution
        y = self.start_y + (gy - self.grid_origin_j) * self.resolution
        return x, y

    def mark_cell(self, gx: int, gy: int, value: int):
        if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
            self.grid[gy][gx] = value

    def mark_world(self, x: float, y: float, value: int):
        idx = self.world_to_grid(x, y)
        if idx is None:
            return
        gx, gy = idx
        self.mark_cell(gx, gy, value)

    def mark_robot_cell_free(self):
        if self.robot_x is None or self.robot_y is None:
            return
        idx = self.world_to_grid(self.robot_x, self.robot_y)
        if idx is not None:
            gx, gy = idx
            self.mark_cell(gx, gy, self.grid_free)

    def save_grid_map(self):
        if self.start_x is None:
            return

        data = {
            "resolution": self.resolution,
            "grid_width": self.grid_width,
            "grid_height": self.grid_height,
            "grid_origin_i": self.grid_origin_i,
            "grid_origin_j": self.grid_origin_j,
            "start_x": self.start_x,
            "start_y": self.start_y,
            "start_yaw": self.start_yaw,
            "grid": self.grid,
        }

        try:
            with open(self.map_save_path, 'w', encoding='utf-8') as f:
                json.dump(data, f)
        except Exception as e:
            self.get_logger().warn(f'Failed to save grid map: {e}')

    # ======================================================
    # 目标 / 邻居 / 规划
    # ======================================================
    def current_base_goal_world(self) -> Optional[Tuple[float, float]]:
        if self.current_wp_idx >= len(self.base_waypoints):
            return None
        return self.base_waypoints[self.current_wp_idx]

    def current_base_goal_grid(self) -> Optional[Tuple[int, int]]:
        goal = self.current_base_goal_world()
        if goal is None:
            return None
        return self.world_to_grid(goal[0], goal[1])

    def current_robot_cell(self) -> Optional[Tuple[int, int]]:
        if self.robot_x is None or self.robot_y is None:
            return None
        return self.world_to_grid(self.robot_x, self.robot_y)

    def get_4_neighbors(self, gx: int, gy: int) -> List[Tuple[int, int]]:
        return [
            (gx + 1, gy),
            (gx - 1, gy),
            (gx, gy + 1),
            (gx, gy - 1),
        ]

    def is_in_grid(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.grid_width and 0 <= gy < self.grid_height

    def is_recently_blocked(self, gx: int, gy: int) -> bool:
        key = (gx, gy)
        if key not in self.recently_blocked_cells:
            return False

        t = self.recently_blocked_cells[key]
        if self.now_sec() - t > self.block_memory_duration:
            del self.recently_blocked_cells[key]
            return False
        return True

    def is_cell_blocked(self, gx: int, gy: int) -> bool:
        if not self.is_in_grid(gx, gy):
            return True

        if self.is_recently_blocked(gx, gy):
            return True

        value = self.grid[gy][gx]
        return value == self.grid_occ

    def manhattan_distance(self, a: Tuple[int, int], b: Tuple[int, int]) -> int:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def bfs_next_cell(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        if start == goal:
            return start

        q = deque([start])
        parent = {start: None}
        found = False

        while q:
            cur = q.popleft()

            if cur == goal:
                found = True
                break

            nbs = self.get_4_neighbors(cur[0], cur[1])
            nbs.sort(key=lambda nb: self.manhattan_distance(nb, goal))

            for nb in nbs:
                if nb in parent:
                    continue

                gx, gy = nb
                if self.is_cell_blocked(gx, gy):
                    continue

                parent[nb] = cur
                q.append(nb)

        if not found:
            return None

        path = []
        cur = goal
        while cur is not None:
            path.append(cur)
            cur = parent[cur]
        path.reverse()

        if len(path) >= 2:
            return path[1]

        return None

    def choose_next_cell(
        self,
        current_cell: Tuple[int, int],
        goal_cell: Tuple[int, int]
    ) -> Optional[Tuple[int, int]]:
        neighbors = self.get_4_neighbors(current_cell[0], current_cell[1])

        dgx = goal_cell[0] - current_cell[0]
        dgy = goal_cell[1] - current_cell[1]
        preferred_axis = 'x' if abs(dgx) >= abs(dgy) else 'y'

        scored = []

        for nb in neighbors:
            gx, gy = nb
            if self.is_cell_blocked(gx, gy):
                continue

            step_dx = gx - current_cell[0]
            step_dy = gy - current_cell[1]

            score = self.manhattan_distance(nb, goal_cell)

            if self.last_cell is not None and nb == self.last_cell:
                score += 10

            if nb in self.recent_cells:
                score += 8

            if preferred_axis == 'x':
                if dgx > 0:
                    if step_dx == -1:
                        score += 6
                    if step_dy != 0:
                        score -= 2
                else:
                    if step_dx == 1:
                        score += 6
                    if step_dy != 0:
                        score -= 2
            else:
                if dgy > 0:
                    if step_dy == -1:
                        score += 6
                    if step_dx != 0:
                        score -= 2
                else:
                    if step_dy == 1:
                        score += 6
                    if step_dx != 0:
                        score -= 2

            scored.append((score, nb))

        if not scored:
            return None

        scored.sort(key=lambda x: x[0])
        return scored[0][1]

    # ======================================================
    # 朝向 / 前方格点
    # ======================================================
    def axis_yaw_from_cell_delta(self, from_cell: Tuple[int, int], to_cell: Tuple[int, int]) -> float:
        dx = to_cell[0] - from_cell[0]
        dy = to_cell[1] - from_cell[1]

        if dx == 1 and dy == 0:
            return 0.0
        if dx == -1 and dy == 0:
            return math.pi
        if dx == 0 and dy == 1:
            return math.pi / 2.0
        if dx == 0 and dy == -1:
            return -math.pi / 2.0

        return 0.0

    def heading_to_front_neighbor(self) -> Optional[Tuple[int, int]]:
        cur = self.current_robot_cell()
        if cur is None or self.robot_yaw is None:
            return None

        yaw = self.robot_yaw
        dirs = [
            (abs(normalize_angle(yaw - 0.0)), (cur[0] + 1, cur[1])),
            (abs(normalize_angle(yaw - math.pi)), (cur[0] - 1, cur[1])),
            (abs(normalize_angle(yaw - math.pi / 2.0)), (cur[0], cur[1] + 1)),
            (abs(normalize_angle(yaw + math.pi / 2.0)), (cur[0], cur[1] - 1)),
        ]
        dirs.sort(key=lambda x: x[0])
        return dirs[0][1]

    def mark_front_cell_blocked(self):
        # 优先：直接放弃当前正在尝试探索的前方目标格点
        if self.next_cell_goal is not None:
            gx, gy = self.next_cell_goal
            if self.is_in_grid(gx, gy):
                if self.grid[gy][gx] != self.grid_occ:
                    self.mark_cell(gx, gy, self.grid_occ)
                    self.get_logger().info(f'Marked blocked next cell: ({gx}, {gy})')
                self.recently_blocked_cells[(gx, gy)] = self.now_sec()
                return

        # 兜底：按当前朝向估计前方格点
        front_cell = self.heading_to_front_neighbor()
        if front_cell is None:
            return

        gx, gy = front_cell
        if not self.is_in_grid(gx, gy):
            return

        if self.grid[gy][gx] != self.grid_occ:
            self.mark_cell(gx, gy, self.grid_occ)
            self.get_logger().info(f'Marked blocked front cell: ({gx}, {gy})')

        self.recently_blocked_cells[(gx, gy)] = self.now_sec()

    # ======================================================
    # 回调
    # ======================================================
    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.robot_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        self.set_start_pose_if_needed()
        self.mark_robot_cell_free()

    def scan_callback(self, msg: LaserScan):
        # =========================
        # A. 保留原来的建图逻辑：遍历前方扇区并标记 occupied
        # =========================
        angle = msg.angle_min
        for r in msg.ranges:
            if -self.front_half_angle_rad <= angle <= self.front_half_angle_rad:
                if math.isfinite(r) and (msg.range_min < r < msg.range_max):
                    if (
                        self.robot_x is not None
                        and self.robot_y is not None
                        and self.robot_yaw is not None
                    ):
                        world_angle = self.robot_yaw + angle
                        obs_x = self.robot_x + r * math.cos(world_angle)
                        obs_y = self.robot_y + r * math.sin(world_angle)
                        self.mark_world(obs_x, obs_y, self.grid_occ)

            angle += msg.angle_increment

        # =========================
        # B. 用 scan.py 的方式替换“前方距离测算”
        # 直接按 scan 数组中心切片取前方区域
        # =========================
        n = len(msg.ranges)
        if n < 30:
            self.latest_front_min = float('inf')
            return

        c = n // 2
        front = msg.ranges[max(0, c - 50): min(n, c + 50)]

        front_min = finite_min(front)
        self.latest_front_min = front_min

        # =========================
        # C. 保留原来的滤波 + 阈值判定
        # =========================
        if math.isfinite(front_min):
            if math.isinf(self.front_filtered):
                self.front_filtered = front_min
            else:
                self.front_filtered = (
                    self.front_filter_alpha * self.front_filtered
                    + (1.0 - self.front_filter_alpha) * front_min
                )

        if self.front_filtered < self.stop_threshold:
            self.obstacle_blocked = True
        elif self.front_filtered > self.resume_threshold:
            self.obstacle_blocked = False

    # ======================================================
    # 控制
    # ======================================================
    def control_loop(self):
        if self.robot_x is None or self.robot_y is None or self.robot_yaw is None:
            return

        now = self.now_sec()
        if now - self.last_map_save_time > self.map_save_interval:
            self.save_grid_map()
            self.last_map_save_time = now

        goal_world = self.current_base_goal_world()
        goal_cell = self.current_base_goal_grid()
        current_cell = self.current_robot_cell()

        if goal_world is None or goal_cell is None or current_cell is None:
            self.stop_robot()
            return

        if math.hypot(goal_world[0] - self.robot_x, goal_world[1] - self.robot_y) < 0.18:
            self.get_logger().info(
                f'Reached base waypoint {self.current_wp_idx}: ({goal_world[0]:.2f}, {goal_world[1]:.2f})'
            )
            self.current_wp_idx += 1
            self.next_cell_goal = None
            self.state = 'PLAN_NEXT'

            if self.current_wp_idx >= len(self.base_waypoints):
                self.stop_robot()
                self.save_grid_map()
                self.get_logger().info('All waypoints reached. Grid map saved.')
            return

        # ===============================
        # PLAN_NEXT：优先 BFS 选下一格
        # ===============================
        if self.state == 'PLAN_NEXT':
            next_cell = self.bfs_next_cell(current_cell, goal_cell)

            if next_cell is None:
                next_cell = self.choose_next_cell(current_cell, goal_cell)

            if next_cell is None:
                self.stop_robot()
                self.get_logger().warn('No valid path / neighbor available.')
                return

            self.next_cell_goal = next_cell
            self.target_yaw = self.axis_yaw_from_cell_delta(current_cell, next_cell)
            self.state = 'ALIGN_TO_STEP'
            self.get_logger().info(
                f'PLAN_NEXT: current={current_cell}, goal={goal_cell}, next={next_cell}'
            )
            return

        # ===============================
        # ALIGN_TO_STEP：只负责转向，不处理障碍
        # ===============================
        if self.state == 'ALIGN_TO_STEP':
            if self.next_cell_goal is None or self.target_yaw is None:
                self.state = 'PLAN_NEXT'
                return

            error = normalize_angle(self.target_yaw - self.robot_yaw)
            cmd = Twist()

            if abs(error) < self.turn_stop_tolerance:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                self.state = 'MOVE_TO_STEP'
                return

            ang = self.turn_kp * error
            ang = clamp(ang, -self.turn_max_speed, self.turn_max_speed)

            if abs(ang) < self.turn_min_speed:
                ang = self.turn_min_speed * (1 if ang > 0 else -1)

            cmd.linear.x = 0.0
            cmd.angular.z = ang
            self.cmd_pub.publish(cmd)
            return

        # ===============================
        # MOVE_TO_STEP：真正前进时才处理障碍
        # 停止前进时，同时放弃当前目标格点
        # ===============================
        if self.state == 'MOVE_TO_STEP':
            if self.next_cell_goal is None:
                self.state = 'PLAN_NEXT'
                return

            if self.obstacle_blocked:
                self.stop_robot()
                self.mark_front_cell_blocked()
                self.next_cell_goal = None
                self.state = 'PLAN_NEXT'
                return

            target_x, target_y = self.grid_to_world(
                self.next_cell_goal[0], self.next_cell_goal[1]
            )
            dist = math.hypot(target_x - self.robot_x, target_y - self.robot_y)

            if dist < self.cell_center_tolerance:
                self.last_cell = current_cell

                self.recent_cells.append(current_cell)
                if len(self.recent_cells) > self.recent_cells_maxlen:
                    self.recent_cells.pop(0)

                self.mark_cell(self.next_cell_goal[0], self.next_cell_goal[1], self.grid_free)
                self.state = 'PLAN_NEXT'
                self.next_cell_goal = None
                self.stop_robot()
                return

            expected_yaw = self.axis_yaw_from_cell_delta(current_cell, self.next_cell_goal)
            yaw_error = normalize_angle(expected_yaw - self.robot_yaw)

            cmd = Twist()
            if abs(yaw_error) > self.axis_yaw_tolerance:
                self.target_yaw = expected_yaw
                self.state = 'ALIGN_TO_STEP'
                return
            else:
                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                return

        self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    node = GridStepNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.save_grid_map()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
