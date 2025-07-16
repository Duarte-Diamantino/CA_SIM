#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Joy
import numpy as np
import math
from scipy.optimize import minimize
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


# MPC parameters
dt = 0.010                       # time step (s)
horizon = 100                    # prediction horizon (steps)
V_MAX = 1.0                    # max speed (m/s)
V_MIN = 0.2                    # min speed (m/s)
GAMMA_MAX = math.pi / 8.0      # max steering angle (rad)
WHEEL_BASE = 0.335             # wheel base (m)

# Actuation mapping constants
MAX_RAW_SPEED_CMD = 23070.0    # raw speed units
TARGET_RPM = 5000.0            # target RPM to maintain
MIN_RPM = 4000.0               # minimum RPM allowed
SERVO_CENTER = 0.5304
SERVO_MAX    = 0.94299
SERVO_MIN    = 0.11781

# Cost weights - REBALANCED
w_cross_track = 10.0     # Increased cross track error weight
w_heading = 8.0          # Heading error
w_speed_maintain = 5.0   # Speed maintenance (reduced priority)
w_smooth_steer = 0.5     # Smooth steering changes
w_progress = 2.0         # NEW: Progress along path weight

# Path following parameters
lookahead_distance = 0.8    # Reduced lookahead distance
min_lookahead = 0.3         # Minimum lookahead distance

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
        )
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False
        self.ref_path = []
        self.last_gamma = 0.0
        self.last_velocity = 1.0
        self.target_raw_speed = TARGET_RPM / 23070.0 * MAX_RAW_SPEED_CMD
        self.min_raw_speed = MIN_RPM / 23070.0 * MAX_RAW_SPEED_CMD
        
        # NEW: Progress tracking
        self.current_path_index = 0
        self.path_progress = 0.0
        
        # Joy control
        self.joy_override = False
        
        # Subscribers and publishers
        self.create_subscription(
            Odometry, 
            '/odometry/filtered', 
            self.odom_cb, 
            qos_profile=qos_profile
        )

        self.create_subscription(Path, '/reference_path', self.path_cb, 10)
        self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        
        self.speed_pub = self.create_publisher(Float64, '/commands/motor/speed', 10)
        self.steer_pub = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/predicted_trajectories', 10)
        
        # Control timer
        self.create_timer(dt, self.update)
        
        self.get_logger().info('Fixed MPC Controller initialized.')

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z)
        )
        self.have_odom = True

    def joy_cb(self, msg: Joy):
        self.joy_override = any(button > 0 for button in msg.buttons)

    def path_cb(self, msg: Path):
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        pts.sort(key=lambda point: point[0])
        self.ref_path = pts
        self.current_path_index = 0  # Reset progress tracking
        self.path_progress = 0.0
        self.get_logger().info(f'Received {len(pts)} path points, ordered from x={pts[0][0]:.2f} to x={pts[-1][0]:.2f}')

    def find_closest_point_on_path_segment(self, x, y, p1, p2):
        """Find closest point on a line segment between p1 and p2"""
        x1, y1 = p1
        x2, y2 = p2
        
        # Vector from p1 to p2
        dx = x2 - x1
        dy = y2 - y1
        
        # If points are the same, return p1
        if dx == 0 and dy == 0:
            return x1, y1, 0.0
        
        # Parameter t for closest point on line
        t = ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)
        
        # Clamp t to [0, 1] to stay on segment
        t = max(0.0, min(1.0, t))
        
        # Calculate closest point
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        return closest_x, closest_y, t

    def find_closest_point_on_path(self, x, y):
        """Find closest point on the entire path (considering line segments)"""
        if not self.ref_path or len(self.ref_path) < 2:
            if self.ref_path:
                return self.ref_path[0][0], self.ref_path[0][1], 0
            return None, None, -1
        
        min_dist = float('inf')
        closest_point = None
        closest_segment_idx = 0
        
        for i in range(len(self.ref_path) - 1):
            p1 = self.ref_path[i]
            p2 = self.ref_path[i + 1]
            
            closest_x, closest_y, t = self.find_closest_point_on_path_segment(x, y, p1, p2)
            dist = math.hypot(x - closest_x, y - closest_y)
            
            if dist < min_dist:
                min_dist = dist
                closest_point = (closest_x, closest_y)
                closest_segment_idx = i
        
        if closest_point:
            return closest_point[0], closest_point[1], closest_segment_idx
        return None, None, -1

    def get_path_tangent_at_segment(self, segment_idx):
        """Get path tangent direction at a specific segment"""
        if not self.ref_path or len(self.ref_path) < 2:
            return 0.0
        
        # Ensure valid segment index
        segment_idx = max(0, min(segment_idx, len(self.ref_path) - 2))
        
        p1 = self.ref_path[segment_idx]
        p2 = self.ref_path[segment_idx + 1]
        
        tangent_angle = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        return tangent_angle

    def update_path_progress(self, current_x, current_y):
        """Update progress along path - only move forward"""
        if not self.ref_path or len(self.ref_path) < 2:
            return
        
        # Find current position relative to path
        closest_x, closest_y, segment_idx = self.find_closest_point_on_path(current_x, current_y)
        
        if segment_idx >= 0:
            # Only update if we're progressing forward or staying close
            if segment_idx >= self.current_path_index:
                self.current_path_index = segment_idx
                
                # Calculate progress within current segment
                if segment_idx < len(self.ref_path) - 1:
                    p1 = self.ref_path[segment_idx]
                    p2 = self.ref_path[segment_idx + 1]
                    
                    # Calculate how far along the segment we are
                    segment_length = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
                    if segment_length > 0:
                        progress_in_segment = math.hypot(closest_x - p1[0], closest_y - p1[1]) / segment_length
                        self.path_progress = segment_idx + progress_in_segment
                    else:
                        self.path_progress = segment_idx
                else:
                    self.path_progress = len(self.ref_path) - 1

    def find_lookahead_point(self, current_x, current_y):
        """Find lookahead point - always move forward along path"""
        if not self.ref_path:
            return None, None, 0
        
        # Update progress first
        self.update_path_progress(current_x, current_y)
        
        # Start looking from current progress point
        start_idx = max(0, int(self.current_path_index))
        
        # Calculate dynamic lookahead distance
        dynamic_lookahead = max(min_lookahead, lookahead_distance)
        
        # Find point at lookahead distance ahead
        for i in range(start_idx, len(self.ref_path)):
            px, py = self.ref_path[i]
            dist = math.hypot(current_x - px, current_y - py)
            
            if dist >= dynamic_lookahead:
                return px, py, i
        
        # If no point found at lookahead distance, use last point
        if self.ref_path:
            px, py = self.ref_path[-1]
            return px, py, len(self.ref_path) - 1
        
        return None, None, 0

    def simulate_trajectory(self, v, gamma):
        """Simulate vehicle trajectory"""
        traj = []
        x, y, yaw = self.x, self.y, self.yaw
        
        for _ in range(horizon):
            traj.append((x, y, yaw))
            # Bicycle model
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw += (v / WHEEL_BASE) * math.tan(gamma) * dt
            yaw = self.normalize_angle(yaw)
        
        return traj

    def cross_track_error(self, x, y):
        """Calculate cross track error - distance to closest point on path"""
        if not self.ref_path:
            return float('inf')
        
        closest_x, closest_y, _ = self.find_closest_point_on_path(x, y)
        if closest_x is not None:
            return math.hypot(x - closest_x, y - closest_y)
        
        return float('inf')

    def heading_error_to_path_tangent(self, x, y, current_yaw):
        """Calculate heading error relative to path tangent"""
        if not self.ref_path or len(self.ref_path) < 2:
            return 0.0
        
        # Get closest segment
        _, _, segment_idx = self.find_closest_point_on_path(x, y)
        
        if segment_idx >= 0:
            path_tangent = self.get_path_tangent_at_segment(segment_idx)
            heading_error = self.normalize_angle(current_yaw - path_tangent)
            return heading_error
        
        return 0.0

    def progress_reward(self, trajectory):
        """Calculate reward for making progress along path"""
        if not self.ref_path or len(trajectory) == 0:
            return 0.0
        
        # Use the final position of the trajectory
        final_x, final_y, _ = trajectory[-1]
        
        # Calculate progress made
        current_progress = self.path_progress
        
        # Find what progress would be at final position
        closest_x, closest_y, segment_idx = self.find_closest_point_on_path(final_x, final_y)
        
        if segment_idx >= 0:
            future_progress = segment_idx
            if segment_idx < len(self.ref_path) - 1:
                p1 = self.ref_path[segment_idx]
                p2 = self.ref_path[segment_idx + 1]
                segment_length = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
                if segment_length > 0:
                    progress_in_segment = math.hypot(closest_x - p1[0], closest_y - p1[1]) / segment_length
                    future_progress = segment_idx + progress_in_segment
            
            # Reward forward progress, penalize backward movement
            progress_delta = future_progress - current_progress
            return -progress_delta  # Negative because we minimize cost
        
        return 0.0

    def cost_function(self, control_inputs):
        """Enhanced cost function with better path following"""
        v, gamma = control_inputs
        
        # Convert v to RPM equivalent for constraint checking
        v_rpm = v / V_MAX * 23070.0
        
        # Hard constraints
        if v_rpm < MIN_RPM or v_rpm > 23070.0 or abs(gamma) > GAMMA_MAX:
            return 1e6
        
        trajectory = self.simulate_trajectory(v, gamma)
        total_cost = 0.0
        
        # Cross track error and heading error costs
        cross_track_cost = 0.0
        heading_cost = 0.0
        
        for i, (x, y, yaw) in enumerate(trajectory):
            # Cross track error - higher weight for trajectory points
            cte = self.cross_track_error(x, y)
            weight_factor = math.exp(-0.02 * i)  # Decay over horizon
            cross_track_cost += weight_factor * (cte * cte)
            
            # Heading error relative to path tangent
            heading_error = self.heading_error_to_path_tangent(x, y, yaw)
            heading_cost += weight_factor * (heading_error * heading_error)
        
        total_cost += w_cross_track * cross_track_cost
        total_cost += w_heading * heading_cost
        
        # Speed maintenance cost
        target_speed = self.target_raw_speed / MAX_RAW_SPEED_CMD * V_MAX
        speed_error = v - target_speed
        total_cost += w_speed_maintain * (speed_error * speed_error)
        
        # Smooth steering cost
        steer_change_cost = (gamma - self.last_gamma) ** 2
        total_cost += w_smooth_steer * steer_change_cost
        
        # Progress reward (NEW)
        progress_cost = self.progress_reward(trajectory)
        total_cost += w_progress * progress_cost
        
        return total_cost

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def publish_markers(self, trajectory, target_x, target_y, closest_x, closest_y):
        """Publish visualization markers"""
        ma = MarkerArray()
        marker_id = 0
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.header.frame_id = 'map'
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.action = Marker.DELETEALL
        ma.markers.append(clear_marker)
        marker_id += 1
        
        # Future trajectory (yellow spheres)
        for i, (x, y, yaw) in enumerate(trajectory):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'predicted_trajectory'
            m.id = marker_id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=x, y=y, z=0.0)
            m.scale.x = m.scale.y = m.scale.z = 0.1
            m.color.a = 0.7
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 0.0
            ma.markers.append(m)
            marker_id += 1
        
        # Current vehicle position (red arrow)
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'vehicle'
        m.id = marker_id
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position = Point(x=self.x, y=self.y, z=0.1)
        m.pose.orientation.z = math.sin(self.yaw / 2.0)
        m.pose.orientation.w = math.cos(self.yaw / 2.0)
        m.scale.x = 0.5
        m.scale.y = 0.1
        m.scale.z = 0.1
        m.color.a = 1.0
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        ma.markers.append(m)
        marker_id += 1
        
        # Target point (blue sphere)
        if target_x is not None and target_y is not None:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'target'
            m.id = marker_id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=target_x, y=target_y, z=0.1)
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color.a = 1.0
            m.color.r = 0.0
            m.color.g = 0.0
            m.color.b = 1.0
            ma.markers.append(m)
            marker_id += 1
        
        # Closest point on path (purple sphere)
        if closest_x is not None and closest_y is not None:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'closest_point'
            m.id = marker_id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=closest_x, y=closest_y, z=0.05)
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 1.0  # Purple
            ma.markers.append(m)
            marker_id += 1
        
        # Path points (green small spheres)
        for i, (px, py) in enumerate(self.ref_path[::3]):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'reference_path'
            m.id = marker_id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=px, y=py, z=0.0)
            m.scale.x = m.scale.y = m.scale.z = 0.05
            m.color.a = 0.5
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            ma.markers.append(m)
            marker_id += 1
        
        self.marker_pub.publish(ma)

    def update(self):
        """Main control loop - FIXED"""
        if not self.have_odom or not self.ref_path:
            return
        
        # Check joy override
        if self.joy_override:
            return
        
        # Find target point (using improved lookahead)
        target_x, target_y, target_idx = self.find_lookahead_point(self.x, self.y)
        
        if target_x is None:
            self.get_logger().warn('No target point found')
            return
        
        # Find closest point on path for visualization and error calculation
        closest_x, closest_y, closest_segment = self.find_closest_point_on_path(self.x, self.y)
        
        # Calculate current cross track error
        current_cte = self.cross_track_error(self.x, self.y)
        
        # Calculate current heading error relative to path tangent
        current_heading_error = self.heading_error_to_path_tangent(self.x, self.y, self.yaw)
        
        # Initial guess
        initial_v = max(V_MIN, min(V_MAX, self.last_velocity))
        initial_guess = [initial_v, self.last_gamma * 0.8]
        
        # Bounds
        v_min_ms = MIN_RPM / 23070.0 * V_MAX
        v_max_ms = V_MAX
        bounds = [(v_min_ms, v_max_ms), (-GAMMA_MAX, GAMMA_MAX)]
        
        # Optimize
        try:
            result = minimize(
                self.cost_function,
                initial_guess,
                method='L-BFGS-B',
                bounds=bounds,
                options={'maxiter': 50, 'ftol': 1e-4}
            )
            
            if result.success and result.fun < 1e5:
                best_v, best_gamma = result.x
            else:
                self.get_logger().warn(f'Optimization failed, using safe fallback')
                best_v = self.target_raw_speed / MAX_RAW_SPEED_CMD * V_MAX
                heading_to_target = math.atan2(target_y - self.y, target_x - self.x)
                heading_error = self.normalize_angle(heading_to_target - self.yaw)
                best_gamma = max(-GAMMA_MAX, min(GAMMA_MAX, heading_error * 0.5))
        
        except Exception as e:
            self.get_logger().error(f'Optimization error: {e}')
            best_v = self.target_raw_speed / MAX_RAW_SPEED_CMD * V_MAX
            best_gamma = self.last_gamma * 0.5
        
        # Apply smoothing
        smooth_alpha = 0.3
        final_v = smooth_alpha * self.last_velocity + (1 - smooth_alpha) * best_v
        final_gamma = smooth_alpha * self.last_gamma + (1 - smooth_alpha) * best_gamma
        
        # Ensure minimum speed
        final_v = max(V_MIN, final_v)
        
        # Generate trajectory for visualization
        trajectory = self.simulate_trajectory(final_v, final_gamma)
        
        # Publish control commands
        raw_speed = max(self.min_raw_speed, min(self.target_raw_speed, TARGET_RPM / 23070.0 * MAX_RAW_SPEED_CMD))
        raw_speed = max(-MAX_RAW_SPEED_CMD, min(raw_speed, MAX_RAW_SPEED_CMD))
        speed_msg = Float64(data=raw_speed)
        self.speed_pub.publish(speed_msg)
        
        # Steering
        gamma_corrected = -final_gamma if final_v >= 0.0 else final_gamma
        servo_cmd = SERVO_CENTER + (gamma_corrected / GAMMA_MAX) * (SERVO_MAX - SERVO_CENTER)
        servo_cmd = max(SERVO_MIN, min(servo_cmd, SERVO_MAX))
        steer_msg = Float64(data=servo_cmd)
        self.steer_pub.publish(steer_msg)
        
        # Logging
        actual_rpm = raw_speed / MAX_RAW_SPEED_CMD * 23070.0
        self.get_logger().info(
            f'CTE: {current_cte:.3f}m, HeadErr: {math.degrees(current_heading_error):.1f}°, '
            f'Progress: {self.path_progress:.1f}/{len(self.ref_path)-1}, '
            f'Target: ({target_x:.2f},{target_y:.2f}), '
            f'RPM: {actual_rpm:.0f}, γ: {final_gamma:.3f}rad'
        )
        
        # Publish markers
        self.publish_markers(trajectory, target_x, target_y, closest_x, closest_y)
        
        # Update for next iteration
        self.last_velocity = final_v 
        self.last_gamma = final_gamma


def main(args=None):
    rclpy.init(args=args)
    node = MPCController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
