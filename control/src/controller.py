#!/usr/bin/env python3

print('File Start')
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion, Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from scipy.spatial.transform import Rotation 
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import tf2_geometry_msgs
#import the chosen callback group type from rclpy.executors
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float32
import math
import time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control.action import FollowPath
from sensor_msgs.msg import LaserScan


def quat_to_yaw(q: Quaternion):
    # convert a Quaternion to a yaw angle
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('ZYX')[0]

def angle_wrap(a: float) -> float:
    # wrap an angle to -pi to pi
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class Controller(Node):
    def __init__(self):

        # Initialize node, name it 'controller'
        super().__init__('controller')

        self.odom_frame = 'odom'
        self.base_frame = 'base_link'

        # vars to keep track of the pos
        self.x: float = 0.0
        self.y: float = 0.0
        self.yaw: float = 0.0

        self.pos_goal_tolerance = 0.08     # [m] "close enough" for position
        self.yaw_goal_tolerance = math.radians(10.0)  # [rad] "close enough" for yaw

        self.debug = True #True

        self.cg_group = ReentrantCallbackGroup()

        qos_scan = QoSProfile(depth=10)
        qos_scan.reliability = QoSReliabilityPolicy.BEST_EFFORT

        #Subscribe to the '/odom' topic, call self.update.odometry when messages are published to that topic
        self.get_logger().info('Initializing')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.update_odometry, 20, callback_group=self.cg_group)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_scan)
        print("initialized odom")

        #Subscribe to the '/nav_path' topic that contains Path messages published by the path_generator node
        #self.path_sub = self.create_subscription(Path, '/nav_path', self.handle_path, 10, callback_group=self.cg_group)

        #Subscribe to the '/move_base_simple/goal' topic that contains PoseStamped messages published by RVIZ
        self.goal_sub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.go_to, 10, callback_group=self.cg_group)

        # Publish TwistStamped messages to the '/cmd_vel' topic.
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.goal_pub = self.create_publisher(PoseStamped, '/go_to_pose', 10)

        self.turn_error_pub = self.create_publisher(Float32, '/controller/turn_error', 10)
        self.dist_error_pub = self.create_publisher(Float32, '/controller/dist_error', 10)

        #action
        self.follow_path_action_server = ActionServer(
            self,
            FollowPath,
            'follow_path',
            execute_callback=self.execute_follow_path,
            goal_callback=self.follow_path_goal_callback,
            cancel_callback=self.follow_path_cancel_callback,
            callback_group=self.cg_group,
        )

        self.follow_path_goal_active = False
        self.estop_triggered_in_goal = False

        self.estop_enabled = self.declare_parameter(
            'estop_enabled', True
        ).value
        self.estop_distance = self.declare_parameter(
            'estop_distance', 0.18  # meters in front of robot
        ).value
        self.estop_angle_window_deg = self.declare_parameter(
            'estop_angle_window_deg', 30.0  # +/- degrees around front
        ).value
        self.estop_angle_deadband_deg = self.declare_parameter(
            'estop_angle_deadband_deg', 5.0  # +/- 5 deg counted as "aligned"
        ).value
        self.estop_turn_speed = self.declare_parameter(
            'estop_turn_speed', 0.4  # rad/s while turning toward clear space
        ).value
        self.estop_backup_speed = self.declare_parameter(
            'estop_backup_speed', 0.03  # m/s backwards
        ).value
        self.estop_center_tolerance = self.declare_parameter(
            'estop_center_tolerance', 0.05
        ).value

        self.estop_active = False
        self.min_front_distance = 10000000.0

        self.best_clear_angle = 0.0
        self.have_scan = False

        # Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def dbg(self, msg):
        if self.debug:
            self.get_logger().info(msg)

    def update_odometry(self, msg: Odometry):
        '''
        A callback that updates the current pose of the robot
        :param msg [Odometry] the current odometry information
        '''
        # Grab the data of the current pose
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = quat_to_yaw(msg.pose.pose.orientation)

        # if int(time.time()*10) % 10 == 0:
        #     self.dbg(f"ODOM UPDATE → x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.1f}deg")

        pose = PoseStamped()
        pose.header.frame_id = self.odom_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation = msg.pose.pose.orientation


    def send_speed(self, linear_speed: float, angular_speed: float):
        """
        Sends speed to the /cmd_vel topic, which runs the turtlebot motors.
        """
        if self.estop_enabled and self.estop_active:
            # If we don't have a scan yet, safest is to fully stop
            if not self.have_scan:
                linear_speed = 0.0
                angular_speed = 0.0
            else:
                # Move in the direction of largest free space:
                # rotate toward best_clear_angle
                # no forward motion while in E-STOP
                linear_speed = -abs(self.estop_backup_speed)
                angular_speed = 0.0

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame

        msg.twist.linear.x = float(linear_speed)
        msg.twist.angular.z = float(angular_speed)

        self.cmd_vel_pub.publish(msg)



    
    def drive(self, distance: float, linear_speed: float):
        '''
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        '''
        
        # record start pos
        start_x = self.x
        start_y = self.y

        # record for forward or backwards
        if distance >= 0.0:
            direction = 1.0  
        else: 
            direction =-1.0

        # account for forwards or backwards
        commanded_speed = linear_speed * direction
        goal_dist = abs(distance)

        while rclpy.ok():
            # calculate how far from target (euclidian dist.)
            dx = self.x - start_x
            dy = self.y - start_y
            traveled = math.hypot(dx, dy)

            # calculate remaining dist. to goal making sure it doesn't go negative
            remaining = max(0.0, goal_dist - traveled)

            # check if current pose is within 1mm of goal
            if remaining <= 1e-3:
                break

            # maintain proportional control to get to the goal
            v = commanded_speed
            if remaining < 0.10:
                v = max(0.02, remaining * 0.6) 

            self.send_speed(v, 0.0)

            # try not to loop too fast
            time.sleep(0.01)

        # stop robot
        self.send_speed(0.0, 0.0)
        

    def rotate(self, angle: float, angular_speed: float):
        '''
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        '''
        # store starting position of the robot
        start_yaw = self.yaw
        target_yaw = angle_wrap(start_yaw + angle)

        # check if angular speed is entered wrong
        if angular_speed != 0.0:
            max_speed = abs(angular_speed) 
        else:
            max_speed = 0.8
            
        while rclpy.ok():
            # calculate error
            err = angle_wrap(target_yaw - self.yaw)
            # check if error is within 1 radian of goal
            if abs(err) < math.radians(1.0):
                break

            # implementing proportionak control
            w = max(-max_speed, min(max_speed, 2.0 * err))
            if abs(err) < math.radians(10):
                w = math.copysign(max(math.radians(5), abs(err)), w)

            # use above function to moive robot
            self.send_speed(0.0, w)
            # try not to loop too fast
            time.sleep(0.01)

        # stop robot
        self.send_speed(0.0, 0.0)

    def go_to(self, msg: PoseStamped):
        '''
        Uses rotate() and drive() to get to a specific pose.
        :param msg [PoseStamped] The target or "goal" pose.
        '''

        try:
            # transfer from the original frame to the odom frame
            goal_odom = self.tf_buffer.transform(msg, self.odom_frame, timeout=Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().warn(f'failed to transform goal to {self.odom_frame}: {e}')
            return

        # store goal information
        gx = goal_odom.pose.position.x
        gy = goal_odom.pose.position.y
        gyaw = quat_to_yaw(goal_odom.pose.orientation)

        self.dbg(f"GOAL (odom) → gx={gx:.3f}, gy={gy:.3f}, gyaw={math.degrees(gyaw):.1f}deg")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.odom_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = gx
        goal_pose.pose.position.y = gy
        goal_pose.pose.orientation = goal_odom.pose.orientation
        
        self.goal_pub.publish(goal_pose)

        # store current information
        dx = gx - self.x
        dy = gy - self.y
        heading_to_goal = math.atan2(dy, dx)

        # rotate towards goal
        self.dbg(f"Initial heading_to_goal={math.degrees(heading_to_goal):.1f}deg")
        self.smooth_rotate(angle_wrap(heading_to_goal - self.yaw), angular_speed=0.35, heading_tol=30.0)

        # calculate Euclidian distance to goal
        dist = math.hypot(dx, dy)
        # drive towards goal
        self.dbg(f"Driving distance {dist:.3f} m toward goal")
        self.smooth_drive(distance=dist, linear_speed=0.10, heading_target=heading_to_goal) #0.22m/s is the robots linear max
        # rotate towards end pose
        #self.smooth_rotate(angle_wrap(gyaw - self.yaw), angular_speed=0.6)
        self.dbg("Final orientation alignment complete")
        self.get_logger().info('Reached goal.')
    
    def handle_path(self, path:Path):
        '''
        A callback function that handles iterating through a path.
        :param path [Path] The path that the robot will drive.
        '''
        # if no poses recieved log that
        if not path.poses:
            self.get_logger().warn('Received empty path.')
            return

        # if recieved then say so and use go to to interate through the poses
        self.get_logger().info(f'Received path with {len(path.poses)} poses.')
        for i, pose in enumerate(path.poses):
            x_way = round(path.poses[i].pose.position.x, 3)
            y_way = round(path.poses[i].pose.position.y, 3) 
            self.get_logger().info(f'Waypoint ({x_way ,y_way}) {i + 1}/{len(path.poses)}')
            self.go_to(pose)
        self.get_logger().info('Completed path.')
    
    async def execute_follow_path(self, goal_handle):
        self.get_logger().info("Executing FollowPath goal")

        path = goal_handle.request.path
        result = FollowPath.Result()

        # Mark this goal as active & clear the E-STOP flag for this goal
        self.follow_path_goal_active = True
        self.estop_triggered_in_goal = False

        if not path.poses:
            self.get_logger().warn("FollowPath goal has empty path")
            result.success = False
            result.message = "Empty path"
            goal_handle.abort()
            self.follow_path_goal_active = False
            return result

        for i, pose in enumerate(path.poses):
            # If E-STOP happened at any point in this goal, abort
            if self.estop_triggered_in_goal:
                self.get_logger().warn(
                    "FollowPath: aborting goal because E-STOP was triggered."
                )
                result.success = False
                result.message = "Aborted due to E-STOP"
                goal_handle.abort()
                self.follow_path_goal_active = False
                return result

            # Check cancel request
            if goal_handle.is_cancel_requested:
                self.get_logger().info("FollowPath goal canceled by client")
                goal_handle.canceled()
                result.success = False
                result.message = "Goal canceled"
                self.follow_path_goal_active = False
                return result

            # publish feedback
            feedback = FollowPath.Feedback()
            feedback.current_index = i
            feedback.current_target = pose
            goal_handle.publish_feedback(feedback)

            x_waypoint = round(pose.pose.position.x, 3)
            y_waypoint = round(pose.pose.position.y, 3)
            self.get_logger().info(
                f"FollowPath: waypoint {i+1}/{len(path.poses)} at ({x_waypoint}, {y_waypoint})"
            )

            # Actually move to this waypoint
            self.go_to(pose)

            # Check again *after* go_to, in case E-STOP hit mid-move
            if self.estop_triggered_in_goal:
                self.get_logger().warn(
                    "FollowPath: E-STOP triggered while driving to waypoint; aborting."
                )
                result.success = False
                result.message = "Aborted due to E-STOP"
                goal_handle.abort()
                self.follow_path_goal_active = False
                return result

        self.get_logger().info("FollowPath goal completed successfully")
        result.success = True
        result.message = "Path completed"
        goal_handle.succeed()
        self.follow_path_goal_active = False
        return result


    def follow_path_goal_callback(self, goal_request: FollowPath.Goal):
        self.get_logger().info(
            f"Received FollowPath goal with {len(goal_request.path.poses)} poses"
        )
        return GoalResponse.ACCEPT


    def follow_path_cancel_callback(self, goal_handle):
        self.get_logger().info("Received FollowPath cancel request")
        return CancelResponse.ACCEPT

    def smooth_drive(self, distance: float, linear_speed: float, heading_target=None):
        """
        Smoothly drives the robot in a straight line using a trapezoidal velocity profile.
        Uses loop-time-derived dt (no rclpy.Rate).
        :param distance     [float] [m]   The signed distance to cover.
        :param linear_speed [float] [m/s] The max forward linear speed (|vmax| used).
        """

        # motion control parameters
        drive_tol   = 0.03 # [m]
        a_max       = 0.1 # [m/s^2]
        v_min       = 0.02 # [m/s]
        dt_min      = 1e-4 # [s]

        # store initial pose
        start_x = self.x
        start_y = self.y
        # store forwards or backwardds
        direction = 1.0 if distance >= 0.0 else -1.0
        goal = abs(distance)
        vmax = abs(linear_speed)

        # heading control gains
        k_heading = 2.0
        w_max = 0.6

        v = 0.0
        t_prev = time.monotonic()

        while rclpy.ok():
            
            if self.estop_enabled and self.estop_triggered_in_goal and not self.estop_active:
                self.dbg("smooth_drive: E-STOP was triggered and is now cleared; aborting drive.")
                break

            # calculate Euclidian dist to target
            dx, dy = (self.x - start_x), (self.y - start_y)
            d = math.hypot(dx, dy)
            #remaining distance
            error = max(0.0, goal - d) 

            # if within threshold then break
            if error <= drive_tol:
                break

            msg = Float32()
            msg.data = float(error)
            self.dist_error_pub.publish(msg)

            # update current parameters
            t_now = time.monotonic()
            dt = t_now - t_prev
            t_prev = t_now

            # calculate breaking distance from current speed
            brake_dist = (v * v) / (2.0 * a_max) + v * dt

            # if breaking dist is larger then total goal then cap it 
            if abs(brake_dist) > goal:
                brake_dist = 0.5 * goal

            # check if ready to stop
            if error <= brake_dist:
                # if close begin deceleration
                v = max(0.0, v - a_max * dt)
            else:
                # if far then keep accelerating
                v = min(vmax, v + a_max * dt)

            # ensure we don't overshoot
            v = min(v, error / dt)

            # don't go over max or below min
            if v > 0.0:
                v = max(min(v, vmax), min(v_min, error / dt))

            w_cmd = 0.0

            if heading_target is not None:
                h_err = angle_wrap(heading_target - self.yaw)

                # simple proportional control to maintain target heading
                w_cmd = k_heading * h_err
                if w_cmd > w_max:
                    w_cmd = w_max
                elif w_cmd < -w_max:
                    w_cmd = -w_max

            # if int(time.time()*10) % 10 == 0:  # 10 Hz
            #     self.dbg(f"Drive loop -> d={d:.3f}, err={error:.3f}, v={v:.3f}, w={w_cmd:.3f}")

            # move forwards
            self.send_speed(direction * v, w_cmd)

        # stop robot
        self.send_speed(0.0, 0.0)

    def smooth_rotate(self, angle: float, angular_speed: float, heading_tol=None):
        """
        Smoothly rotates the robot in place using a trapezoidal angular velocity profile.
        Uses loop-time-derived dt (no rclpy.Rate).

        :param angle          [float] [rad]   The signed angle to rotate (CCW +).
        :param angular_speed  [float] [rad/s] The max angular speed (|wmax| used).
        :param heading_tol    [float] [deg] the loop braek angle
        """
        # stop within 1 deg
        if heading_tol is None:
            yaw_tol = math.radians(5) #[rad]
        else:
            yaw_tol = math.radians(heading_tol)
        # 10 deg/s
        alpha_max = math.radians(15.0) #[rad/s^2] 
        # 5 deg/s
        w_min = math.radians(10) # [rad/s]
        dt_min = 1e-4 # [s]

        # calculate yaw information
        start_yaw = self.yaw
        target_yaw = angle_wrap(start_yaw + angle) #absolute
        total_span  = abs(angle_wrap(target_yaw - start_yaw)) #relative

        # calculate angular speed information
        w = 0.0
        wmax = abs(angular_speed) if angular_speed != 0.0 else math.radians(45.0)
        t_prev = time.monotonic()

        while rclpy.ok():
            
            if self.estop_enabled and self.estop_triggered_in_goal and not self.estop_active:
                self.dbg("smooth_drive: E-STOP was triggered and is now cleared; aborting drive.")
                break

            # calculate error
            err = angle_wrap(target_yaw - self.yaw)
            angerr = abs(err)

            # if angle is within tolerance then break
            if angerr <= yaw_tol:
                break
            
            msg = Float32()
            msg.data = float(self.yaw)
            self.turn_error_pub.publish(msg)

            # update time
            t_now = time.monotonic()
            dt = t_now - t_prev
            t_prev = t_now

            # calculate angle needed to start breaking
            brake_ang = (w * w) / (2.0 * alpha_max) + w * dt 

            # if breaking angle is larger then total goal then cap it 
            if brake_ang > total_span:
                brake_ang = 0.5 * total_span

            # check if ready to stop
            if angerr <= brake_ang:
                # if close begin deceleration
                w = max(0.0, w - alpha_max * dt)
            else:
                # if far then keep accelerating
                w = min(wmax, w + alpha_max * dt)

            # ensure we don't overshoot
            w = min(w, angerr / dt)

            # don't go over max or below min
            if w > 0.0:
                w = max(min(w, wmax), min(w_min, angerr / dt))

            # apply correct direction
            w_signed = math.copysign(w, err)
            self.send_speed(0.0, w_signed)

            # if int(time.time()*10) % 10 == 0:
            #     self.dbg(f"Rotate loop -> err={math.degrees(angerr):.2f} deg, w={math.degrees(w):.2f} deg/s")

        # stop robot
        self.send_speed(0.0, 0.0)

    #def _drive_cmd_cb(self, msg: Float32):
        #self.get_logger().info(f'/drive_distance: {msg.data:.3f} m')
        #self.drive(msg.data, float(self.get_parameter('default_linear_speed').value))

    #def _rotate_cmd_cb(self, msg: Float32):
        #self.get_logger().info(f'/rotate_angle: {msg.data:.3f} rad')
        #self.rotate(msg.data, float(self.get_parameter('default_angular_speed').value))

    def scan_callback(self, msg: LaserScan):
        """
        Monitor the laser scan and:
          - Activate E-STOP if obstacle is too close in front.
          - While E-STOP is active and backing up, clear it when the robot
            is roughly centered between front and back obstacles.
        """
        self.have_scan = True

        if not self.estop_enabled:
            return

        angle_window_front = math.radians(self.estop_angle_window_deg)
        angle_window_back  = angle_window_front
        back_center_angle  = math.pi  # 180 deg behind

        min_front = float('inf')
        min_back  = float('inf')

        angle = msg.angle_min
        for r in msg.ranges:
            # Skip invalid / out-of-range readings
            if (not math.isfinite(r) or
                r <= 0.0 or
                r < msg.range_min or
                r > msg.range_max):
                angle += msg.angle_increment
                continue

            # FRONT window (around 0 rad)
            if abs(angle) <= angle_window_front:
                if r < min_front:
                    min_front = r

            # BACK window (around pi rad)
            back_offset = angle_wrap(angle - back_center_angle)
            if abs(back_offset) <= angle_window_back:
                if r < min_back:
                    min_back = r

            angle += msg.angle_increment

        self.min_front_distance = min_front
        if not self.estop_active:
            new_estop = (min_front < self.estop_distance)

        else:
            both_valid = math.isfinite(min_front) and math.isfinite(min_back)

            if both_valid:
                centered = (
                    (min_front > self.estop_distance) and
                    (min_back  > self.estop_distance) and
                    (abs(min_front - min_back) <= self.estop_center_tolerance)
                )
                new_estop = not centered
            else:
                new_estop = (min_front < self.estop_distance)


        if new_estop and not self.estop_active:
            self.get_logger().warn(
                f"E-STOP ACTIVATED: front={min_front:.3f} m, back={min_back:.3f} m"
            )
            if self.follow_path_goal_active:
                self.estop_triggered_in_goal = True

        elif not new_estop and self.estop_active:
            self.get_logger().info(
                f"E-STOP CLEARED: front={min_front:.3f} m, back={min_back:.3f} m"
            )

        self.estop_active = new_estop

        if self.estop_active:
            self.send_speed(0.0, 0.0)


def main(args=None):
    print('starting')
    rclpy.init(args=args)
    node = Controller()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.get_logger().info('shutting down controller.')
    node.send_speed(0.0, 0.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
