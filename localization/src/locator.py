#!/usr/bin/env python3
import math
import random
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, Quaternion, Point, TransformStamped
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster


class Locator(Node):

    def __init__(self):
        super().__init__("Locator")

        # params
        self.num_particles = self.declare_parameter(
            "num_particles", 300
        ).value
        self.motion_noise_xy = self.declare_parameter(
            "motion_noise_xy", 0.01  # meters (std dev)
        ).value
        self.motion_noise_theta = self.declare_parameter(
            "motion_noise_theta", 0.08  # radians (std dev)
        ).value
        self.beam_step = self.declare_parameter(
            "beam_step", 10  # use every Nth scan beam
        ).value
        self.z_hit_std = self.declare_parameter(
            "z_hit_std", 0.05  # meters
        ).value
        self.occ_threshold = self.declare_parameter(
            "occ_threshold", 50
        ).value
        self.resample_every_n_scans = self.declare_parameter(
            "resample_every_n_scans", 1  # resample every N scans
        ).value
        self.random_particle_ratio = self.declare_parameter(
            "random_particle_ratio", 0.05  # fraction replaced by random particles
        ).value

        #weights must sum to 1
        self.z_hit_weight = self.declare_parameter(
            "z_hit_weight", 0.90
        ).value
        self.z_short_weight = self.declare_parameter(
            "z_short_weight", 0.025
        ).value
        self.z_max_weight = self.declare_parameter(
            "z_max_weight", 0.025
        ).value
        self.z_rand_weight = self.declare_parameter(
            "z_rand_weight", 0.05
        ).value

        # Exponential decay for "short readings"
        self.lambda_short = self.declare_parameter(
            "lambda_short", 0.1
        ).value

        self.covariance_clamped = False


        self.map = None
        self.map_arr = None # 2D array of occupancy values

        self.particles = None # (N, 3): [x, y, theta]
        self.weights = None

        self.last_odom = None
        self.scan_counter = 0

        self.map_frame = "map"
        self.odom_frame = "odom"
        self.base_frame = "base_link"

        # TF broadcaster (for map -> odom)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.transform = None

        qos_map = QoSProfile(depth=10)
        qos_map.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        qos_scan = QoSProfile(depth=10)
        qos_scan.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.handle_map, qos_profile=qos_map)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.handle_odom, 50)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.handle_scan, qos_profile=qos_scan)

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/mcl_pose", 10)
        self.particle_pub = self.create_publisher(PoseArray, "/mcl_particles", 10)

    #map helpers
    @staticmethod
    def occgrid_to_array(grid: OccupancyGrid) -> np.ndarray:
        dtype = np.int16
        width = grid.info.width
        height = grid.info.height

        data = np.array(grid.data, dtype=dtype)
        if data.size != width * height:
            raise ValueError(
                f"OccupancyGrid data size {data.size} does not match "
                f"width*height = {width * height}"
            )

        return data.reshape((height, width))

    @staticmethod
    def grid_to_world(mapdata: OccupancyGrid, p: tuple[int, int]) -> Point:
        origin = mapdata.info.origin
        resolution = mapdata.info.resolution
        gx, gy = p
        wx = origin.position.x + (gx + 0.5) * resolution
        wy = origin.position.y + (gy + 0.5) * resolution
        pt = Point()
        pt.x = wx
        pt.y = wy
        pt.z = 0.0
        return pt

    @staticmethod
    def world_to_grid(mapdata: OccupancyGrid, wp: Point) -> tuple[int, int]:
        origin = mapdata.info.origin
        resolution = mapdata.info.resolution
        wx = wp.x
        wy = wp.y
        gx = int((wx - origin.position.x) / resolution)
        gy = int((wy - origin.position.y) / resolution)
        return (gx, gy)

    @staticmethod
    def angle_wrap(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def angle_wrap_array(angles: np.ndarray) -> np.ndarray:
        return (angles + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def quat_to_yaw(q: Quaternion) -> float:
        r = Rotation.from_quat([q.x, q.y, q.z, q.w])
        yaw, _, _ = r.as_euler("ZYX")
        return float(yaw)

    @staticmethod
    def yaw_to_quat(yaw: float) -> Quaternion:
        r = Rotation.from_euler("z", yaw)
        x, y, z, w = r.as_quat()
        q = Quaternion()
        q.x = float(x)
        q.y = float(y)
        q.z = float(z)
        q.w = float(w)
        return q

    #callbacks
    def handle_map(self, msg: OccupancyGrid):
        self.map = msg
        self.map_arr = self.occgrid_to_array(msg)

        self.get_logger().info(
            f"Received map: size=({msg.info.width}, {msg.info.height}), "
            f"res={msg.info.resolution}"
        )

        if self.particles is None:
            self.initialize_particles()

    def handle_odom(self, msg: Odometry):
        if self.map is None or self.map_arr is None:
            self.last_odom = msg
            return

        if self.particles is None:
            self.initialize_particles()

        if self.last_odom is None:
            self.last_odom = msg
            return

        dx, dy, dtheta = self.compute_odom_delta(self.last_odom, msg)
        self.last_odom = msg
        self.motion_update(dx, dy, dtheta)

    def handle_scan(self, scan: LaserScan):
        if (self.map is None
            or self.map_arr is None
            or self.particles is None
            or self.weights is None):
            return

        self.sensor_update(scan)

        self.scan_counter += 1
        if self.scan_counter % self.resample_every_n_scans == 0:
            self.resample_particles()

        self.publish_pose(scan.header.stamp)
        self.publish_particles(scan.header.stamp)

    def initialize_particles(self):
        if self.map is None or self.map_arr is None:
            return

        data = self.map_arr
        free_cells = np.where((data >= 0) & (data <= self.occ_threshold))
        num_free = free_cells[0].size

        if num_free == 0:
            self.get_logger().warn(
                "MonteCarloLocalization: map has no free cells; cannot initialize particles."
            )
            return

        self.get_logger().info(
            f"Initializing {self.num_particles} particles over {num_free} free cells."
        )

        idx = np.random.choice(num_free, self.num_particles, replace=True)
        ys = free_cells[0][idx]
        xs = free_cells[1][idx]

        origin = self.map.info.origin
        res = self.map.info.resolution

        xs_world = xs * res + origin.position.x + res * 0.5
        ys_world = ys * res + origin.position.y + res * 0.5
        thetas = np.random.uniform(-math.pi, math.pi, size=self.num_particles)

        self.particles = np.vstack((xs_world, ys_world, thetas)).T
        self.weights = np.ones(self.num_particles, dtype=float) / float(self.num_particles)

    #prediction
    def compute_odom_delta(self, old: Odometry, new: Odometry) -> tuple[float, float, float]:
        old_pos = old.pose.pose.position
        new_pos = new.pose.pose.position

        old_q = old.pose.pose.orientation
        new_q = new.pose.pose.orientation

        old_yaw = self.quat_to_yaw(old_q)
        new_yaw = self.quat_to_yaw(new_q)

        dx = new_pos.x - old_pos.x
        dy = new_pos.y - old_pos.y
        dtheta = self.angle_wrap(new_yaw - old_yaw)

        return dx, dy, dtheta

    def motion_update(self, dx: float, dy: float, dtheta: float):
        if self.particles is None:
            return

        N = self.num_particles

        noisy_dx = dx + np.random.normal(0.0, self.motion_noise_xy, size=N)
        noisy_dy = dy + np.random.normal(0.0, self.motion_noise_xy, size=N)
        noisy_dtheta = dtheta + np.random.normal(0.0, self.motion_noise_theta, size=N)

        self.particles[:, 0] += noisy_dx
        self.particles[:, 1] += noisy_dy
        self.particles[:, 2] = self.angle_wrap_array(self.particles[:, 2] + noisy_dtheta)

    #observation
    def beam_model_prob(self, z_meas: float, z_exp: float, z_max: float) -> float:
        """
        Full beam model p(z | x, m) as a mixture of:
          - p_hit: Gaussian around expected range z_exp
          - p_short: exponential for unexpected closer obstacles
          - p_max: point-mass at max range
          - p_rand: uniform random measurement
        """

        # Invalid or out-of-range measurements get tiny probability
        if math.isnan(z_meas) or math.isinf(z_meas) or z_meas < 0.0:
            return 1e-12

        # Clamp to max range
        if z_meas > z_max:
            z_meas = z_max

        p_hit = 0.0
        p_short = 0.0
        p_max = 0.0
        p_rand = 0.0

        #p_hit -> Gaussian around z_exp
        if 0.0 <= z_meas <= z_max:
            sigma = self.z_hit_std
            if sigma > 0.0:
                norm = 1.0 / (sigma * math.sqrt(2.0 * math.pi))
                p_hit = norm * math.exp(-0.5 * ((z_meas - z_exp) / sigma) ** 2)

        # p_short -> unexpected obstacles closer than expected range only if measurement is closer than expected
        if 0.0 <= z_meas <= z_exp:
            lam = self.lambda_short
            if lam > 0.0:
                # Simple exponential; normalization over [0, z_exp] again omitted
                p_short = lam * math.exp(-lam * z_meas)

        # p_max -> detection failure, returns max range
        max_window = 0.05 * z_max  # 5% band near max range
        if abs(z_meas - z_max) < max_window:
            p_max = 1.0

        # p_rand -> random measurement, uniform over [0, z_max]
        if 0.0 <= z_meas <= z_max and z_max > 0.0:
            p_rand = 1.0 / z_max

        p = (
            self.z_hit_weight * p_hit
            + self.z_short_weight * p_short
            + self.z_max_weight * p_max
            + self.z_rand_weight * p_rand
        )

        #clamp to avoid zeros getting multiplied in
        return max(p, 1e-12)

    def sensor_update(self, scan: LaserScan):
        if self.particles is None or self.map_arr is None:
            return

        max_range = scan.range_max
        origin = self.map.info.origin
        res = self.map.info.resolution

        angle_min = scan.angle_min
        angle_inc = scan.angle_increment

        # Use every Nth beam to reduce computation
        beam_indices = range(0, len(scan.ranges), self.beam_step)

        N = self.num_particles
        new_weights = np.zeros(N, dtype=float)

        for i in range(N):
            x, y, theta = self.particles[i]
            w_i = 1.0

            for bi in beam_indices:
                z_meas = scan.ranges[bi]

                # skip invalid measurements
                if math.isinf(z_meas) or math.isnan(z_meas):
                    continue

                beam_angle = theta + angle_min + bi * angle_inc

                z_exp = self.raycast(
                    x, y, beam_angle, max_range,
                    self.map_arr, origin, res, self.occ_threshold
                )

                # full beam model for this beam
                p_beam = self.beam_model_prob(z_meas, z_exp, max_range)

                # multiply into the particle's likelihood
                w_i *= p_beam

            new_weights[i] = w_i

        w_sum = float(np.sum(new_weights))

        if w_sum <= 0.0 or not np.isfinite(w_sum):
            self.get_logger().warn(
                "MonteCarloLocalization: sensor_update produced invalid weights; "
                "resetting to uniform."
            )
            self.weights = np.ones(N, dtype=float) / float(N)
        else:
            # normalize so weights sum to 1
            self.weights = new_weights / w_sum

    @staticmethod
    def raycast(x: float, y: float, theta: float, max_range: float, map_arr: np.ndarray, origin, res: float, occ_threshold: int,) -> float:
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        step = res * 0.5
        dist = 0.0

        height, width = map_arr.shape

        while dist < max_range:
            wx = x + dist * cos_t
            wy = y + dist * sin_t

            gx = int((wx - origin.position.x) / res)
            gy = int((wy - origin.position.y) / res)

            #if at edge of map
            if gx < 0 or gy < 0 or gx >= width or gy >= height:
                return dist

            val = int(map_arr[gy, gx])
            
            if val == -1 or val > occ_threshold:
                return dist

            dist += step

        return max_range

    def resample_particles(self):
        if self.particles is None or self.weights is None:
            return

        N = self.num_particles

        positions = (np.arange(N) + random.random()) / N
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.0
        indexes = np.searchsorted(cumulative_sum, positions)

        new_particles = self.particles[indexes].copy()

        if not self.covariance_clamped:
            num_random = int(self.random_particle_ratio * N)
        else:
            num_random = 0

        if (self.map is not None and self.map_arr is not None):
            map = self.map_arr
            free_cells = np.where((map >= 0) & (map <= self.occ_threshold))
            num_free = free_cells[0].size

            if num_free > 0:
                ridx = np.random.choice(num_free, num_random, replace=True)
                ys = free_cells[0][ridx]
                xs = free_cells[1][ridx]

                origin = self.map.info.origin
                res = self.map.info.resolution

                xs_world = xs * res + origin.position.x + res * 0.5
                ys_world = ys * res + origin.position.y + res * 0.5
                thetas = np.random.uniform(-math.pi, math.pi, size=num_random)

                new_particles[:num_random, 0] = xs_world
                new_particles[:num_random, 1] = ys_world
                new_particles[:num_random, 2] = thetas

        self.particles = new_particles
        self.weights = np.ones(N, dtype=float) / float(N)

    #publisher functions
    def publish_pose(self, stamp):
        """Publish the weighted mean pose of the particle cloud with
        covariance estimated from particle spread."""
        if self.particles is None or self.weights is None:
            return

        # Normalize weights just in case numeric error crept in
        w = self.weights.astype(float)
        w_sum = np.sum(w)
        if w_sum <= 0.0 or not np.isfinite(w_sum):
            w = np.ones_like(w) / float(len(w))
        else:
            w /= w_sum

        xs = self.particles[:, 0]
        ys = self.particles[:, 1]
        thetas = self.particles[:, 2]

        # Weighted mean
        x_mean = float(np.sum(w * xs))
        y_mean = float(np.sum(w * ys))

        # Mean angle via circular average
        cos_mean = np.sum(w * np.cos(thetas))
        sin_mean = np.sum(w * np.sin(thetas))
        theta_mean = math.atan2(sin_mean, cos_mean)

        q = self.yaw_to_quat(theta_mean)

        # Covariance from particle spread
        dx = xs - x_mean
        dy = ys - y_mean
        dtheta = self.angle_wrap_array(thetas - theta_mean)

        var_x = float(np.sum(w * dx * dx))
        var_y = float(np.sum(w * dy * dy))
        var_th = float(np.sum(w * dtheta * dtheta))

        cov_xy  = float(np.sum(w * dx * dy))
        cov_xth = float(np.sum(w * dx * dtheta))
        cov_yth = float(np.sum(w * dy * dtheta))

        min_pos_var = .0055
        min_ang_var = .007

        var_x = max(var_x, min_pos_var)
        var_y = max(var_y, min_pos_var)
        var_th = max(var_th, min_ang_var)

        if var_x == min_pos_var or var_y == min_pos_var or var_th == min_ang_var:
            self.covariance_clamped = True

        # 6x6 covariance matrix flattened (x, y, z, roll, pitch, yaw)
        cov = [0.0] * 36

        # Variances
        cov[0]  = var_x # Var(x)
        cov[7]  = var_y # Var(y)
        cov[35] = var_th # Var(yaw)

        # Cross terms (symmetric)
        cov[1]  = cov_xy # Cov(x, y)
        cov[6]  = cov_xy

        cov[5]  = cov_xth # Cov(x, yaw)
        cov[30] = cov_xth

        cov[11] = cov_yth # Cov(y, yaw)
        cov[31] = cov_yth

        stamp = self.get_clock().now().to_msg()

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame

        msg.pose.pose.position.x = x_mean
        msg.pose.pose.position.y = y_mean
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = q
        msg.pose.covariance = cov

        self.pose_pub.publish(msg)

        # Also publish map -> odom TF using this estimate
        self.publish_tf(stamp, x_mean, y_mean, theta_mean)


    def publish_particles(self, stamp):
        if self.particles is None:
            return

        pa = PoseArray()
        pa.header.stamp = stamp
        pa.header.frame_id = self.map_frame

        for x, y, theta in self.particles:
            q = self.yaw_to_quat(theta)
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = 0.0
            p.orientation = q
            pa.poses.append(p)

        self.particle_pub.publish(pa)

    def publish_tf(self, stamp, x_map: float, y_map: float, theta_map: float):
        """
        Publish the transform map -> odom.
        """
        if self.last_odom is None:
            return

        # Pose of base in odom frame
        odom_pos = self.last_odom.pose.pose.position
        odom_q = self.last_odom.pose.pose.orientation
        theta_odom = self.quat_to_yaw(odom_q)
        x_odom = odom_pos.x
        y_odom = odom_pos.y

        #R_map_odom = R_map_base * R_odom_base^T => yaw_map_odom = theta_map - theta_odom
        theta_map_odom = self.angle_wrap(theta_map - theta_odom)
        cos_mo = math.cos(theta_map_odom)
        sin_mo = math.sin(theta_map_odom)

        # p_map_odom = p_map_base - R_map_odom * p_odom_base
        tx = x_map - (cos_mo * x_odom - sin_mo * y_odom)
        ty = y_map - (sin_mo * x_odom + cos_mo * y_odom)

        q_mo = self.yaw_to_quat(theta_map_odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame

        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = 0.0
        t.transform.rotation = q_mo

        # self.tf_broadcaster.sendTransform(t)
        
        if not self.covariance_clamped:
            self.transform = t
            self.tf_broadcaster.sendTransform(t)
        else:
            self.transform.header.stamp = stamp
            self.tf_broadcaster.sendTransform(self.transform)
        


def main(args=None):
    rclpy.init(args=args)
    node = Locator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
