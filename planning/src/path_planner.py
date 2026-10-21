#!/usr/bin/env python3
from __future__ import annotations
from typing import List, Tuple

import time
import math
import rclpy
from geometry_msgs.msg import Point, PoseStamped, PointStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from scipy.spatial.transform import Rotation 
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, Duration
from nav_msgs.msg import OccupancyGrid
from priority_queue import PriorityQueue
import tf2_geometry_msgs
from rclpy.time import Time
from rclpy.action import ActionClient
from control.action import FollowPath
from std_msgs.msg import Bool
from collections import deque



class PathPlanner(Node):
    def __init__(self):

        # TODO: Initialize the node and call it "path_planner"

        super().__init__('path_planner')

        # Create Quality of Service (QoS) policy. Include a profile, depth, and durablilty policy. 
        # Hint: See the ROS 2 Docs if you get stuck!

        # Version 1: Transient Local

        qos_1 = QoSProfile(depth=10)
        qos_1.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Version 2: Volatile

        qos_2 = QoSProfile(depth=10)
        qos_2.durability = QoSDurabilityPolicy.VOLATILE


        # Global variables
        self.pose_frame = 'base_link'
        self.data_for_map = None
        self.x: float = 0.0
        self.y: float = 0.0
        self.yaw: float = 0.0
        self.padding = 0

        #Subscribe to the map topic
        #Subscribe to the RVIZ nav goal pose, and have it call plan_path
        # Subscribe to clicked_point

        self.create_subscription(OccupancyGrid, '/map', self.handle_map, qos_profile=qos_1)
        self.goal_sub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.plan_path, qos_profile=qos_2)
        self.risk_sub = self.create_subscription(OccupancyGrid, '/explorer/risk_map', self.handle_risk_map, qos_profile=qos_1)
        self.home_goal_sub = self.create_subscription(PoseStamped, '/explorer/go_home', self.plan_home_path, qos_profile=qos_2)

        self.trim_paths_enabled = True

        #self.clicked_point_sub = self.create_subscription(PointStamped,'/clicked_point', self.handle_clicked_point, qos_profile=qos_2)

        # Create a new publisher for '/path_planner/c_obstacle' that publishes a message of type GridCells
        # Create publisher to send a Path message to nav_path. (If you did Lab2 ROS2 action extra credit, make an action client instead)
        # Create other publishers with varying message types (GridCells, Path, etc) for visualizing data in Rviz


        self.C_space_pub = self.create_publisher(GridCells, '/path_planner/c_obstacle', qos_profile=qos_1)
        self.nearest_cell_pub = self.create_publisher(GridCells, '/path_planner/nearest_cell', qos_profile=qos_2)
        self.path_pub = self.create_publisher(Path, '/nav_path', qos_profile=qos_1)
        
        self.risk_map = None
        self.risk_weight = 1.2
        self.distance_weight = 1.2
        
        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')

        # track the currently running goal & whether it's active
        self.current_goal_handle = None
        self.follow_path_active = False

        # If a new path is requested while an action is running,
        # we stash it here until the cancel completes.
        self.pending_path_goal = None

        self.frontier_grid_pub = self.create_publisher(GridCells, '/path_planner/frontier', qos_profile=qos_2)
        self.visited_grid_pub = self.create_publisher(GridCells, '/path_planner/visited', qos_profile=qos_2)
        self.path_grid_pub = self.create_publisher(GridCells, '/path_planner/path', qos_profile=qos_2)
        self.goal_complete_pub = self.create_publisher(Bool, '/path_planner/goal_complete', qos_profile=qos_1)

        #stall / progress tracking
        self.last_feedback_index = None # last waypoint index reported
        self.last_progress_time = None # last time we advanced an index
        # self.progress_timeout = 15.0 # seconds of no progress before we declare a stall

        # Timer to periodically check if FollowPath is stalled
        # self.stall_timer = self.create_timer(0.5, self.check_follow_path_stall)
        #self.path_pub = self.create_publisher(Path, '/path_planner/nav_path', qos_1)
        #self.nearest_cell_pub = self.create_publisher(GridCells, '/path_planner/nearest_cell', qos_profile=qos_1)

        self.estop_replan_enabled = self.declare_parameter(
            'estop_replan_enabled', True
        ).value
        self.estop_max_replans = self.declare_parameter(
            'estop_max_replans', 10  # how many times to retry the same goal after E-STOP
        ).value
        self.estop_replan_attempts = 0
        self.suppress_estop_reset = False

        self.last_goal = None
        
        self.early_completion_enabled = self.declare_parameter(
            'early_completion_enabled', False
        ).value
        self.completion_threshold = self.declare_parameter(
            'completion_threshold', 0.75
        ).value  # 75% by default

        # self.current_path_length = 0
        # self.early_completion_sent = False
         
        self.get_logger().info("path planner node initalized!")

        # Setup TF Buffer and listener

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Setup Rate function for sleep
        self.rate = self.create_rate(0.005)

    @staticmethod
    def grid_to_index(mapdata: OccupancyGrid, p: tuple[int, int]) -> int:
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param p [(int, int)] The cell coordinate.
        :return  [int] The index.
        """
        
        width = mapdata.info.width

        x = p[0]
        y = p[1]

        index = (width * y) + x

        return int(index) 


    @staticmethod
    def euclidean_distance(p1: tuple[float, float], p2: tuple[float, float]) -> float:
        """
        Calculates the Euclidean distance between two points.
        :param p1 [(float, float)] first point.
        :param p2 [(float, float)] second point.
        :return   [float]          distance.
        """

        c1_x = p1[0]
        c1_y = p1[1]

        c2_x = p2[0]
        c2_y = p2[1]

        # Euclidean distance calculation
        distance = math.sqrt((c2_x - c1_x)**2 + (c2_y - c1_y)**2)

        return float(distance)
        

    @staticmethod
    def grid_to_world(mapdata: OccupancyGrid, p: tuple[int, int]) -> Point:
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param p [(int, int)] The cell coordinate.
        :return        [Point]         The position in the world.
        """
        
        origin = mapdata.info.origin
        resolution = mapdata.info.resolution

        Coord_x = p[0]
        Coord_y = p[1]

        world_coord_x = (origin.position.x) + (Coord_x + 0.5) * resolution
        world_coord_y = (origin.position.y) + (Coord_y + 0.5) * resolution

        point = Point()
        point.x = world_coord_x
        point.y = world_coord_y
        point.z = 0.0

        return point

        
    @staticmethod
    def world_to_grid(mapdata: OccupancyGrid, wp: Point) -> tuple[int, int]:
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """

        origin = mapdata.info.origin
        resolution = mapdata.info.resolution

        world_coord_x = wp.x
        world_coord_y = wp.y

        Coord_x = (world_coord_x - origin.position.x)/resolution
        Coord_y = (world_coord_y - origin.position.y)/resolution 

        tup = (int(Coord_x), int(Coord_y))

        return tup


        
    @staticmethod
    def path_to_poses(mapdata: OccupancyGrid, path: list[tuple[int, int]]) -> list[PoseStamped]:
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        
        coord_list = []
        path_length = len(path)

        if path_length == 0:
            return coord_list
        
        world_points = []
        for i in range(path_length):
            x = path[i][0]
            y = path[i][1]
            cell_to_world = PathPlanner.grid_to_world(mapdata, (x, y))
            world_points.append(cell_to_world)

        for i in range(path_length):
            pose = PoseStamped()
            pose.header.frame_id = 'map'

            pose.pose.position = world_points[i]

            if path_length == 1:
                yaw = 0.0
            elif i < path_length - 1:
                dx = world_points[i + 1].x - world_points[i].x
                dy = world_points[i + 1].y - world_points[i].y
                yaw = math.atan2(dy, dx)
            else:
                dx = world_points[i].x - world_points[i - 1].x
                dy = world_points[i].y - world_points[i - 1].y
                yaw = math.atan2(dy, dx)

            #yaw to quaternion (rotation about Z)
            q = Rotation.from_euler('ZYX', [yaw, 0.0, 0.0]).as_quat()
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            coord_list.append(pose)

        return coord_list


    @staticmethod
    def is_cell_walkable(mapdata:OccupancyGrid, p: tuple[int, int]) -> bool:
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [bool]          True if the cell is walkable, False otherwise
        """
        
        px = p[0]
        py = p[1]

        width = mapdata.info.width
        height = mapdata.info.height

        if not (0 <= px < width and 0 <= py < height):
            return False

        index = PathPlanner.grid_to_index(mapdata,p)
        probability = mapdata.data[index]

        if probability > 0 or probability == -1:
            return False
        else:
            return True
         

    @staticmethod
    def neighbors_of_4(mapdata: OccupancyGrid, p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """

        x = p[0]
        y = p[1]

        north = (x,y + 1)
        east  = (x + 1,y)
        south = (x,y - 1)
        west  = (x - 1,y)

        init_nav_list = [north, east, south, west]
        final_nav_list = []

        for i in range(len(init_nav_list)):
            nav_possibility = init_nav_list[i]
            walkable = PathPlanner.is_cell_walkable(mapdata,nav_possibility)
            if walkable:
                final_nav_list.append(nav_possibility)

        return final_nav_list


    @staticmethod
    def neighbors_of_8(mapdata: OccupancyGrid, p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        
        x = p[0]
        y = p[1]

        north = (x,y + 1)
        east  = (x + 1,y)
        south = (x,y - 1)
        west  = (x - 1,y)

        north_east = (x + 1,y + 1)
        north_west = (x - 1,y + 1)
        south_east = (x + 1,y - 1)
        south_west = (x - 1,y - 1)

        init_nav_list = [north, east, south, west, north_east, north_west, south_east, south_west]
        final_nav_list = []

        for i in range(len(init_nav_list)):
            nav_possibility = init_nav_list[i]
            walkable = PathPlanner.is_cell_walkable(mapdata,nav_possibility)
            if walkable:
                final_nav_list.append(nav_possibility)

        return final_nav_list


    def handle_map(self, map:OccupancyGrid):
        """
        Store the incoming map data in a global variable. 
        This function should be called and the map data updated whenever it is available.
        """

        self.data_for_map = map
        # cspace_map = self.calc_cspace(self.data_for_map, padding=self.padding)
        #self.get_logger().info('HERE')
        #self.get_logger().info(self.data_for_map)
        # self.risk_map = self.calc_risk_map(cspace_map)
        # self.risk_pub.publish(self.risk_map)

    def handle_risk_map(self, map:OccupancyGrid):
        """
        Store the incoming map data in a global variable. 
        This function should be called and the map data updated whenever it is available.
        """

        self.risk_map = map
        


    def calc_cspace(self, mapdata: OccupancyGrid, padding: int) -> OccupancyGrid:
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """

        self.get_logger().info("Calculating C-Space")
        # Go through each cell in the occupancy grid. Inflate the obstacles where necessary

        width = mapdata.info.width
        height = mapdata.info.height
        resolution = mapdata.info.resolution

        Ob_cells = []
        new_map_data = list(mapdata.data)


        for y in range(height):
            for x in range(width):

                index = PathPlanner.grid_to_index(mapdata,(x,y))
                Ob_Prob = mapdata.data[index]

                if Ob_Prob > 50:
                    for Obs_in_y_dist in range(-padding,padding+1):
                        for Obs_in_x_dist in range(-padding,padding+1):
                            x_pos = x + Obs_in_x_dist
                            y_pos = y + Obs_in_y_dist

                            if self.is_cell_walkable(mapdata,(x_pos,y_pos)):

                                new_index = PathPlanner.grid_to_index(mapdata, (x_pos,y_pos))
                                new_map_data[new_index] = 100
                                point = PathPlanner.grid_to_world(mapdata,(x_pos,y_pos))
                                Ob_cells.append(point)
                                    


        # Create a GridCells message and publish it

        c_grid = GridCells()
        c_grid.header.frame_id = 'map'
        c_grid.cell_width = resolution
        c_grid.cell_height = resolution
        c_grid.cells = Ob_cells

        # Return the C-space
        
        self.C_space_pub.publish(c_grid)

        output = OccupancyGrid()
        output.header = mapdata.header
        output.data = new_map_data
        output.info = mapdata.info

        return output
    
    def handle_clicked_point(self, msg: PointStamped):
        """
        Callback for when a point is clicked in RViz using the 'Publish Point' tool.
        Stores the clicked point and publishes it as the nearest cell (or for path planning use).
        """

        # Store the point
        self.clicked_point = msg.point

        # Conversions

        grid_coord = PathPlanner.world_to_grid(self.data_for_map,self.clicked_point)
        nearest_cell_in_world = PathPlanner.grid_to_world(self.data_for_map, grid_coord)

        grid_msg = GridCells()
        grid_msg.header.frame_id = "map"
        grid_msg.cell_width =  self.data_for_map.info.resolution
        grid_msg.cell_height = self.data_for_map.info.resolution
        grid_msg.cells = [nearest_cell_in_world]

        # Publish the nearest cell visualization
        self.nearest_cell_pub.publish(grid_msg)

        self.get_logger().info("Published nearest cell marker for clicked point.")

    def heuristic(self, current: tuple[int, int], goal: tuple[int, int], prev_dir=None) -> float:
        distance = PathPlanner.euclidean_distance(current, goal)
        #other heuristic things

        # change 0.1 based on the influence we want it to have
        risk_heuristic = self.get_risk_cost(current, self.data_for_map) * 0.1 
        if prev_dir is not None:
            # goal_dir = PathPlanner.step_dir(current, goal)

            # dtheta = PathPlanner.angle_between_dirs(prev_dir, goal_dir)

            cost = distance + risk_heuristic
        else:
            cost = distance + risk_heuristic

        return cost
    
    def get_risk_cost(self, cell: tuple[int, int], mapdata: OccupancyGrid) -> float:
        """
        Calculate risk cost for a cell using the risk map.
        Returns 0 if no risk map is available.
        """
        if self.risk_map is None:
            return 0.0
        
        risk_index = self.grid_to_index(mapdata, cell)
        risk_value = self.risk_map.data[risk_index]

        return risk_value * self.risk_weight

    def a_star(self, mapdata: OccupancyGrid, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        # GROUP
        self.get_logger().info("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))

        # Initialize frontier
        frontier = PriorityQueue()
        frontier.put(start, 0.0)

        # Dictionary that holds the pair (Key = current point) and (Value = Previous position)
        came_from = {}

        # Dictionary that holds the pair (Key = Point) and (Value = Cost)
        cost_so_far = {}
        dir_from = {}

        #Dictionary that holds ...I'm lazy
        came_from[start] =  None
        dir_from[start] = None

        # Dictionary that holds the pair (Key = Previous total cost + current value) and (Value = current cost  )
        cost_so_far[start] =  0.0

        visited_cells = []
        # We might add a try catch later to protect from the fact if there aren't any paths possible
        while not frontier.empty():
            current = frontier.get()

            visited_cells.append(current)

            if current == goal:
                break

            frontier_grid = GridCells()
            frontier_grid.header.frame_id = "map"
            frontier_grid.header.stamp = self.get_clock().now().to_msg()
            frontier_grid.cell_width = mapdata.info.resolution
            frontier_grid.cell_height = mapdata.info.resolution

            frontier_cells_world = []

            for pair in frontier.get_queue():
                node = pair[1]

                world_point = self.grid_to_world(mapdata, node)
                frontier_cells_world.append(world_point)

            frontier_grid.cells = frontier_cells_world
            self.frontier_grid_pub.publish(frontier_grid)

            visited_grid = GridCells()
            visited_grid.header.frame_id = "map"
            visited_grid.header.stamp = self.get_clock().now().to_msg()
            visited_grid.cell_width = mapdata.info.resolution
            visited_grid.cell_height = mapdata.info.resolution

            visited_cells_world = []

            for cell in visited_cells:
                world_point = self.grid_to_world(mapdata, cell)
                visited_cells_world.append(world_point)

            visited_grid.cells = visited_cells_world

            self.visited_grid_pub.publish(visited_grid)

            #self.rate.sleep() # issue with this 
            # time.sleep(0.2)

            for next in self.neighbors_of_8(mapdata, current):
                step_cost = self.distance_weight * self.euclidean_distance(next, current)
                # added
                risk_cost = self.get_risk_cost(next, mapdata)

                # turning cost based on change in direction
                prev_dir = dir_from.get(current)
                this_dir = PathPlanner.step_dir(current, next)

                turn_cost = 0.0
                if prev_dir is not None:
                    delta_theta = PathPlanner.angle_between_dirs(prev_dir, this_dir)
                    cost_per_rad = 2.0
                    turn_cost = 0.0  #cost_per_rad * delta_theta

                new_cost = cost_so_far.get(current) + step_cost + turn_cost + risk_cost

                previous_cost = cost_so_far.get(next)
                
                if previous_cost is None or new_cost < previous_cost:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(next, goal, this_dir)
                    frontier.put(next, priority)
                    came_from[next] = current
                    dir_from[next] = this_dir

        if goal not in came_from:
            self.get_logger().warn("A*: No path found")
            no_path_to_goal = []
            return no_path_to_goal
        
        path_to_goal_rev = []

        while came_from.get(current) is not None: 
            previous_step = came_from.get(current)
            path_to_goal_rev.append(current)
            current = previous_step
        
        path_to_goal = []
        path_to_goal = path_to_goal_rev[::-1]

        final_path_grid = GridCells()
        final_path_grid.header.frame_id = "map"
        final_path_grid.header.stamp = self.get_clock().now().to_msg()
        final_path_grid.cell_width = mapdata.info.resolution
        final_path_grid.cell_height = mapdata.info.resolution

        final_path_world = []

        for cell in path_to_goal:
            world_point = self.grid_to_world(mapdata, cell)
            final_path_world.append(world_point)

        final_path_grid.cells = final_path_world

        self.path_grid_pub.publish(final_path_grid)

        return path_to_goal
        
    #@staticmethod
    def optimize_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """   
        path_length = len(path)
        if path_length <= 2:
            return path

        optimized = []

        optimized.append(path[0])

        anchor_x, anchor_y = path[0]

        prev_dx = path[1][0] - path[0][0]
        prev_dy = path[1][1] - path[0][1]

        count = 0

        for i in range(2, path_length):
            curr_x, curr_y = path[i]
            prev_x, prev_y = path[i - 1]

            vx = curr_x - anchor_x
            vy = curr_y - anchor_y

            cross_product = prev_dx * vy - prev_dy * vx
            count = count + 1

            if cross_product != 0:# or count > 3: #direction change
                optimized.append([prev_x, prev_y])

                anchor_x, anchor_y = prev_x, prev_y

                prev_dx = curr_x - prev_x
                prev_dy = curr_y - prev_y

                count = 0
            else:
                pass

        optimized.append(path[-1])
        # self.get_logger().info(optimized)

        return optimized

    def plan_path(self, msg: PoseStamped): 
        # GROUP
        """
        Plans a path between the current pose and the goal message locations.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        if not self.suppress_estop_reset:
            self.estop_replan_attempts = 0
        else:
            self.suppress_estop_reset = False
    
        # Store the latest goal (used for periodic replanning)
        self.last_goal = msg

        map = self.data_for_map

        if map is None:
            self.get_logger().warn("Aborting: no map received yet (data_for_map is None).")
            return

        if not (map.info.resolution > 0.0 and map.info.width > 0 and map.info.height > 0):
            self.get_logger().warn("Aborting: invalid map info.")
            return
        
        cspace_map = self.calc_cspace(map, self.padding)

        map_frame = map.header.frame_id
        transform = self.tf_buffer.lookup_transform(map_frame, self.pose_frame, time=Time(seconds=0), timeout=Duration(seconds=1.0))
        
        #Calculate a path using A*
        start_point = Point()
        start_point.x = transform.transform.translation.x
        start_point.y = transform.transform.translation.y
        start_point.z = 0.0

        goal_point = Point()
        # goal_point.x = transform.transform.x
        # goal_point.y = transform.transform.y
        goal_point.x = msg.pose.position.x
        goal_point.y = msg.pose.position.y
        goal_point.z = 0.0

        start_grid = self.world_to_grid(cspace_map, start_point)
        goal_grid = self.world_to_grid(cspace_map, goal_point)

        if not self.is_cell_walkable(cspace_map, start_grid):
            shifted_start = self.find_nearest_walkable_cell(cspace_map, start_grid)
            if shifted_start is None:
                self.get_logger().warn(
                    f"Start cell {start_grid} is not walkable, and no nearby walkable "
                    "cell was found. Aborting path plan."
                )
                msg = Bool()
                msg.data = False
                self.goal_complete_pub.publish(msg)
                return

            self.get_logger().info(
                f"Start cell {start_grid} not walkable; shifting to nearest walkable "
                f"cell {shifted_start} in C-space."
            )
            start_grid = shifted_start

        grid_path = self.a_star(cspace_map, start_grid, goal_grid)
        optamized_grid_path = self.optimize_path(grid_path)

        if len(optamized_grid_path) > 1:
            exec_grid_path = optamized_grid_path[1:]
        else:
            exec_grid_path = optamized_grid_path

        if self.trim_paths_enabled and self.early_completion_enabled and len(exec_grid_path) > 0:
            n = len(exec_grid_path)
            # how many grid waypoints to keep (ceil of fraction, at least 1)
            keep = max(1, int(math.ceil(self.completion_threshold * n)))
            exec_grid_path = exec_grid_path[:keep]
            self.get_logger().info(
                f"Truncating path to first {keep}/{n} waypoints "
                f"({self.completion_threshold * 100:.1f}%)."
            )

        world_poses = self.path_to_poses(cspace_map, exec_grid_path)
        #world_poses = self.path_to_poses(cspace_map, grid_path) # here

        path_msg = Path()
        path_msg.header.frame_id = cspace_map.header.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = world_poses

        #Return or publish your path to be driven by the controller topic 
        self.path_pub.publish(path_msg)
        self.handle_new_path_goal(path_msg)


    def handle_new_path_goal(self, path: Path):
        """
        Called whenever we have a new path to execute.
        If an action is already running, cancel it and queue this path.
        Otherwise, send this path immediately as a FollowPath goal.
        """
        if not path.poses:
            self.get_logger().warn("New path is empty; ignoring.")
            msg = Bool()
            msg.data = False
            self.goal_complete_pub.publish(msg)
            return

        if self.follow_path_active and self.current_goal_handle is not None:
            # if there is an active FollowPath goal cancel it and queue the new path
            self.get_logger().info(
                f"Active FollowPath goal in progress; canceling and queuing new path "
                f"({len(path.poses)} poses)."
            )
            self.pending_path_goal = path
            self.cancel_current_follow_path_goal()
        else:
            # no active goal, just send this one
            self.get_logger().info(
                f"Sending new FollowPath goal with {len(path.poses)} poses."
            )
            self.send_follow_path_goal(path)

    def plan_home_path(self, msg: PoseStamped):
        """
        Explorer go_home -> disable trimming for this path only.
        """
        self.trim_paths_enabled = False  # disable trimming
        self.get_logger().info("Go-home goal received → Path trimming disabled.")
        self.plan_path(msg)

    def cancel_current_follow_path_goal(self):
        """Request cancellation of the currently active FollowPath goal."""
        if self.current_goal_handle is None:
            self.get_logger().warn("Requested cancel, but no current goal handle.")
            self.follow_path_active = False
            # If a pending goal exists, we can try to send it right away
            if self.pending_path_goal is not None:
                path = self.pending_path_goal
                self.pending_path_goal = None
                self.send_follow_path_goal(path)
            return

        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)


    def cancel_done_callback(self, future):
        """Called when the server responds to our cancel request."""
        cancel_response = future.result()
        self.get_logger().info(f"FollowPath cancel response: {cancel_response}")

        # the previous goal no longer active.
        self.follow_path_active = False
        self.current_goal_handle = None

        self.last_feedback_index = None
        self.last_progress_time = None

        # If we have a pending goal, send it now
        if self.pending_path_goal is not None:
            path = self.pending_path_goal
            self.pending_path_goal = None
            self.get_logger().info(
                f"Sending queued FollowPath goal with {len(path.poses)} poses."
            )
            self.send_follow_path_goal(path)

    def send_follow_path_goal(self, path: Path):
            """Send a FollowPath goal to the controller."""

            if not self.follow_path_client.wait_for_server(timeout_sec=0.5):
                self.get_logger().error("FollowPath action server not available.")
                return
            
            self.current_path_length = len(path.poses)
            self.early_completion_sent = False

            goal_msg = FollowPath.Goal()
            goal_msg.path = path

            send_future = self.follow_path_client.send_goal_async(
                goal_msg,
                feedback_callback=self.follow_path_feedback_cb,
            )
            send_future.add_done_callback(self.follow_path_goal_response_cb)


    def follow_path_goal_response_cb(self, future):
        """Called when the controller accepts/rejects the FollowPath goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("FollowPath goal rejected by controller.")
            self.current_goal_handle = None
            self.follow_path_active = False
            return

        self.get_logger().info("FollowPath goal accepted by controller.")
        self.current_goal_handle = goal_handle
        self.follow_path_active = True

        self.last_feedback_index = None
        self.last_progress_time = self.get_clock().now()

        # Monitor result to clear state when done
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.follow_path_result_cb)


    def follow_path_result_cb(self, future):
        """Called when the FollowPath goal finishes (success/fail)."""
        result = future.result().result
        msg = Bool()
        msg.data = result.success
        self.goal_complete_pub.publish(msg)
        self.get_logger().info(
            f"FollowPath finished: success={result.success}, msg='{result.message}'"
        )

        # Mark no longer active
        self.follow_path_active = False
        self.current_goal_handle = None
        self.pending_path_goal = None

        if not self.trim_paths_enabled:
            self.trim_paths_enabled = True
            self.get_logger().info("Path execution complete → path trimming re-enabled.")


        self.last_feedback_index = None
        self.last_progress_time = None

        if (self.estop_replan_enabled
            and not result.success
            and ("E-STOP" in result.message)
            and self.last_goal is not None):

            if self.estop_replan_attempts < self.estop_max_replans:
                self.estop_replan_attempts += 1
                self.get_logger().warn(
                    f"E-STOP abort detected; replanning to last goal "
                    f"(attempt {self.estop_replan_attempts}/{self.estop_max_replans})."
                )
                # Replan using the latest map + current pose to the same goal pose
                self.suppress_estop_reset = True
                self.plan_path(self.last_goal)
            else:
                self.get_logger().warn(
                    f"Max E-STOP replans reached ({self.estop_max_replans}); "
                    "not replanning this goal again."
                )



    def follow_path_feedback_cb(self, feedback_msg):
        """Called periodically with FollowPath feedback."""
        feedback = feedback_msg.feedback
        idx = feedback.current_index

        self.get_logger().debug(
            f"FollowPath feedback: at waypoint {idx}"
        )

        now = self.get_clock().now()

        #feedback or progress (index increased)
        if self.last_feedback_index is None or idx > self.last_feedback_index:
            self.last_feedback_index = idx
            self.last_progress_time = now

        # if (self.early_completion_enabled and not self.early_completion_sent and self.current_path_length > 0
        # ):
        #     # fraction of path completed (waypoints visited)
        #     frac = float(idx + 1) / float(self.current_path_length)

        #     if frac >= self.completion_threshold:
        #         self.get_logger().info(
        #             f"Reached {frac * 100.0:.1f}% of path
        #             f"(threshold={self.completion_threshold * 100.0:.1f}%). "
        #             "Treating path as complete."
        #         )

        #         # Tell Explorer goal is "complete"
        #         msg = Bool()
        #         msg.data = True
        #         self.goal_complete_pub.publish(msg)

        #         # Cancel the current FollowPath goal so controller stops
        #         if self.current_goal_handle is not None:
        #             try:
        #                 self.current_goal_handle.cancel_goal_async()
        #             except Exception as e:
        #                 self.get_logger().warn(f"Error canceling FollowPath after early completion: {e}")

        #         # Clear state (replanning logic still uses follow_path_active / current_goal_handle)
        #         self.follow_path_active = False
        #         self.current_goal_handle = None
        #         self.pending_path_goal = None

        #         self.early_completion_sent = True
        #         self.last_feedback_index = None
        #         self.last_progress_time = None

    def check_follow_path_stall(self):
        """
        Periodically called by a timer to detect if the FollowPath action
        has stopped making progress (waypoint index not increasing).
        If stalled, cancel the goal and publish goal_complete=True so Explorer
        can pick a new frontier.
        """
        # No active goal → nothing to do
        if not self.follow_path_active or self.current_goal_handle is None:
            return

        if self.last_progress_time is None:
            # Haven't seen feedback yet; give it time
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_progress_time).nanoseconds / 1e9  # seconds

        if elapsed > self.progress_timeout:
            self.get_logger().warn(
                f"FollowPath appears stalled: no progress for {elapsed:.1f} s "
                f"(timeout={self.progress_timeout:.1f} s). Canceling goal and "
                "notifying Explorer."
            )

            # Try to cancel the current goal (we don't queue a new one here)
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"Error canceling stalled FollowPath goal: {e}")

            # Tell Explorer that this goal is "complete" so it can choose a new one
            msg = Bool()
            msg.data = False
            self.goal_complete_pub.publish(msg)

            # Clear all FollowPath state
            self.follow_path_active = False
            self.current_goal_handle = None
            self.pending_path_goal = None
            self.last_feedback_index = None
            self.last_progress_time = None
    
    def find_nearest_walkable_cell(self, mapdata: OccupancyGrid, start: tuple[int, int]) -> tuple[int, int] | None:
        """
        Given a starting grid cell, search outward in the given map for the nearest
        walkable cell (according to is_cell_walkable).

        Returns the grid cell (x, y) if found, or None if no such cell exists.
        """

        if mapdata is None:
            self.get_logger().warn("find_nearest_walkable_cell: mapdata is None")
            return None

        width  = mapdata.info.width
        height = mapdata.info.height

        def in_bounds(x: int, y: int) -> bool:
            return 0 <= x < width and 0 <= y < height

        sx, sy = start

        # Clamp start into bounds so BFS has a valid seed
        if not in_bounds(sx, sy):
            sx = min(max(sx, 0), width  - 1)
            sy = min(max(sy, 0), height - 1)

        visited: set[tuple[int, int]] = set()
        q = deque()

        q.append((sx, sy))
        visited.add((sx, sy))

        # 8-connected neighbor offsets
        neighbor_offsets = [
            ( 0,  1),  # north
            ( 1,  0),  # east
            ( 0, -1),  # south
            (-1,  0),  # west
            ( 1,  1),  # NE
            (-1,  1),  # NW
            ( 1, -1),  # SE
            (-1, -1),  # SW
        ]

        while q:
            x, y = q.popleft()
            cell = (x, y)

            # First valid walkable cell we see is the closest in grid distance
            if self.is_cell_walkable(mapdata, cell):
                return cell

            for dx, dy in neighbor_offsets:
                nx = x + dx
                ny = y + dy
                nb = (nx, ny)

                if not in_bounds(nx, ny):
                    continue
                if nb in visited:
                    continue

                visited.add(nb)
                q.append(nb)

        # No walkable cell found
        return None


    @staticmethod
    def step_dir(a: tuple[int, int], b: tuple[int, int]) -> tuple[int, int]:
                # Return discrete direction (dx, dy) from node a to b.
                return (b[0] - a[0], b[1] - a[1])

    @staticmethod
    def angle_between_dirs(d1: tuple[int, int], d2: tuple[int, int]) -> float:
        # Angle between two integer directions in radians.
        dx1, dy1 = d1
        dx2, dy2 = d2
        n1 = math.hypot(dx1, dy1)
        n2 = math.hypot(dx2, dy2)
        if n1 == 0.0 or n2 == 0.0:
            return 0.0
        dot = dx1 * dx2 + dy1 * dy2
        cos_th = max(-1.0, min(1.0, dot / (n1 * n2)))
        return math.acos(cos_th)
    
def main(args=None):
    rclpy.init(args=args)
    Path_planner = PathPlanner()
    try:
        rclpy.spin(Path_planner)
        
    except KeyboardInterrupt:
        pass

    finally:

        Path_planner.destroy_node()
        rclpy.shutdown()  

if __name__ == '__main__':
    main()
