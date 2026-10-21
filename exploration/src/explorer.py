#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path, OccupancyGrid, GridCells
from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion, Point
from scipy.spatial.transform import Rotation 
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import tf2_geometry_msgs
#import the chosen callback group type from rclpy.executors
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float32, String, Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import math
import time
import collections
import cv2
import numpy as np
from collections import deque
from nav2_msgs.srv import SaveMap

class Explorer(Node):
    def __init__(self):

        # Initialize node, name it 'controller'
        super().__init__('explorer')

        # Version 1: Transient Local
        qos_1 = QoSProfile(depth=10)
        qos_1.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Version 2: Volatile
        qos_2 = QoSProfile(depth=10)
        qos_2.durability = QoSDurabilityPolicy.VOLATILE

        #subscribtions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.handle_map, qos_profile=qos_1)
        self.goal_complete_sub = self.create_subscription(Bool, '/path_planner/goal_complete', self.handle_goal_complete, qos_profile=qos_1)
        
        #publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', qos_2)
        self.home_goal_pub = self.create_publisher(PoseStamped, '/explorer/go_home', qos_2)
        self.frontier_pub = self.create_publisher(GridCells, '/explorer/frontiers', qos_2)
        self.state_pub = self.create_publisher(String, '/explorer/state', 10)
        self.blob_centers_pub = self.create_publisher(GridCells, '/explorer/blob_centers', qos_2)
        self.cspace_map_pub = self.create_publisher(OccupancyGrid, '/cspace_map', qos_1)
        self.C_space_pub = self.create_publisher(GridCells, '/explorer/c_obstacle', qos_profile=qos_1)
        self.robot_cell_pub = self.create_publisher(GridCells,'/robot/current_grid_cell', 10)
        self.risk_map_pub = self.create_publisher(OccupancyGrid, '/explorer/risk_map', qos_1)

        # Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Frames
        self.map_frame = 'map'
        self.base_frame = 'base_link'

        #State
        self.state = "WAIT_FOR_MAP"
        self.map = None
        self.map_arr = None
        self.cspace_map = None
        self.new_map = False
        self.map_saved = False
        self.current_goal = None
        self.risk_map = None
        self.risk_arr = None
        self.cspace_arr = None
        self.goal_active = False
        self.walkable_mask = None
        self.clearance_mask = None
        self.goal_tolerance = 0.05 #meters
        self.map_update_count = 0

        self.debug = True #False

        self.frontier_centers_grid = []
        self.too_close_count = 0
        self.too_close_threshold = 2

        self.padding = self.declare_parameter(
            'padding', 3
        ).value

        self.home_pose = None
        
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
    def grid_to_world(mapdata: OccupancyGrid, p: tuple[int, int]) -> Point:
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param p [(int, int)] The cell coordinate.
        :return        [Point]         The position in the world.
        """
        
        # create vars for the origin and the resolution 
        origin = mapdata.info.origin
        resolution = mapdata.info.resolution

        # grab the x and y coord
        Coord_x = p[0]
        Coord_y = p[1]

        # change it to world coords
        world_coord_x = (origin.position.x) + (Coord_x + 0.5) * resolution
        world_coord_y = (origin.position.y) + (Coord_y + 0.5) * resolution

        # return them as a point
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

        # grab x and y value, use formula to change to grid coords, return as a tuple
        origin = mapdata.info.origin
        resolution = mapdata.info.resolution

        world_coord_x = wp.x
        world_coord_y = wp.y

        Coord_x = (world_coord_x - origin.position.x)/resolution
        Coord_y = (world_coord_y - origin.position.y)/resolution 

        tup = (int(Coord_x), int(Coord_y))

        return tup

    def handle_map(self, chart:OccupancyGrid):
        """
        Store the incoming map data in a global variable. 
        This function should be called and the map data updated whenever it is available.
        """

        # set the map for the edge of the unknown to recognize when a map is recieved
        self.map = chart
        self.new_map = True
        # change the occu grid to an array for the cv to work
        self.map_arr = Explorer.occgrid_to_array(self.map)

        # visalize the location of the robot
        self.publish_robot_cell()
        
        # calc the cspace and publish it and change it to an array for cv
        self.cspace_map = self.calc_cspace(chart, padding=self.padding)
        self.cspace_map_pub.publish(self.cspace_map)
        self.cspace_arr = Explorer.occgrid_to_array(self.cspace_map)
        #calc risk map to use in a* calc
        self.risk_map = self.calc_risk_map(self.cspace_map)

        self.map_update_count = self.map_update_count + 1

        if self.cspace_arr is not None:
            # walkable = known free or low-probability cells
            self.walkable_mask = (self.cspace_arr >= 0) & (self.cspace_arr <= 50)

            # blocked = obstacles or unknown
            blocked = (self.cspace_arr == -1) | (self.cspace_arr > 50)

            # dist from a blocked space 
            clearance_cells = 1  # must match has_clearance()
            # send a kernal with a 3x3 grid surrounding curr location
            ksize = 2 * clearance_cells + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (ksize, ksize))

            # any cell that has a blocked neighbor within Chebyshev distance 1 becomes blocked
            blocked_dilated = cv2.dilate(blocked.astype(np.uint8), kernel)
            self.clearance_mask = (blocked_dilated == 0)
        else:
            self.walkable_mask = None
            self.clearance_mask = None

        if not self.goal_active:
            self.edge_of_the_unknown()

    def publish_robot_cell(self):
        """
        Publish the robot's current grid cell as a single GridCells message
        so it can be visualized in RViz.
        """
        if self.map is None:
            return
        
        stamped_pose = self.get_robot_pose()
        if stamped_pose is None:
            return

        grid = self.world_to_grid(self.map, stamped_pose.pose.position)

        gc = GridCells()
        gc.header.frame_id = self.map_frame
        gc.header.stamp = self.get_clock().now().to_msg()
        gc.cell_width = self.map.info.resolution
        gc.cell_height = self.map.info.resolution

        pt = self.grid_to_world(self.map, grid)
        gc.cells = [pt]

        self.robot_cell_pub.publish(gc)

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

        index = Explorer.grid_to_index(mapdata,p)
        probability = mapdata.data[index]

        return 0 <= probability <= 50
        
    @staticmethod
    def neighbors_of_4(mapdata: OccupancyGrid, p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the 4-neighbor cells of (x,y) that are inside map bounds.
        """
        x, y = p
        candidates = [
            (x,     y + 1),  # north
            (x + 1, y    ),  # east
            (x,     y - 1),  # south
            (x - 1, y    ),  # west
        ]

        w = mapdata.info.width
        h = mapdata.info.height

        neighbors: list[tuple[int, int]] = []
        for nx, ny in candidates:
            if 0 <= nx < w and 0 <= ny < h:
                neighbors.append((nx, ny))
        return neighbors

    
    @staticmethod
    def neighbors_of_8(mapdata: OccupancyGrid, p: tuple[int, int], walkable_only: bool = True) -> list[tuple[int, int]]:
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

        # for each of the cells that are in a 8 square around it make sure that they are walkable then append then to final list
        if walkable_only:
            for i in range(len(init_nav_list)):
                nav_possibility = init_nav_list[i]
                walkable = Explorer.is_cell_walkable(mapdata,nav_possibility)
                if walkable:
                    final_nav_list.append(nav_possibility)
        else:
            final_nav_list = init_nav_list

        return final_nav_list
    
    @staticmethod
    def get_cell_value (mapdata:OccupancyGrid, p: tuple[int, int]) -> int:
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
            return 100

        index = Explorer.grid_to_index(mapdata,p)
        probability = mapdata.data[index]

        return probability
    
    def compute_frontier_mask(self) -> np.ndarray:
        """
        Compute a boolean 2D mask of frontier cells from the *raw* map (self.map).

        A frontier cell is:
        - free in the raw map (value == 0)
        - and has at least one 4-connected neighbor that is unknown (value == -1)
        """
        if self.map is None:
            return None

        if self.map_arr is not None:
            data = self.map_arr
        else:
            data = self.occgrid_to_array(self.map)

        # grab the cells that are free and the ones that are unknown 
        free_mask    = (data == 0)
        unknown_mask = (data == -1)

        # 4-connected neighbors kernel
        kernel = np.array([ [0, 1, 0],
                            [1, 0, 1],
                            [0, 1, 0]], dtype=np.uint8)

        # unknown_neighbor[y,x] = True if at least one 4-neighbor of (y,x) is unknown
        unknown_neighbor = cv2.filter2D(
            unknown_mask.astype(np.uint8),
            ddepth=-1,
            kernel=kernel
        ) > 0

        frontier_mask = free_mask & unknown_neighbor
        return frontier_mask
    
    def frontier_detector(self, mapdata: OccupancyGrid, start: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Find frontier cells: free cells (0) in the raw map that border unknown cells (-1).

        Core logic is the original working BFS frontier detector, with added:
        - self.map / self.cspace_map checks
        - in-bounds helper
        - "fix" non-walkable start by searching a small radius in C-space
        - C-space used for BFS expansion, raw map used for frontier test
        """

        if self.map is None or mapdata is None:
            return []

        # Frontier mask is defined in raw map coordinates
        frontier_mask = self.compute_frontier_mask()
        if frontier_mask is None:
            return []

        width  = mapdata.info.width
        height = mapdata.info.height
        
        # check if the point passed in is in bounds of the map
        def in_bounds(x: int, y: int) -> bool:
            return 0 <= x < width and 0 <= y < height
        

        # if not in bounds then return 0
        sx, sy = start
        if not in_bounds(sx, sy):
            return []

        # If start isn't walkable, try to find a nearby walkable cell in a small radius
        #bounded BFS
        if not self.is_cell_walkable(mapdata, (sx, sy)):
            found = None
            max_radius = 2
            for r in range(1, max_radius + 1):
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        nx = sx + dx
                        ny = sy + dy
                        if not in_bounds(nx, ny):
                            continue
                        if self.is_cell_walkable(mapdata, (nx, ny)):
                            found = (nx, ny)
                            break
                if found is not None:
                    break

            if found is None:
                return []
            sx, sy = found

        start = (sx, sy)

        frontiers: list[tuple[int, int]] = []
        visited: set[tuple[int, int]] = set()
        queue = collections.deque([start])

        while queue:
            x, y = queue.popleft()

            # if the current cell is already visited then continue past
            if (x, y) in visited:
                continue
            visited.add((x, y))

            # if not in bounds then also continue past
            if not in_bounds(x, y):
                continue

            # if an obstacle then continue past
            if not self.is_cell_walkable(mapdata, (x, y)):
                continue

            # check frontier mask to see if marked as a frontier
            if frontier_mask[y, x]:
                frontiers.append((x, y))

            # BFS expansion through neighbors of 4
            for nx, ny in self.neighbors_of_4(mapdata, (x, y)):
                if (nx, ny) in visited:
                    continue
                if not in_bounds(nx, ny):
                    continue
                if not self.is_cell_walkable(mapdata, (nx, ny)):
                    continue
                queue.append((nx, ny))

        if self.debug:
            self.get_logger().info(
                f"frontier_detector: found {len(frontiers)} frontier cells "
                f"(visited {len(visited)} cells)"
            )

        return frontiers

    def has_clearance(self, map: OccupancyGrid, cell: tuple[int, int]) -> bool:
        """
        Returns True if all cells within `clearance_cells` (Chebyshev distance) (check all obstacles in a square not a circle)
        of 'cell' are not occupied (> occ_threshold) or unknown (-1).
        """
        # checks if frontier center is not within one grid cell of a wall
            # box around grid center if obstacle or unknown then false
        # check 3x3 square 
        clearance_cells = 1
        occ_threshold = 50

        x0, y0 = cell
        width  = map.info.width
        height = map.info.height
        data   = map.data

        # check if in bounds of map
        def in_bounds(x: int, y: int) -> bool:
            return 0 <= x < width and 0 <= y < height
        
        # create a 3x3 grid around the center cell
        for dy in range(-clearance_cells, clearance_cells + 1):
            for dx in range(-clearance_cells, clearance_cells + 1):
                x = x0 + dx
                y = y0 + dy

                if not in_bounds(x, y):
                    return False
                
                # check occupancy grid
                idx = self.grid_to_index(map, (x, y))
                val = data[idx]

                # treat unknown and occupied as unsafe in the clearance square
                if val == -1 or val > occ_threshold:
                    return False

        return True

    def shift_to_know(self, center: tuple[int, int]) -> tuple[int, int] | None:
        """
        Given a frontier centroid in grid coords, search outward using 8-connected
        neighbors (BFS) up to max_steps cells away and choose a target cell that:
        - is in known free space (walkable)
        - has clearance from walls (has_clearance)
        - has the lowest risk (distance from walls) in that local neighborhood
            according to the risk map
        - if multiple cells share the same risk, choose the one closest to the
            original frontier center in grid distance
        """

        # make it so the function prefers c_space if possible and return none if no map
        local_map = self.cspace_map if self.cspace_map is not None else self.map
        if local_map is None:
            if self.debug:
                self.get_logger().warn("shift_to_know: no local_map (cspace/map is None)")
            return None
        
        # if using precomputed numpy array /masks enables fast array lookup
            # otherwise use traditional map data structure
        use_mask_fastpath = (
            local_map is self.cspace_map and
            self.cspace_arr is not None and
            self.walkable_mask is not None and
            self.clearance_mask is not None
        )

        if use_mask_fastpath:
            height, width = self.cspace_arr.shape
        else:
            width  = local_map.info.width
            height = local_map.info.height

        # risk array shortcut
        risk_map = self.risk_map
        risk_arr = self.risk_arr if self.risk_arr is not None else None

        # search radius of 5 cells out (BFS)
        max_steps = 5
        visited: set[tuple[int, int]] = set()
        frontier: list[tuple[int, int]] = [center]
        visited.add(center)

        candidates: list[tuple[int, int]] = []
        steps = 0

        def in_bounds(x: int, y: int) -> bool:
            return 0 <= x < width and 0 <= y < height
        
        # main BFS loop
        while frontier and steps < max_steps:
            next_frontier: list[tuple[int, int]] = []

            # process the current BFS layer
            for cell in frontier:
                cx, cy = cell

                # --- candidate test using fast masks if possible ---
                if use_mask_fastpath and in_bounds(cx, cy):
                    if self.walkable_mask[cy, cx] and self.clearance_mask[cy, cx]:
                        candidates.append(cell)
                        if self.debug:
                            self.get_logger().info(
                                f"shift_to_know: candidate (fast) at {cell}"
                            )
                else:
                    # Fallback: original semantics
                    if self.is_cell_walkable(local_map, cell) and self.has_clearance(local_map, cell):
                        candidates.append(cell)
                        if self.debug:
                            self.get_logger().info(
                                f"shift_to_know: candidate (fallback) at {cell}"
                            )

                # Expand search to 8-connected neighbors (walkable-only)
                neighbors = self.neighbors_of_8(local_map, cell)
                for nb in neighbors:
                    if nb not in visited:
                        visited.add(nb)
                        next_frontier.append(nb)

            frontier = next_frontier
            steps += 1

        if self.debug:
            self.get_logger().info(
                f"shift_to_know: total candidates after BFS = {len(candidates)}"
            )

        # check if canidates found
        if not candidates:
            if self.debug:
                self.get_logger().warn(
                    "shift_to_know: no candidates found (walkable+clear cells) within search radius"
                )
            return None

        center_x, center_y = center

        def grid_dist2_to_center(cell: tuple[int, int]) -> int:
            x, y = cell
            dx = x - center_x
            dy = y - center_y
            return dx * dx + dy * dy

        # Risk-based selection if we have a risk map
        if risk_map is not None:
            best_cell: tuple[int, int] | None = None
            best_risk: int | None = None
            best_dist2: int | None = None

            # calc each canidate within risk map
            for cell in candidates:
                x, y = cell
                if not in_bounds(x, y):
                    continue

                if risk_arr is not None:
                    risk_value = int(risk_arr[y, x])
                else:
                    # fallback: index into risk_map.data
                    idx = self.grid_to_index(risk_map, cell)
                    risk_value = risk_map.data[idx]

                if risk_value < 0:
                    # skip obstacles/unknown in risk map
                    if self.debug:
                        self.get_logger().info(
                            f"shift_to_know: skipping {cell} due to risk_value {risk_value}"
                        )
                    continue

                # select based on risk 
                dist2 = grid_dist2_to_center(cell)

                if best_cell is None:
                    best_cell = cell
                    best_risk = risk_value
                    best_dist2 = dist2
                    if self.debug:
                        self.get_logger().info(
                            f"shift_to_know: initial best_cell {cell} with risk {risk_value}, dist2 {dist2}"
                        )
                    continue

                if risk_value < best_risk:
                    if self.debug:
                        self.get_logger().info(
                            f"shift_to_know: new best_cell {cell} with lower risk {risk_value} "
                            f"(was {best_risk})"
                        )
                    best_cell = cell
                    best_risk = risk_value
                    best_dist2 = dist2
                elif risk_value == best_risk and dist2 < best_dist2:
                    if self.debug:
                        self.get_logger().info(
                            f"shift_to_know: tie on risk {risk_value}, but {cell} is closer "
                            f"(dist2 {dist2} < {best_dist2}), updating best"
                        )
                    best_cell = cell
                    best_dist2 = dist2

            # if all the canidates have invalid risk values then use geometric fallback
            if best_cell is None: 
                if self.debug:
                    self.get_logger().warn(
                        "shift_to_know: all candidates had invalid risk"
                        "falling back to closest-to-center"
                    )
                best_cell = candidates[0]
                best_dist2 = grid_dist2_to_center(best_cell)
                for cell in candidates[1:]:
                    dist2 = grid_dist2_to_center(cell)
                    if dist2 < best_dist2:
                        best_cell = cell
                        best_dist2 = dist2

            if self.debug:
                self.get_logger().info(
                    f"shift_to_know: final chosen cell {best_cell} "
                    f"(risk={best_risk}, dist2={best_dist2})"
                )

            return best_cell
        
        # if go through all and no canidate then choose canidate closest to og center
        best_cell = candidates[0]
        best_dist2 = grid_dist2_to_center(best_cell)

        for cell in candidates[1:]:
            dist2 = grid_dist2_to_center(cell)
            if dist2 < best_dist2:
                best_cell = cell
                best_dist2 = dist2

        if self.debug:
            self.get_logger().info(
                f"shift_to_know: risk_map is None; chosen closest cell {best_cell} "
                f"with dist2={best_dist2}"
            )

        return best_cell


    def edge_of_the_unknown(self):
        """
        1. Get robot pose in map frame.
        2. Convert to grid coordinate (start).
        3. Find frontier cells via frontier_detector.
        4. Cluster into blobs, compute centroids, shift into known free cells.
        5. Pick closest shifted center to robot and publish as /move_base_simple/goal.
        """

        local_map = self.cspace_map if self.cspace_map is not None else self.map

        if local_map is None:
            self.get_logger().warn("Explorer: edge_of_the_unknown has no map to work with.")
            return

        stamped_pose = self.get_robot_pose()
        if stamped_pose is None:
            if self.debug:
                self.get_logger().warn("edge_of_the_unknown: no robot pose available")
            return
        
        # save first valid pose as "home" for go home functionality
        if self.home_pose is None:
            self.home_pose = PoseStamped()
            self.home_pose.header.frame_id = self.map_frame
            self.home_pose.pose = stamped_pose.pose

        robot_pt = Point()
        robot_pt.x = stamped_pose.pose.position.x
        robot_pt.y = stamped_pose.pose.position.y
        robot_pt.z = stamped_pose.pose.position.z

        grid_start = self.world_to_grid(local_map, robot_pt)

        # grab list of frontiers
        frontier_list = self.frontier_detector(local_map, grid_start)  # list[(int,int)]
        if self.debug:
            self.get_logger().info(
                f"edge_of_the_unknown: robot grid {grid_start}, "
                f"found {len(frontier_list)} frontier cells"
            )

        # once there is no more frontiers then go home
        if not frontier_list:
            self.get_logger().warn("Explorer: no frontiers detected in edge_of_the_unknown.")
            self.go_home()
            return

        # publish raw frontier cells
        gc = GridCells()
        gc.header.frame_id = self.map_frame
        gc.header.stamp = self.get_clock().now().to_msg()
        gc.cell_width = self.map.info.resolution
        gc.cell_height = self.map.info.resolution
        gc.cells = [self.grid_to_world(self.map, c) for c in frontier_list]
        self.frontier_pub.publish(gc)

        # take the frontiers and cluster them into points for the robot to explore
        blobs = self.cluster_frontiers(frontier_list, max_gap=1)
        if self.debug:
            self.get_logger().info(
                f"edge_of_the_unknown: clustered frontiers into {len(blobs)} blobs"
            )

        if not blobs:
            self.get_logger().warn("Explorer: no blobs formed from frontier set.")
            return

        candidate_world_points = []
        new_centers_grid = []

        # find centroids
        for i, blob in enumerate(blobs):
            xs = [c[0] for c in blob]
            ys = [c[1] for c in blob]
            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)
            centroid = (int(round(cx)), int(round(cy)))

            if self.debug:
                self.get_logger().info(
                    f"edge_of_the_unknown: blob {i} size={len(blob)}, "
                    f"centroid approx={centroid}"
                )
            
            # shift target point from centroid to safe place
            shifted_grid = self.shift_to_know(centroid)
            if shifted_grid is None:
                if self.debug:
                    self.get_logger().warn(
                        f"edge_of_the_unknown: shift_to_know returned None for blob {i} "
                        f"(centroid {centroid})"
                    )
                continue

            if self.debug:
                self.get_logger().info(
                    f"edge_of_the_unknown: blob {i} centroid {centroid} shifted to {shifted_grid}"
                )

            new_centers_grid.append(shifted_grid)

        if self.debug:
            self.get_logger().info(
                f"edge_of_the_unknown: new_centers_grid size = {len(new_centers_grid)}"
            )

        # Prune old global centers that are no longer valid frontier centers as map update
        if self.debug:
            self.get_logger().info(
                f"edge_of_the_unknown: global centers before prune = "
                f"{len(self.frontier_centers_grid)}"
            )

        pruned_global = []
        for cell in self.frontier_centers_grid:
            if self.is_frontier_center(self.map, cell):
                pruned_global.append(cell)
            elif self.debug:
                self.get_logger().info(
                    f"edge_of_the_unknown: dropping old center {cell} "
                    f"(no longer a frontier center)"
                )
        self.frontier_centers_grid = pruned_global

        # Add new centers (avoid duplicates)
        for c in new_centers_grid:
            if c not in self.frontier_centers_grid:
                self.frontier_centers_grid.append(c)

        if self.debug:
            self.get_logger().info(
                f"edge_of_the_unknown: global centers after update = "
                f"{len(self.frontier_centers_grid)}"
            )

        # if we still have no centers, map is likely conquered around here
        if not self.frontier_centers_grid:
            self.get_logger().info("Explorer: no global frontier centers remain; map conquered.")
            self.go_home()
            return

        candidate_world_points = [
            self.grid_to_world(self.map, c) for c in self.frontier_centers_grid
        ]

        if not candidate_world_points:
            self.get_logger().warn("Explorer: no valid shifted frontier centers (all blocked?).")
            return

        # Visualize current centers
        centers_gc = GridCells()
        centers_gc.header.frame_id = self.map_frame
        centers_gc.header.stamp = self.get_clock().now().to_msg()
        centers_gc.cell_width = self.map.info.resolution
        centers_gc.cell_height = self.map.info.resolution
        centers_gc.cells = candidate_world_points
        self.blob_centers_pub.publish(centers_gc)

        #Choose the nearest center, but if it's too close, increment a counter and wait for a new map.
        upper_limit = .10
        lower_limit = .05
        close_thresh = max(min(2.0 * self.map.info.resolution, upper_limit), lower_limit)

        # Find closest center
        best_idx = None
        best_dist = float('inf')

        for i, wp in enumerate(candidate_world_points):
            d = math.hypot(wp.x - robot_pt.x, wp.y - robot_pt.y)
            if d < best_dist:
                best_dist = d
                best_idx = i

        if self.debug:
            self.get_logger().info(
                f"edge_of_the_unknown: nearest frontier center index={best_idx}, "
                f"distance={best_dist:.3f} m"
            )

        if best_idx is None:
            self.get_logger().warn("Explorer: could not select a frontier center.")
            return

        nearest_grid = self.frontier_centers_grid[best_idx]
        nearest_pt = candidate_world_points[best_idx]

        #if nearest is too close, either wait or discard depending on count
        if best_dist < close_thresh:
            self.too_close_count += 1

            if self.too_close_count < self.too_close_threshold:
                # Just wait for another map update, don't send a new goal
                self.get_logger().info(
                    f"Nearest frontier center at ({nearest_pt.x:.2f}, {nearest_pt.y:.2f}) "
                    f"is very close ({best_dist:.3f} m). "
                    f"Waiting for new map ({self.too_close_count}/"
                    f"{self.too_close_threshold}) before skipping it."
                )
                self.goal_active = False
                return
            else:
                # We've seen this too-close situation enough times; drop this center
                self.get_logger().info(
                    f"Nearest frontier center at ({nearest_pt.x:.2f}, {nearest_pt.y:.2f}) "
                    f"remained too close for {self.too_close_threshold} map updates. "
                    f"Discarding it and searching for a further frontier."
                )
                del self.frontier_centers_grid[best_idx]
                self.too_close_count = 0

                # if there is no frontiers left after getting rid of too close point then false
                if not self.frontier_centers_grid:
                    self.get_logger().info(
                        "Explorer: no global frontier centers remain after discarding close ones; "
                        "map may be conquered."
                    )
                    self.goal_active = False
                    return
                
                # if frontiers remain then recalc the nearest frontier
                candidate_world_points = [
                    self.grid_to_world(self.map, c) for c in self.frontier_centers_grid
                ]

                best_idx = None
                best_dist = float('inf')
                for i, wp in enumerate(candidate_world_points):
                    d = math.hypot(wp.x - robot_pt.x, wp.y - robot_pt.y)
                    if d < best_dist:
                        best_dist = d
                        best_idx = i

                if best_idx is None:
                    self.get_logger().warn("Explorer: could not select a frontier center after discard.")
                    return
                
                nearest_pt = candidate_world_points[best_idx]

        best_pt = nearest_pt

        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position = best_pt
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self.current_goal = best_pt
        self.goal_active = True
        self.new_map = False
        self.too_close_count = 0  # reset whenever we actually commit to a new goal

        self.get_logger().info(
            f"Explorer: sent frontier goal at ({best_pt.x:.2f}, {best_pt.y:.2f}) in {self.map_frame}"
        )

        self.state = "WAIT_FOR_MAP"



    def cluster_frontiers(self, frontier_list: list[tuple[int, int]], max_gap: int = 2):
        """
        Cluster frontier cells into larger blobs by grouping any frontier
        cells that are within max_gap grid cells of each other
        """
        # handles making the blobs 
            # 2 is the threshold
            # needs 5 by 5 grid and will count all frontiers within to be covered by blob
        if not frontier_list:
            return []

        xs = np.array([c[0] for c in frontier_list], dtype=np.int32)
        ys = np.array([c[1] for c in frontier_list], dtype=np.int32)

        min_x, max_x = xs.min(), xs.max()
        min_y, max_y = ys.min(), ys.max()

        width  = max_x - min_x + 1
        height = max_y - min_y + 1

        # Binary mask of frontier cells in a tight bounding box
        mask = np.zeros((height, width), dtype=np.uint8)
        mask[ys - min_y, xs - min_x] = 1

        # Dilate to connect cells within max_gap
        if max_gap > 0:
            ksize = 2 * max_gap + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (ksize, ksize))
            dilated = cv2.dilate(mask, kernel)
        else:
            dilated = mask

        # connected components on the dilated mask
        num_labels, labels = cv2.connectedComponents(dilated)

        # Group original frontier cells by component label
        blobs_dict: dict[int, list[tuple[int, int]]] = {
            label: [] for label in range(1, num_labels)
        }

        for (x, y) in frontier_list:
            lx = x - min_x
            ly = y - min_y
            label = labels[ly, lx]
            if label == 0:
                continue
            blobs_dict[label].append((x, y))

        blobs = [cells for cells in blobs_dict.values() if cells]
        return blobs

        
    def get_robot_pose(self) -> PoseStamped | None:
        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.5))
        except Exception:
            self.get_logger().warn("TF lookup failed for explorer get_robot_pose()")
            return None

        p = PoseStamped()
        p.header.frame_id = self.map_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = transform.transform.translation.x
        p.pose.position.y = transform.transform.translation.y
        p.pose.position.z = transform.transform.translation.z
        return p
    
    def calc_cspace(self, mapdata: OccupancyGrid, padding: int) -> OccupancyGrid:
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Uses OpenCV dilation for fast inflation, while keeping the original
        denoising semantics between unknown and free.
        """
        width  = mapdata.info.width
        height = mapdata.info.height
        resolution = mapdata.info.resolution

        # Base arrays
        original_arr = Explorer.occgrid_to_array(mapdata)
        filtered_arr = original_arr.copy()

        def in_bounds(x: int, y: int) -> bool:
            return 0 <= x < width and 0 <= y < height

        # at least 5 of 8 must be known and free space 
        MIN_KNOWN_NEIGHBORS = 5
        MIN_FREE_NEIGHBORS  = 5

        #filter island of unknown in free space
        for y in range(height):
            for x in range(width):
                val = original_arr[y, x]
                if val != -1:
                    continue

                known = 0
                free  = 0
                occ   = 0

                # look at 8 connected neighbors
                for ny in range(y - 1, y + 2):
                    if ny < 0 or ny >= height:
                        continue
                    for nx in range(x - 1, x + 2):
                        if nx < 0 or nx >= width:
                            continue
                        if nx == x and ny == y:
                            continue

                        # skip unknown neighbors 
                        nval = original_arr[ny, nx]
                        if nval == -1:
                            continue

                        known += 1
                        if nval > 50:
                            occ += 1
                        elif 0 <= nval <= 50:
                            free += 1

                if (
                    known >= MIN_KNOWN_NEIGHBORS and
                    free  >= MIN_FREE_NEIGHBORS  and
                    occ == 0
                ):
                    filtered_arr[y, x] = 0

        MIN_UNKNOWN_NEIGHBORS = 5
        MAX_KNOWN_NEIGHBORS   = 2

        #filter island knowns in unknown space
        for y in range(height):
            for x in range(width):
                val = original_arr[y, x]
                if not (0 <= val <= 50):
                    continue

                unknown = 0
                known   = 0

                for ny in range(y - 1, y + 2):
                    for nx in range(x - 1, x + 2):
                        if nx == x and ny == y:
                            continue
                        if not in_bounds(nx, ny):
                            continue

                        nval = original_arr[ny, nx]
                        if nval == -1:
                            unknown += 1
                        else:
                            known += 1

                if (
                    unknown >= MIN_UNKNOWN_NEIGHBORS and
                    known   <= MAX_KNOWN_NEIGHBORS
                ):
                    filtered_arr[y, x] = -1

        # Obstacles from original map
        obs_mask = (original_arr > 50).astype(np.uint8)
        
        #Inflate obstacles using OpenCV dilation
        if padding > 0:
            #creates the dilation mask
            ksize = 2 * padding + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (ksize, ksize))
            inflated_mask = cv2.dilate(obs_mask, kernel)
        else:
            inflated_mask = obs_mask

        # Inflate into known free cells in filtered map
        new_map_arr = filtered_arr.copy()
        mask_inflate = (
            (inflated_mask == 1) &
            (filtered_arr >= 0) &
            (filtered_arr <= 50)
        )
        new_map_arr[mask_inflate] = 100

        # Build list of inflated C-space cells for visualization
        Ob_cells = []
        ys, xs = np.where(mask_inflate)
        for y, x in zip(ys, xs):
            Ob_cells.append(self.grid_to_world(mapdata, (int(x), int(y))))

        c_grid = GridCells()
        c_grid.header.frame_id = 'map'
        c_grid.cell_width = resolution
        c_grid.cell_height = resolution
        c_grid.cells = Ob_cells
        self.C_space_pub.publish(c_grid)

        # Build C-space OccupancyGrid
        output = Explorer.array_to_occgrid(new_map_arr, mapdata)
        return output

    def handle_goal_complete(self, msg: Bool):
        """
        Handle completion notification from the path planner.
        msg.data == True  -> path reached successfully
        msg.data == False -> planner failed (e.g., A*: No path found)
        """

        if self.debug:
            self.get_logger().info(
                f"PathPlanner reports goal complete (success={msg.data})"
            )

        # In all cases, we are no longer actively pursuing a goal
        self.goal_active = False
        self.map_update_count = 0

        # If planning FAILED, treat current_goal as unreachable and prune it
        if not msg.data:
            if self.current_goal is not None and self.map is not None:
                # current_goal is a Point in map frame
                unreachable_grid = self.world_to_grid(self.map, self.current_goal)

                # If that grid cell corresponds to one of our frontier centers,
                # remove it — we now know it cannot be reached.
                if unreachable_grid in self.frontier_centers_grid:
                    self.frontier_centers_grid.remove(unreachable_grid)
                    if self.debug:
                        self.get_logger().warn(
                            f"Explorer: removing unreachable frontier center "
                            f"{unreachable_grid} after A* failure."
                        )
                else:
                    # If it *wasn't* one of the known frontier cells, log a warning
                    if self.debug:
                        self.get_logger().warn(
                            "Explorer: A* failed but current_goal's grid cell "
                            f"{unreachable_grid} was not in frontier_centers_grid."
                        )
            else:
                if self.debug:
                    self.get_logger().warn(
                        "Explorer: A* failure reported but current_goal/map is None."
                    )

            # On failure, just wait for the next map; edge_of_the_unknown()
            # will run again on the next handle_map and pick a new target.
            return

        # If we get here, the planner reported SUCCESS
        if self.debug:
            self.get_logger().info("Explorer: goal reached successfully.")

        self.current_goal = None

    def is_frontier_center(self, mapdata: OccupancyGrid, cell: tuple[int, int]) -> bool:
        """
        Return True if there is a frontier center within radius 2 cells
        around this cell. We only walk through walkable cells and
        consider a cell a frontier if it is free and has at least one
        unknown (-1) 8-neighbor.
        """

        # If the starting cell isn't free/walkable, we can't use it
        if not self.is_cell_walkable(mapdata, cell):
            return False

        max_steps = 3
        visited: set[tuple[int, int]] = set()

        frontier = deque([(cell, 0)])
        visited.add(cell)

        while frontier:
            current, dist = frontier.popleft()

            # Check if this cell itself qualifies as a frontier center:
            # - walkable
            # - at least one unknown neighbor
            if self.is_cell_walkable(mapdata, current):
                for nb in self.neighbors_of_8(mapdata, current, False):
                    if self.get_cell_value(mapdata, nb) == -1:
                        return True

            # Stop expanding once we've hit the radius limit
            if dist >= max_steps:
                continue

            # Expand to 8-neighbors, but only through walkable cells
            for nb in self.neighbors_of_8(mapdata, current):
                if nb in visited:
                    continue

                visited.add(nb)
                frontier.append((nb, dist + 1))

        return False
    
    def calc_risk_map(self, map: OccupancyGrid, max_distance_cells: int = 7, max_risk: int = 100) -> OccupancyGrid:
        """
        Build a risk OccupancyGrid from a C-space map
        High risk near inflated obstacles
        Risk decays with grid distance up to max_distance_cells
        added self.risk_weight in the init to use to tune the wall avoidance in A*
        """
        if self.cspace_arr is not None and map is self.cspace_map:
            data = self.cspace_arr
        else:
            data = Explorer.occgrid_to_array(map)

        obstacles = (data > 50)
        unknown   = (data == -1)

        # For distanceTransform:
        #   0 pixels -> obstacles (distance 0)
        #   1 pixels -> everything else (free + unknown)
        img = np.where(obstacles, 0, 1).astype(np.uint8)

        # Distance (in cells) to nearest obstacle
        dist = cv2.distanceTransform(img, cv2.DIST_L2, 3)

        # Risk array
        risk_arr = np.zeros_like(data, dtype=np.int16)

        # Obstacles / unknowns: risk = -1
        risk_arr[obstacles | unknown] = -1

        # Free cells: map distance to risk
        free_mask = (data >= 0) & (data <= 50)

        # Norm mapping similar to your original BFS-based logic
        norm = (max_distance_cells - (dist - 1.0)) / float(max_distance_cells)
        norm = np.clip(norm, 0.0, 1.0)
        risk_vals = (norm * max_risk).round().astype(np.int16)

        risk_arr[free_mask] = risk_vals[free_mask]

        risk_grid = Explorer.array_to_occgrid(risk_arr, map)
        self.risk_arr = risk_arr
        self.risk_map_pub.publish(risk_grid)

        return risk_grid
    
    def save_current_map(self, map_name: str = "explored_map"):
        """
        Save map using the map server service
        """
        # check if have map
        self.map_saved = True
        
        if self.map is None:
            self.get_logger().warn("No map available to save")
            return False
        
        # create client to talk to the map saver service
        client = self.create_client(SaveMap, '/map_saver/save_map')
        
        # wait for the service to be available
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Map saver service not available")
            return False

        request = SaveMap.Request()
        # TODO: CHANGE HERE BASED ON FILE PATH
        maps_dir = "path/to/maps"
        request.map_url = f"{maps_dir}/{map_name}"
        
        # send request to save map
        client.call_async(request)

        return True
    
    def find_nearest_walkable_cell(self, start: tuple[int, int]) -> tuple[int, int] | None:
        """
        Given a starting grid cell, search outward for the nearest walkable cell

        Returns the grid cell (x, y) if found, or None if no such cell exists.
        """
        #bounded BFS to find walkable bounds
        local_map = self.cspace_map if self.cspace_map is not None else self.map
        if local_map is None:
            self.get_logger().warn("find_nearest_walkable_cell: no map available")
            return None

        width  = local_map.info.width
        height = local_map.info.height

        def in_bounds(x: int, y: int) -> bool:
            return 0 <= x < width and 0 <= y < height

        sx, sy = start

        # Clamp start point into valid map range.
        if not in_bounds(sx, sy):
            sx = min(max(sx, 0), width  - 1)
            sy = min(max(sy, 0), height - 1)

        visited: set[tuple[int, int]] = set()
        q = deque()

        q.append((sx, sy))
        visited.add((sx, sy))

        while q:
            cell = q.popleft()
            x, y = cell

            # Check if this candidate cell is walkable AND has the required clearance.
            #   - occupancy checks
            #   - inflated C-space constraints
            if self.is_cell_walkable(local_map, cell) and self.has_clearance(local_map, cell):
                return cell

            # Otherwise expand to all 8-connected neighbors
            # walkable_only=False ensures we still traverse unknown/blocked cells
            # so BFS can step outward until it finds something valid.
            for nb in self.neighbors_of_8(local_map, cell, walkable_only=False):
                nx, ny = nb
                if not in_bounds(nx, ny):
                    continue
                if nb in visited:
                    continue
                visited.add(nb)
                q.append(nb)

        #if nothing found return nothing
        return None

    
    def go_home(self):
        """
        Send the robot back to the saved home pose (initial position).

        If the saved home position is not valid in the C-space (colliding or unknown),
        we instead choose the nearest valid C-space cell and go there.
        """

        #save the current map if it hasn't been saved yet
        if not self.map_saved:
            self.save_current_map("explored_map")

        # We cannot return home without a stored home pose
        if self.home_pose is None:
            self.get_logger().warn(
                "Explorer: no home pose stored; cannot return home."
            )
            return

        # Choose C-space map if available
        local_map = self.cspace_map if self.cspace_map is not None else self.map
        if local_map is None:
            self.get_logger().warn(
                "Explorer: go_home has no map to check C-space validity."
            )
            return

        # World -> grid for home position
        home_world = self.home_pose.pose.position
        home_grid  = self.world_to_grid(local_map, home_world)

        # Check if home cell is valid in C-space
        use_home_cell = (
            self.is_cell_walkable(local_map, home_grid) and
            self.has_clearance(local_map, home_grid)
        )

        #Home location is valid in C-space
        if use_home_cell:
            # Home is already a valid C-space position; keep original position
            target_point = Point()
            target_point.x = home_world.x
            target_point.y = home_world.y
            target_point.z = home_world.z

            self.get_logger().info(
                f"Explorer: home pose is valid in C-space, returning to "
                f"({target_point.x:.2f}, {target_point.y:.2f})"
            )
        else:
            # Home is in collision / unknown in C-space: snap to nearest valid C-space cell
            nearest_cell = self.find_nearest_walkable_cell(home_grid)
            if nearest_cell is None:
                self.get_logger().warn(
                    "Explorer: failed to find a safe C-space cell near home; cannot return home."
                )
                return

            target_point = self.grid_to_world(local_map, nearest_cell)

            self.get_logger().info(
                "Explorer: home pose is not valid in C-space, snapping to nearest "
                f"valid cell at ({target_point.x:.2f}, {target_point.y:.2f})"
            )

        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()

        # Keep the original orientation from home_pose, but update the position
        goal.pose = self.home_pose.pose
        goal.pose.position.x = target_point.x
        goal.pose.position.y = target_point.y
        goal.pose.position.z = target_point.z

        self.goal_pub.publish(goal)

        self.current_goal = goal.pose.position
        self.goal_active = True
        self.state = "RETURN_HOME"

        self.get_logger().info(
            f"Explorer: no more frontiers, returning home to "
            f"({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})"
        )
        # here we have saved map and conrol c to enter diff launch file
        # launch file will differentiate between the slam and amcl


    @staticmethod
    def occgrid_to_array(grid: OccupancyGrid) -> np.ndarray:
        """
        Convert an OccupancyGrid to a 2D numpy array of shape (height, width).

        Indexing is consistent with grid_to_index:
            index = width * y + x
        so arr[y, x] corresponds to cell (x, y) in grid coordinates.
        """
        dtype = np.int16

        width = grid.info.width
        height = grid.info.height

        # flattened layout is row-major with index = y * width + x, i checked this matches :)
        data = np.array(grid.data, dtype=dtype)

        if data.size != width * height:
            raise ValueError(
                f"OccupancyGrid data size {data.size} does not match "
                f"width*height = {width * height}"
            )

        return data.reshape((height, width))

    @staticmethod
    def array_to_occgrid(arr: np.ndarray, template: OccupancyGrid) -> OccupancyGrid:
        """
        Convert a 2D numpy array back into an OccupancyGrid, using an existing
        grid as a template for header and info.

        arr must be shape (height, width) matching template.info.
        """
        dtype = np.int8

        if arr.ndim != 2:
            raise ValueError(
                f"array_to_occgrid expects a 2D array, got shape {arr.shape}"
            )

        height, width = arr.shape
        if width != template.info.width or height != template.info.height:
            raise ValueError(
                f"Array shape {arr.shape} does not match template "
                f"(height={template.info.height}, width={template.info.width})"
            )

        out = OccupancyGrid()
        out.header = template.header
        out.info = template.info

        #flattens the array to 1D list
        out.data = arr.astype(dtype).reshape(-1).tolist()
        return out

def main(args=None):
    rclpy.init(args=args)
    explorer = Explorer()
    try:
        rclpy.spin(explorer)
        
    except KeyboardInterrupt:
        pass

    finally:

        explorer.destroy_node()
        rclpy.shutdown()  

if __name__ == '__main__':
    main()
