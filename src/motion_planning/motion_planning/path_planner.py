from geometry_msgs.msg import PoseArray, PointStamped, Point, PoseStamped


from nav_msgs.msg import OccupancyGrid, Path

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tf_transformations import quaternion_from_euler
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose_stamped

import rclpy
from rclpy.node import Node

import math
import numpy as np

class GRID_STATES:
    EMPTY = 0
    BORDER = -100
    OCCUPIED = 100
    
    def __init__():
        pass

class PathPlanner(Node):
    threshold_distance = 0.16 # half of the robot's width
    target_x = None
    target_y = None
    completed = False
    origin_x = None
    origin_y = None
    map = OccupancyGrid()
    def __init__(self):
        super().__init__('path_planner')
        self.get_logger().info('Welcome to path planner')

        # Initialize the transform listener and assign it a buffer
        self.tf2Buffer = Buffer(cache_time=None)
        self.tfBuffer = Buffer(cache_time=None)

        #transform listener fills the buffer
        self.listener = TransformListener(self.tf2Buffer, self)
        self.listener2 = TransformListener(self.tfBuffer, self)

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)

        self.goal_sub = self.create_subscription(PointStamped, '/plan_goal', self.goal_cb, 10)

        self._point_pub = self.create_publisher(
            PointStamped, '/clicked_point', 10)

        self._poses_pub = self.create_publisher(
            PoseArray, '/planned_poses', 10)

        # Initialize the path publisher
        self._path_pub = self.create_publisher(Path, 'path', 10)
        # Store the path here
        self._path = Path() 

        self.timer = self.create_timer(0.1, self.dijkstra)

    def publish_point(self):
        if self.target_x is None or self.target_y is None:
            return
        pointStamped = PointStamped()
        pointStamped.header.stamp = self.get_clock().now().to_msg()
        pointStamped.header.frame_id = 'odom'
        pointStamped.point.x = self.target_x
        pointStamped.point.y = self.target_y
        
        self._point_pub.publish(pointStamped) 

    def goal_cb(self, msg:PointStamped):
        stamp = rclpy.time.Time()

        transform_base2odom = self.tf2Buffer.lookup_transform(msg.header.frame_id, 'base_link', stamp)
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.header.stamp = stamp.to_msg()
        target_pose.pose.position.x = msg.point.x
        target_pose.pose.position.y = msg.point.y
        target_pose.pose.orientation.w = 1.0
        target_pose.pose.orientation.x = target_pose.pose.orientation.y = target_pose.pose.orientation.z = 0.0
        target_pose = do_transform_pose_stamped(target_pose, transform_base2odom)
        self.goal_t = stamp
        self.target_x = target_pose.pose.position.x
        self.target_y = target_pose.pose.position.y
        self.publish_point()
        self.get_logger().info('Received goal: x: %f, y: %f' % (self.target_x, self.target_y))
        
    def map_cb(self, msg):
        if len(self.map.data) > 0:
            return
        self.map = msg
        

    def dijkstra(self):
        if len(self.map.data) == 0 or self.target_x is None or self.target_y is None:
            return None
        
        self.get_logger().info("Dijkstra")
        
        # Compute robot's position
        child_frame = 'base_link'
        parent_frame = 'odom'
        stamp = rclpy.time.Time()
        
        if self.tfBuffer.can_transform(parent_frame, child_frame, stamp) == 0:
            return
            
        transform = self.tfBuffer.lookup_transform(parent_frame, child_frame, stamp)
        self.origin_x = transform.transform.translation.x
        self.origin_y = transform.transform.translation.y

        map_origin_x, map_origin_y = self.from_coordinates_to_grid_index(self.origin_x, self.origin_y)
        map_target_x, map_target_y = self.from_coordinates_to_grid_index(self.target_x, self.target_y)
        self.get_logger().info("Target: %d %d" % (map_target_x, map_target_y))
                                
        matrix_map = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))
        
        # Good filter but conflicts with the exploration phase
        if matrix_map[map_target_x, map_target_y] == GRID_STATES.OCCUPIED:
            self.get_logger().info("Destination is an obstacle")
            return None
        elif matrix_map[map_target_x, map_target_y] == GRID_STATES.BORDER:
            self.get_logger().info("Destination is a border")
        
        # Create an empty buffer grid
        obstacle_buffer = np.zeros_like(matrix_map)
        threshold_nb_cells = self.from_meters_to_square_nb(self.threshold_distance)
        self.get_logger().info("Threshold: %d" % threshold_nb_cells)
        
        # Iterate over each cell in the occupancy grid
        for i in range(matrix_map.shape[0]):
            for j in range(matrix_map.shape[1]):
                # Check if the current cell is an obstacle
                if matrix_map[i, j] == GRID_STATES.OCCUPIED:
                    # Iterate over neighboring cells within the threshold distance
                    for di in range(-threshold_nb_cells, threshold_nb_cells + 1):
                        for dj in range(-threshold_nb_cells, threshold_nb_cells + 1):
                            # Calculate the indices of the neighboring cell
                            ni, nj = i + di, j + dj
                            # Check if the neighboring cell is within the grid bounds
                            if 0 <= ni < matrix_map.shape[0] and 0 <= nj < matrix_map.shape[1]:
                                # Check if the neighboring cell is not an obstacle
                                if matrix_map[ni, nj] == GRID_STATES.EMPTY:
                                    # Update the buffer grid to mark the neighboring cell as part of the buffer
                                    obstacle_buffer[ni, nj] = 1

        # Update the occupancy grid: set points within the buffer to obstacles
        matrix_map[obstacle_buffer.astype(bool)] = 100
        
        queue = []
        visited = []
        distance = []
        links = []
        for i in range(len(matrix_map)):
            distance.append([math.inf for x in range(len(matrix_map[0]))])
            visited.append([False for x in range(len(matrix_map[0]))])
            links.append([None for x in range(len(matrix_map[0]))])
        distance[map_origin_x][map_origin_y] = 0
        queue.append([map_origin_x, map_origin_y])
        
        while len(queue) > 0:
            [x, y] = queue.pop(0)
            visited[x][y] = True
            if x == map_target_x and y == map_target_y:
                self.completed = True
                break
            for i in range(-1, 2):
                for j in range(-1, 2):
                    if i == 0 and j == 0:
                        continue
                    new_x = x + i
                    new_y = y + j
                    if  new_x < 0 or new_x >= len(matrix_map) or new_y < 0 or new_y >= len(matrix_map[0]):
                        continue
                    if (visited[new_x][new_y] or matrix_map[new_x][new_y] == GRID_STATES.OCCUPIED or (matrix_map[new_x][new_y] == GRID_STATES.BORDER and (new_x != map_target_x or new_y != map_target_y))):
                        continue
                    if abs(i) == abs(j) and distance[new_x][new_y] > distance[x][y] + math.sqrt(2):
                        distance[new_x][new_y] = distance[x][y] + math.sqrt(2)
                        links[new_x][new_y] = [x, y]
                        queue.append([new_x, new_y])
                    elif distance[new_x][new_y] > distance[x][y] + 1:
                        distance[new_x][new_y] = distance[x][y] + 1
                        links[new_x][new_y] = [x, y]
                        queue.append([new_x, new_y])
                        
        if not self.completed:
            self.get_logger().info("No path found")
            return None
        
        self.completed = False    
        x, y = map_target_x, map_target_y
        current_direction = [0, 0]
        while links[x][y] is not None:
            if current_direction == [x-links[x][y][0], y-links[x][y][1]]:
                x, y = links[x][y]
                continue
            current_direction = [x-links[x][y][0], y-links[x][y][1]]
            self.publish_path(self.get_clock().now().to_msg(), x, y, 0.0)
            x, y = links[x][y]

        self.publish_path(self.get_clock().now().to_msg(), map_origin_x, map_origin_y, 0.0)

        pose_array = PoseArray()
        for pose_stamped in self._path.poses:
            # Access attribute of each pose object
            pose_array.poses.append(pose_stamped.pose)
            # Do something with attribute_value
        self.get_logger().info("Publishing poses")
        self._poses_pub.publish(pose_array)
        self.target_x = self.target_y = None
        self._path = Path()

    def publish_path(self, stamp, x, y, yaw):
        """Takes a 2D pose appends it to the path and publishes the whole path.

        Keyword arguments:
        stamp -- timestamp of the transform
        x -- x coordinate of the 2D pose
        y -- y coordinate of the 2D pose
        yaw -- yaw of the 2D pose (in radians)
        """

        self._path.header.stamp = stamp
        self._path.header.frame_id = 'odom'

        pose = PoseStamped()
        pose.header = self._path.header
        world_x, world_y = self.from_grid_index_to_coordinates(x, y)
        pose.pose.position.x = world_x
        pose.pose.position.y = world_y
        pose.pose.position.z = 0.01  # 1 cm up so it will be above ground level

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self._path.poses.append(pose)

        self._path_pub.publish(self._path)

    def from_grid_index_to_coordinates(self, x, y):
        """Transforms a pair of map coordinates to world coordinates.

        Keyword arguments:
        x -- x coordinate in the map
        y -- y coordinate in the map

        Returns:
        x -- x coordinate in the world
        y -- y coordinate in the world
        """

        map_x, map_y = x, y

        y = map_x * self.map.info.resolution + self.map.info.origin.position.x + self.map.info.resolution
        x = map_y * self.map.info.resolution + self.map.info.origin.position.y + self.map.info.resolution

        return x, y

    def from_coordinates_to_grid_index(self, x, y):
        """Transforms a pair of world coordinates to map coordinates.

        Keyword arguments:
        x -- x coordinate in the world
        y -- y coordinate in the world

        Returns:
        x -- x coordinate in the map
        y -- y coordinate in the map
        """

        world_x, world_y = x-self.map.info.origin.position.x, y-self.map.info.origin.position.y

        y = int(world_x / self.map.info.resolution)
        x = int(world_y / self.map.info.resolution)

        return x, y  
    
    def from_meters_to_square_nb(self, meters):
        """Transforms a distance in meters to the number of squares.

        Keyword arguments:
        meters -- distance in meters

        Returns:
        squares -- distance in squares
        """

        return math.ceil(meters / self.map.info.resolution)



def main():
    rclpy.init()
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    
if __name__ == '__main__':
    main()