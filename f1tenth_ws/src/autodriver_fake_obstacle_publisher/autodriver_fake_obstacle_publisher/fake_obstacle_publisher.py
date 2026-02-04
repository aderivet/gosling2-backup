"""
Publish from:
    - a config file specifying the number of a type of actor [done]
    - a config file specifying the positions of each actor [done]
    - a path message (of waypoints) for specifying some locations automatically (maybe later)

Todo:
    * copy sizes from /carla/objects
    * subscribe to odometry message and use for distance calculations
    * setup transient local publishing for marker and obstacle array to prevent republishing if static
"""
import random
import time
import yaml
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from geometry_msgs.msg import Pose, Twist, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField
from derived_object_msgs.msg import Object, ObjectArray
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
from tf2_ros import TransformBroadcaster, TransformListener, Buffer, LookupException, ConnectivityException, \
    ExtrapolationException
from tf2_geometry_msgs import do_transform_pose
import struct
import sys


class FakeObstaclePublisher(Node):
    def __init__(self):
        super(FakeObstaclePublisher, self).__init__('fake_obstacle_publisher')

        # Load parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('publish_rate', 0.0)  # Optional overwrite
        self.declare_parameter('frame_id', '')  # Optional overwrite
        self.declare_parameter('roi', [])  # Optional region of interest (x_min, x_max, y_min, y_max, z_min, z_max)
        self.declare_parameter('obstacle_config_type', 'path')  # random, individual, path
        self.declare_parameter('respawn', True)  # Whether to respawn obstacles removed by ROI filtering
        self.declare_parameter('use_random_seed', False)  # Whether to use a random seed
        self.declare_parameter('random_seed', 0)  # Optional random seed for reproducibility
        self.declare_parameter('dropout_percentage', 0.0)  # Percentage of obstacles to drop (simulate false negatives)

        # Get optional parameters or use defaults from the config file
        self.config_file = self.get_parameter('config_file').get_parameter_value().string_value
        if self.config_file:
            # todo: fix this as path based spawning doesn't require a config
            with open(self.config_file, 'r') as file:
                self.config = yaml.safe_load(file)
        else:
            self.get_logger().error("No config file provided. Exiting.")
            self.destroy_node()
            sys.exit(1)

        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.publish_rate = self.publish_rate if self.publish_rate > 0.0 else self.config.get('publish_rate', 10.0)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value or self.config.get('frame_id',
                                                                                                             'map')
        self.roi = self.get_parameter('roi').get_parameter_value().double_array_value or self.config.get(
            'roi', [-500, 500.0] * 3 if self.frame_id == 'map' else [-20.0, 20.0, -20.0, 20.0, 0.0, 5.0])
        self.obstacle_config_type = self.get_parameter('obstacle_config_type').get_parameter_value().string_value
        if self.obstacle_config_type not in ['random', 'individual', 'path']:
            self.get_logger().error("Invalid obstacle_config_type. Must be 'random', 'individual', or 'path'. Exiting.")
            self.destroy_node()
            sys.exit(1)
        self.respawn = self.get_parameter('respawn').get_parameter_value().bool_value
        self.use_random_seed = self.get_parameter('use_random_seed').get_parameter_value().bool_value
        self.random_seed = self.get_parameter('random_seed').get_parameter_value().integer_value
        self.dropout_percentage = self.get_parameter('dropout_percentage').get_parameter_value().double_value
        self.path_topic = self.declare_parameter('path_topic', '/trajectory/path').get_parameter_value().string_value
        self.selection_criteria = self.declare_parameter('selection_criteria',
                                                         'equidistant').get_parameter_value().string_value  # equidistant or random
        self.start_index = self.declare_parameter('start_index', 5).get_parameter_value().integer_value
        self.num_obstacles_on_path = self.declare_parameter('num_items_from_path', -10).get_parameter_value().integer_value  # the number of obstacles

        if self.use_random_seed:
            random.seed(self.random_seed)
            self.get_logger().info(f"Random seed set to: {self.random_seed}")
        else:
            self.get_logger().info("Random seed not set, using true randomness.")

        # Validate dropout percentage
        if not (0.0 <= self.dropout_percentage <= 100.0):
            self.get_logger().error("Dropout percentage must be between 0 and 100. Exiting.")
            self.destroy_node()
            return

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.obstacles_pub = self.create_publisher(MarkerArray, 'fake_obstacles/marker_array', 10)
        self.object_array_pub = self.create_publisher(ObjectArray, 'fake_obstacles/object_array', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'fake_obstacles/pointcloud', 10)
        self.path_pubs = []  # One Path publisher per obstacle

        # Setup subscribers
        self.path_received = False
        self.obstacles = None
        if self.obstacle_config_type == 'path':
            latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.path_sub = self.create_subscription(
                    Path,
                    self.path_topic,
                    self.path_callback,
                    qos_profile=latching_qos,
            )  # to get the desired/reference states of the robot

        # Initialize obstacles and their paths
        self.create_obstacles()
        self.paths = []

        # Timer for publishing
        self.dt = 1 / self.publish_rate
        self.timer = self.create_timer(self.dt, self.publish_obstacles)

        # Log details
        self.get_logger().info(f"Publish rate: {self.publish_rate} Hz, Frame ID: {self.frame_id}")
        self.get_logger().info(f"ROI: {self.roi}, Respawn: {self.respawn}")
        self.get_logger().info(f"Dropout Percentage: {self.dropout_percentage}%")

    def _initialize_publishers(self):
        success = False
        # Create a Path publisher for each obstacle
        if self.obstacles is not None:
            self.paths = [[] for _ in range(len(self.obstacles))]  # List of paths, one per obstacle
            self.get_logger().info(f"Initialized {len(self.obstacles)} obstacles.")
            for i in range(len(self.obstacles)):
                self.path_pubs.append(self.create_publisher(Path, f'fake_obstacles/obstacle_{i}/path', 10))
            success = True

        return success

    def create_obstacles(self):
        obstacles = []
        actor_types = self.config.get('actors', {})
        # for random obstacle generation
        if self.obstacle_config_type == 'random':
            for actor_type, params in actor_types.items():
                num_actors = params.get('count', 0)
                for _ in range(num_actors):
                    obstacle = {
                        'type': actor_type,
                        'position': params.get(
                                'initial_position',
                                [random.uniform(self.roi[0], self.roi[1]),
                                 random.uniform(self.roi[2], self.roi[3]),
                                 random.uniform(self.roi[4], self.roi[5])]),
                        'velocity': params.get('velocity', [0.0, 0.0, 0.0]),
                        'acceleration': params.get('acceleration', [0.0, 0.0, 0.0]),
                        'size': params.get('size', [2.0, 2.0, 10.0]),
                        'orientation': math.radians(params.get('initial_orientation', 0.0)),  # For car-like actors
                        'steering_angle': 0.0,  # For car-like actors
                        'angular_velocity': params.get('angular_velocity', [0.0, 0.0, 0.0]),  # For pedestrians,
                        'history_length': 0
                    }

                    obstacles.append(obstacle)
                    time.sleep(0.1)
                time.sleep(0.1)
        elif self.obstacle_config_type == 'individual':
            for individual, attributes in actor_types.items():
                obstacle = {
                    'type': individual.split("_")[0],
                    'position': attributes.get(
                        'initial_position',
                        [random.uniform(self.roi[0], self.roi[1]),
                         random.uniform(self.roi[2], self.roi[3]),
                         random.uniform(self.roi[4], self.roi[5])]),
                    'velocity': attributes.get('velocity', [0.0, 0.0, 0.0]),
                    'acceleration': attributes.get('acceleration', [0.0, 0.0, 0.0]),
                    'size': attributes.get('size', [2.0, 2.0, 10.0]),
                    'orientation': math.radians(attributes.get('initial_orientation', 0.0)),  # For car-like actors
                    'steering_angle': 0.0,  # For car-like actors
                    'angular_velocity': attributes.get('angular_velocity', [0.0, 0.0, 0.0]),  # For pedestrians,
                    'history_length': 0
                }

                obstacles.append(obstacle)

        elif self.obstacle_config_type == 'path':
            # will be initialized in the callback
            pass

        self.obstacles = obstacles
        self.path_received = True
        return obstacles

    def update_kinematic_obstacle(self, obstacle, dt):
        # Kinematic model for car-like actors
        position = obstacle['position']
        orientation = obstacle['orientation']
        velocity = obstacle['velocity']
        steering_angle = obstacle['steering_angle']

        # Update position and orientation
        v = velocity[0]  # Assume velocity is along the x-axis of the car
        L = 2.5  # Wheelbase
        delta = steering_angle

        # Kinematic bicycle model
        dx = v * math.cos(orientation) * dt
        dy = v * math.sin(orientation) * dt
        dtheta = (v / L) * math.tan(delta) * dt

        obstacle['position'][0] += dx
        obstacle['position'][1] += dy
        obstacle['orientation'] += dtheta
        return obstacle

    def update_pedestrian(self, obstacle, dt):
        # 6-DOF model for pedestrian
        position = obstacle['position']
        velocity = obstacle['velocity']
        acceleration = obstacle['acceleration']
        angular_velocity = obstacle['angular_velocity']

        # Linear motion
        for i in range(3):
            position[i] += velocity[i] * dt
            velocity[i] += acceleration[i] * dt

        # Angular motion
        for i in range(3):
            obstacle['orientation'] += angular_velocity[i] * dt

        return obstacle

    def update_generic(self, obstacle, dt):
        # Simple motion model: position += velocity * dt
        obstacle['position'] = [
            obstacle['position'][i] + obstacle['velocity'][i] * dt
            for i in range(3)
        ]
        return obstacle

    def is_within_roi(self, position):
        x, y, z = position
        return (self.roi[0] <= x <= self.roi[1] and
                self.roi[2] <= y <= self.roi[3] and
                self.roi[4] <= z <= self.roi[5])

    def respawn_obstacle(self, obstacle):
        obstacle['position'] = [
            random.uniform(self.roi[0], self.roi[1]),
            random.uniform(self.roi[2], self.roi[3]),
            random.uniform(self.roi[4], self.roi[5]),
        ]
        # obstacle['velocity'] = [0.0, 0.0, 0.0]  # [random.random() * random.randint(-2, 2), 0.0, 0.0]. todo: refactor
        obstacle['acceleration'] = [obstacle['velocity'][0] / self.dt, 0.0, 0.0]
        obstacle['history_length'] = 0
        return obstacle

    def update_obstacle(self, obstacle, dt):
        obstacle['history_length'] += 1
        if obstacle['type'] in ['car', 'truck', 'bus', 'cyclist']:
            return self.update_kinematic_obstacle(obstacle, dt)
        elif obstacle['type'] == 'pedestrian':
            return self.update_pedestrian(obstacle, dt)
        elif obstacle['type'] == 'generic':
            return self.update_generic(obstacle, dt)
        return obstacle

    def publish_obstacles(self):
        if (self.obstacles is None) or (not self.path_received) or (len(self.paths) == 0):
            # this condition is used to handle the timer firing before the paths are initialized from the subscription callback
            paths_initialized = self._initialize_publishers()
            if not paths_initialized:
                return


        marker_array = MarkerArray()
        object_array = ObjectArray()
        object_array.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")  # self.frame_id

        # Lookup the transform from map to ego_vehicle. todo: don't hardcode this
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "ego_vehicle",
                rclpy.time.Time(),  # self.get_clock().now().to_msg()
                rclpy.duration.Duration(seconds=0.2)
            )
        except tf2_ros.LookupException as e:
            self.get_logger().error(f"TF Lookup Error: {str(e)}")
            return
        except tf2_ros.ConnectivityException as e:
            self.get_logger().error(f"TF Connectivity Error: {str(e)}")
            return
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().error(f"TF Extrapolation Error: {str(e)}")
            return

        points = []
        sizes = []

        new_obstacles = []
        for i, obstacle in enumerate(self.obstacles):
            # Check if the obstacle should be dropped based on dropout percentage
            if random.uniform(0, 100) < self.dropout_percentage:
                continue  # Skip publishing this obstacle

            # Update obstacle position and check ROI
            obstacle = self.update_obstacle(obstacle, self.dt)
            object_within_roi = True
            if not self.is_within_roi(obstacle['position']):
                if self.respawn:
                    obstacle = self.respawn_obstacle(obstacle)
                    object_within_roi = False
                else:
                    continue

            new_obstacles.append(obstacle)

            # Add point for PointCloud
            points.append(obstacle['position'])
            sizes.append(obstacle['size'])

            # Create a Marker for visualization
            marker = Marker()
            marker.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.frame_id)  # frame_id=self.frame_id, map
            marker.ns = 'fake_obstacles'
            marker.id = i
            marker.type = Marker.CUBE if obstacle['type'] in ['car', 'truck', 'bus', 'cyclist'] else Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = obstacle['position']
            marker.pose.orientation.w = math.cos(obstacle['orientation'] / 2.0)
            marker.pose.orientation.z = math.sin(obstacle['orientation'] / 2.0)
            marker.scale.x, marker.scale.y, marker.scale.z = obstacle['size']
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = {
                'car': (0.0, 0.0, 1.0, 1.0),
                'truck': (0.5, 0.5, 0.5, 1.0),
                'bus': (1.0, 1.0, 0.0, 1.0),
                'pedestrian': (1.0, 0.0, 0.0, 1.0),
                'cyclist': (0.0, 1.0, 0.0, 1.0)
            }.get(obstacle['type'], (1.0, 1.0, 1.0, 1.0))
            # marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()  # self.dt

            # Transform the marker pose to map frame
            transformed_pose = do_transform_pose(marker.pose, transform)
            # marker.pose = transformed_pose  # Update marker pose
            marker_array.markers.append(marker)

            # Create an Object for state publishing
            obj = Object()
            obj.id = i
            # obj.header.frame_id = "map"
            # obj.header.stamp = self.get_clock().now().to_msg()
            """
            * https://docs.ros.org/en/melodic/api/derived_object_msgs/html/msg/ObjectArray.html 
            * https://docs.ros.org/en/jazzy/p/derived_object_msgs/interfaces/msg/Object.html
            """
            obj.detection_level = Object.OBJECT_TRACKED if obstacle['history_length'] > 0 else Object.OBJECT_DETECTED
            obj.object_classified = True
            obj.classification = {
                'car': Object.CLASSIFICATION_CAR,
                'truck': Object.CLASSIFICATION_TRUCK,
                'bus': Object.CLASSIFICATION_OTHER_VEHICLE,
                'pedestrian': Object.CLASSIFICATION_PEDESTRIAN,
                'cyclist': Object.CLASSIFICATION_MOTORCYCLE,
                'bike': Object.CLASSIFICATION_BIKE
            }.get(obstacle['type'], Object.CLASSIFICATION_UNKNOWN)
            obj.classification_certainty = int(255)

            obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = obstacle['position']
            obj.pose.orientation.w = math.cos(obstacle['orientation'] / 2.0)
            obj.pose.orientation.z = math.sin(obstacle['orientation'] / 2.0)

            obj.twist.linear.x, obj.twist.linear.y, obj.twist.linear.z = obstacle['velocity']
            obj.twist.angular.x, obj.twist.angular.y, obj.twist.angular.z = obstacle['angular_velocity']

            obj.accel.linear.x, obj.accel.linear.y, obj.accel.linear.z = 0.0, 0.0, 0.0
            obj.accel.angular.x, obj.accel.angular.y, obj.accel.angular.z = 0.0, 0.0, 0.0

            obj.shape.type = SolidPrimitive().BOX if obstacle['type'] in ['car', 'truck', 'bus', 'cyclist'] else SolidPrimitive().SPHERE
            obj.shape.dimensions = obstacle['size'][1:] if obstacle['type'] == 'pedestrian' else obstacle['size']

            # obj.pose = transformed_pose
            # todo: transform pointcloud points

            object_array.objects.append(obj)

            # Update path and publish it
            self.update_and_publish_path(i, obstacle, object_within_roi=object_within_roi)

        self.obstacles = new_obstacles

        # Publish marker array and object array
        self.obstacles_pub.publish(marker_array)
        self.object_array_pub.publish(object_array)

        # Publish point cloud
        self.publish_pointcloud(points, sizes=sizes, min_points=100, max_points=200)

    def publish_pointcloud(self, points, sizes=None, min_points=100, max_points=500):
        '''
        Todo:
            * switch to a gaussian distribution for point generation. '''
        cloud = PointCloud2()
        cloud.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.frame_id)

        if sizes is not None:
            new_points = []
            for i, point in enumerate(points):
                # Unpack center and size
                cx, cy, cz = point
                width, height, depth = sizes[i]

                # Determine bounds of the box
                x_min, x_max = cx - width / 2, cx + width / 2
                y_min, y_max = cy - height / 2, cy + height / 2
                z_min, z_max = cz - depth / 2, cz + depth / 2

                num_points = random.randint(min_points, max_points)  # generate num_points random points
                ''' Pythons default random seed is the current system time. 
                However, since this loop runs very fast, the time does not change much within calls which leads to the 
                same numbers being generated multiple times. 
                To fix this we can use random.SystemRandom() instead of random 
                which uses the OS random number generator if available. 
                Another option is to set the seed in each loop with the count or nanosecond timestamp. '''
                # points_ = [
                #     (
                #         random.uniform(int(x_min), int(x_max)),
                #         random.uniform(int(y_min), int(y_max)),
                #         random.uniform(int(z_min), int(z_max))
                #     )
                #     for _ in range(num_points)
                # ]

                # Method 1: using the systems randomizer
                points_ = [
                    (
                        random.SystemRandom().uniform(int(x_min), int(x_max)),
                        random.SystemRandom().uniform(int(y_min), int(y_max)),
                        random.SystemRandom().uniform(int(z_min), int(z_max))
                    )
                    for _ in range(num_points)
                ]
                new_points.extend(points_)
                # Method 2 (fix by setting the seed)
                # for index in range(num_points):
                #     random.seed(time.time_ns())  # random.seed(index)
                #     points_ = [
                #         random.uniform(int(x_min), int(x_max)),
                #         random.uniform(int(y_min), int(y_max)),
                #         random.uniform(int(z_min), int(z_max))
                #     ]
                #     new_points.append(points_)
                # random.seed(None)

            points = new_points

        cloud.height = 1  # Unordered point cloud
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12  # 3 fields * 4 bytes/field
        cloud.row_step = cloud.point_step * len(points)
        cloud.is_dense = True  # Assume no invalid points

        # Pack points into the data field
        buffer = []
        for p in points:
            buffer.append(struct.pack('fff', *p))
        cloud.data = b''.join(buffer)

        self.pointcloud_pub.publish(cloud)

    def update_and_publish_path(self, obstacle_id, obstacle, object_within_roi=True):
        # Create a new PoseStamped for the current position
        pose_stamped = PoseStamped()
        pose_stamped.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.frame_id)
        pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z = obstacle['position']
        pose_stamped.pose.orientation.w = math.cos(obstacle['orientation'] / 2.0)
        pose_stamped.pose.orientation.z = math.sin(obstacle['orientation'] / 2.0)

        # Append to the path for this obstacle
        if object_within_roi:
            self.paths[obstacle_id].append(pose_stamped)
        else:
            self.paths[obstacle_id] = [pose_stamped]

        # Create a Path message and publish
        path_msg = Path()
        path_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.frame_id)
        path_msg.poses = self.paths[obstacle_id]

        self.path_pubs[obstacle_id].publish(path_msg)


    def path_callback(self, data):
        """
        Todo: add flag to remove duplicates, smooth path, speed, yaw, etc.
        :param data:
        :return:
        """
        path_frame_id = data.header.frame_id
        path_time = data.header.stamp

        frame_id_list = []
        coordinate_list = []
        yaw_list = []

        for pose_msg in data.poses:
            node_time = pose_msg.header.stamp
            node_frame_id = pose_msg.header.frame_id
            frame_id_list.append(node_frame_id)

            node_position = pose_msg.pose.position
            node_x = node_position.x
            node_y = node_position.y
            node_z = node_position.z
            coordinate_list.append([node_x, node_y, 0.3])

            node_orientation_quaternion = pose_msg.pose.orientation
            node_qx = node_orientation_quaternion.x
            node_qy = node_orientation_quaternion.y
            node_qz = node_orientation_quaternion.z
            node_qw = node_orientation_quaternion.w

            _, _, node_yaw = tf_transformations.euler_from_quaternion([node_qx, node_qy, node_qz, node_qw])
            yaw_list.append(node_yaw)

        # create the obstacles at equidistant points along the path
        obstacles = []
        actor_types = ['car', 'pedestrian', 'cyclist']
        if self.num_obstacles_on_path == 0:
            self.num_obstacles_on_path = int(len(coordinate_list) * (10 / 100))  # select ten percent

        if self.num_obstacles_on_path < 0:
            self.num_obstacles_on_path = int(len(coordinate_list) * abs(self.num_obstacles_on_path / 100))

        if self.selection_criteria == 'equidistant':
            skip = len(coordinate_list) // self.num_obstacles_on_path
            indices = range(self.start_index, len(coordinate_list), skip)
        else:
            indices = random.sample(range(self.start_index, len(coordinate_list)), self.num_obstacles_on_path)

        # create the obstacles at equidistant or random points along the path
        for i in indices:
            obstacle_position = coordinate_list[i]
            obstacle_orientation = yaw_list[i]
            obstacle = {
                'type': random.choice(actor_types),
                'position': obstacle_position,
                'orientation': obstacle_orientation,
                'velocity': [0.0, 0.0, 0.0],
                'acceleration': [0.0, 0.0, 0.0],
                'size': [2.0, 2.0, 10.0],
                'steering_angle': 0.0,  # For car-like actors
                'angular_velocity': [0.0, 0.0, 0.0],  # For pedestrians,
                'history_length': 0
            }
            obstacles.append(obstacle)

        self.path_received = True
        self.obstacles = obstacles
        return obstacles


def main(args=None):
    rclpy.init(args=args)
    node = FakeObstaclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
