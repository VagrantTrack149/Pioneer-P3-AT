import rclpy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry

HALF_DISTANCE_BETWEEN_WHEELS = 0.165
WHEEL_RADIUS = 0.0975

class P3DXDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left wheel')
        self.__right_motor = self.__robot.getDevice('right wheel')

        self.__imu = self.__robot.getDevice('imu')
        self.__gps = self.__robot.getDevice('gps')
        self.__lidar = self.__robot.getDevice('Sick LMS 291')

        self.__timestep = int(self.__robot.getBasicTimeStep())
        self.__imu.enable(self.__timestep)
        self.__gps.enable(self.__timestep)
        self.__lidar.enable(self.__timestep)

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('p3dxnode')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__odom_publisher = self.__node.create_publisher(Odometry, 'odom', 10)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)

        # Publish Odometry
        gps_values = self.__gps.getValues()
        imu_quat = self.__imu.getQuaternion()

        if gps_values and imu_quat:
            odom = Odometry()
            odom.header.stamp = self.__node.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'

            odom.pose.pose.position.x = gps_values[0]
            odom.pose.pose.position.y = gps_values[1]
            odom.pose.pose.position.z = gps_values[2]

            # Webots quat is [w, x, y, z], ROS quat is (x, y, z, w)
            odom.pose.pose.orientation.x = imu_quat[1]
            odom.pose.pose.orientation.y = imu_quat[2]
            odom.pose.pose.orientation.z = imu_quat[3]
            odom.pose.pose.orientation.w = imu_quat[0]

            self.__odom_publisher.publish(odom)
