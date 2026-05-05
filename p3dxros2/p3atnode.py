import math
import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from rclpy.clock import Clock, ClockType


class P3ATDriver:
    def init(self, webots_node, properties):
        self._robot = webots_node.robot
        self._motors = {
            'fl': self._robot.getDevice('front left wheel'),
            'fr': self._robot.getDevice('front right wheel'),
            'bl': self._robot.getDevice('back left wheel'),
            'br': self._robot.getDevice('back right wheel')
        }
        self._gps = self._robot.getDevice('gps')
        self._imu = self._robot.getDevice('imu')
        self._lidar = self._robot.getDevice('Sick LMS 291')
        self._timestep = int(self._robot.getBasicTimeStep())

        self._gps.enable(self._timestep)
        self._imu.enable(self._timestep)
        if self._lidar:
            self._lidar.enable(self._timestep)

        for motor in self._motors.values():
            motor.setPosition(float('inf'))
            motor.setVelocity(0)

        rclpy.init(args=None)
        self._node = rclpy.create_node('p3at_driver')

        # Reloj del sistema (wall clock) — consistente con SLAM Toolbox
        self._clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        self._tf_broadcaster = TransformBroadcaster(self._node)
        self._node.create_subscription(Twist, 'cmd_vel', self._cmd_vel_callback, 1)
        self._odom_publisher = self._node.create_publisher(Odometry, 'odom', 10)
        self._target_twist = Twist()

    def _cmd_vel_callback(self, twist):
        self._target_twist = twist

    def step(self):
        rclpy.spin_once(self._node, timeout_sec=0)
        v_x = self._target_twist.linear.x
        v_theta = self._target_twist.angular.z

        v_l = (v_x - v_theta * 0.2) / 0.11
        v_r = (v_x + v_theta * 0.2) / 0.11

        self._motors['fl'].setVelocity(v_l)
        self._motors['bl'].setVelocity(v_l)
        self._motors['fr'].setVelocity(v_r)
        self._motors['br'].setVelocity(v_r)

        gps_val = self._gps.getValues()
        imu_q = self._imu.getQuaternion()  # Webots R2025a: [x, y, z, w]

        if gps_val and imu_q:
            # Timestamp del sistema real — evita desfase con el /scan de Webots
            now = self._clock.now().to_msg()

            pos_x = gps_val[0]
            pos_y = gps_val[1]
            # Si el robot se mueve en Y en vez del plano XY, cambia a:
            # pos_x = gps_val[0]
            # pos_y = -gps_val[2]

            qx, qy, qz, qw = imu_q[0], imu_q[1], imu_q[2], imu_q[3]
            yaw = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz)
            )
            qz_2d = math.sin(yaw / 2.0)
            qw_2d = math.cos(yaw / 2.0)

            # TF: odom -> base_footprint
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = pos_x
            t.transform.translation.y = pos_y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = qz_2d
            t.transform.rotation.w = qw_2d
            self._tf_broadcaster.sendTransform(t)

            # Odometria
            o = Odometry()
            o.header.stamp = now
            o.header.frame_id = 'odom'
            o.child_frame_id = 'base_footprint'
            o.pose.pose.position.x = pos_x
            o.pose.pose.position.y = pos_y
            o.pose.pose.position.z = 0.0
            o.pose.pose.orientation.x = 0.0
            o.pose.pose.orientation.y = 0.0
            o.pose.pose.orientation.z = qz_2d
            o.pose.pose.orientation.w = qw_2d
            o.twist.twist.linear.x = self._target_twist.linear.x
            o.twist.twist.angular.z = self._target_twist.angular.z
            self._odom_publisher.publish(o)


def main(args=None):
    pass
