
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

class TopicRelay(Node):
    def __init__(self):
        super().__init__('topic_relay')
        
        # Mevcut IMU aboneliği
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_listener_callback,
            10)
        
        # Mevcut IMU yayını
        self.imu_publisher = self.create_publisher(Imu, '/micro_ros_imu', 10)

        # Mevcut GPS aboneliği
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps/data',
            self.gps_listener_callback,
            10)

        # Mevcut GPS yayını
        self.gps_publisher = self.create_publisher(NavSatFix, '/micro_ros_gps', 10)

        # Yeni GPS aboneliği ("/simple_drone/gps/nav")
        self.new_gps_subscription = self.create_subscription(
            NavSatFix,
            '/simple_drone/gps/nav',  # Yeni GPS topic adresi
            self.new_gps_listener_callback,
            10)

        # Yeni GPS yayını
        self.new_gps_publisher = self.create_publisher(NavSatFix, '/micro_ros_new_gps', 10)

        # Yeni IMU aboneliği ("/simple_drone/imu/out")
        self.new_imu_subscription = self.create_subscription(
            Imu,
            '/simple_drone/imu/out',  # Yeni IMU topic adresi
            self.new_imu_listener_callback,
            10)

        # Yeni IMU yayını
        self.new_imu_publisher = self.create_publisher(Imu, '/micro_ros_new_imu', 10)

    def imu_listener_callback(self, msg):
        # Mevcut IMU verisini iletme
        self.imu_publisher.publish(msg)

    def gps_listener_callback(self, msg):
        # Mevcut GPS verisini iletme
        self.gps_publisher.publish(msg)

    def new_gps_listener_callback(self, msg):
        # Yeni GPS verisini iletme
        self.new_gps_publisher.publish(msg)

    def new_imu_listener_callback(self, msg):
        # Yeni IMU verisini iletme
        self.new_imu_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TopicRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

