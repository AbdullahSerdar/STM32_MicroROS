import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/simple_drone/front/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # ROS mesajını OpenCV formatına çevir
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Görüntüyü ekranda göster
        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)  # Ekranın sürekli yenilenmesi için gerekli

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
