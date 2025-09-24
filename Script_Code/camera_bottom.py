import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist  # STM32'ye veri göndermek için
import numpy as np
import cv2
from cv_bridge import CvBridge

class BottomImageProcessor(Node):
    def __init__(self):
        super().__init__('bottom_image_processor')
        self.subscription = self.create_subscription(
            Image,
            '/simple_drone/bottom/image_raw',
            self.image_callback,
            10)
        
        # STM32'ye veri göndermek için publisher tanımlama
        self.center_pub = self.create_publisher(Twist, '/center_coordinates', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Mesajdaki görüntü verisini ayrıştır ve OpenCV formatına çevir
            image_data = np.array(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

            # Görüntüyü gri tonlara çevir ve ikili görüntü oluştur
            gray = cv2.cvtColor(image_data, cv2.COLOR_RGB2GRAY)
            _, binary = cv2.threshold(gray, 130, 180, cv2.THRESH_BINARY)

            # Konturları bul
            contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 1000:
                    x, y, w, h = cv2.boundingRect(cnt)
                    center_x, center_y = x + w // 2, y + h // 2
                    cv2.circle(image_data, (center_x, center_y), 5, (0, 0, 255), -1)

                    # Köşe noktalarını al ve çizgi çiz
                    top_left = (x, y)
                    top_right = (x + w, y)
                    bottom_left = (x, y + h)
                    bottom_right = (x + w, y + h)
                    
                    # Köşeler arasında çizgiler oluştur
                    cv2.line(image_data, top_left, top_right, (255, 0, 0), 2)
                    cv2.line(image_data, top_right, bottom_right, (255, 0, 0), 2)
                    cv2.line(image_data, bottom_right, bottom_left, (255, 0, 0), 2)
                    cv2.line(image_data, bottom_left, top_left, (255, 0, 0), 2)

                    # Center coordinates verisini STM32'ye gönder
                    center_msg = Twist()
                    center_msg.linear.x = float(center_x)
                    center_msg.linear.y = float(center_y)
                    self.center_pub.publish(center_msg)

                    print(f"Center coordinates sent: {center_x}, {center_y}")

            cv2.imshow("Bottom Camera View with Husky Detection", image_data)
            cv2.waitKey(10)
        except Exception as e:
            self.get_logger().error(f"Görüntü işleme başarısız: {e}")

def main(args=None):
    rclpy.init(args=args)
    bottom_image_processor = BottomImageProcessor()
    rclpy.spin(bottom_image_processor)
    bottom_image_processor.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2

class BottomImageProcessor(Node):
    def __init__(self):
        super().__init__('bottom_image_processor')
        self.subscription = self.create_subscription(
            Image,
            '/simple_drone/bottom/image_raw',
            self.image_callback,
            10)

    def image_callback(self, msg):
        # Mesajdaki görüntü verisini ayrıştır ve OpenCV formatına çevir
        try:
            # Boyutları ve veri türünü kullanarak NumPy dizisine dönüştür
            image_data = np.array(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

            # Görüntüyü gri tonlara çevir ve ikili görüntü oluştur
            gray = cv2.cvtColor(image_data, cv2.COLOR_RGB2GRAY)
            _, binary = cv2.threshold(gray, 130, 170, cv2.THRESH_BINARY)

            # Konturları bul
            contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Aracın üst kısmını belirlemek için dikdörtgen konturu bul
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 1000:  # Alan değeri ayarlanabilir
                    x, y, w, h = cv2.boundingRect(cnt)
                    
                    # Dikdörtgenin sınırlarını çiz
                    cv2.rectangle(image_data, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    # Aracın merkezi
                    center_x, center_y = x + w // 2, y + h // 2
                    cv2.circle(image_data, (center_x, center_y), 5, (0, 0, 255), -1)  # Merkezi işaretle

                    # Merkez koordinatlarını görüntüleyin
                    print(f"Center coordinates: {center_x}, {center_y}")

            # Görüntüyü ekranda göster
            cv2.imshow("Bottom Camera View with Husky Detection", image_data)
            cv2.waitKey(10)  # Daha büyük değerlerde deneyin
        except Exception as e:
            self.get_logger().error(f"Görüntü işleme başarısız: {e}")

def main(args=None):
    rclpy.init(args=args)
    bottom_image_processor = BottomImageProcessor()
    rclpy.spin(bottom_image_processor)
    bottom_image_processor.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
'''
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class BottomImageSubscriber(Node):
    def __init__(self):
        super().__init__('bottom_image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/simple_drone/bottom/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # ROS mesajını OpenCV formatına çevir
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Görüntüyü ekranda göster
        cv2.imshow("Bottom Camera View", cv_image)
        cv2.waitKey(1)  # Ekranın sürekli yenilenmesi için gerekli

def main(args=None):
    rclpy.init(args=args)
    bottom_image_subscriber = BottomImageSubscriber()
    rclpy.spin(bottom_image_subscriber)
    bottom_image_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
'''
