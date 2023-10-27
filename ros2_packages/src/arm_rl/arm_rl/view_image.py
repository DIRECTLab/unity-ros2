import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

class ImageViewer(Node):
  def __init__(self):
    super().__init__('image_viewer')
    self.bridge = CvBridge()
    self.image_sub = self.create_subscription(CompressedImage, '/camera/image_compressed', self.show_image, 10)
  
  def show_image(self, msg: CompressedImage):
    img = self.bridge.compressed_imgmsg_to_cv2(msg)
    cv2.imshow('Window', img)
    cv2.waitKey(1)

def main(args=None):
  rclpy.init()
  
  sub = ImageViewer()
  
  rclpy.spin(sub)
  
  sub.destroy_node()
  
  rclpy.shutdown()
  
if __name__ == "__main__":
  main()