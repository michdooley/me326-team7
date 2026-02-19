#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np

class BinDetector(Node):
    def __init__(self):
        super().__init__('bin_detector')
        self.bridge = CvBridge()

        # 1. Color ranges (HSV) - kept from your original code
        self.colors = {
            'red': {
                'lower1': np.array([0, 100, 50]), 'upper1': np.array([10, 255, 255]),
                'lower2': np.array([160, 100, 50]), 'upper2': np.array([180, 255, 255])
            },
            'green': {'lower': np.array([35, 100, 50]), 'upper': np.array([85, 255, 255])},
            'blue': {'lower': np.array([100, 150, 50]), 'upper': np.array([140, 255, 255])},
            'yellow': {'lower': np.array([20, 100, 100]), 'upper': np.array([35, 255, 255])}
        }

        # 2. Setup Publisher for /bin_pos
        self.bin_pub = self.create_publisher(Detection2DArray, '/bin_pos', 10)

        # 3. Setup Subscriber for Camera (Matching the YOLO pipeline)
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.listener_callback, qos_profile)
        
        self.get_logger().info("Bin Detector Node started, publishing to /bin_pos")

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Message to hold all detected bins in this frame
            detection_array = Detection2DArray()
            detection_array.header = msg.header

            for color_name, bounds in self.colors.items():
                if color_name == 'red':
                    mask1 = cv2.inRange(hsv_frame, bounds['lower1'], bounds['upper1'])
                    mask2 = cv2.inRange(hsv_frame, bounds['lower2'], bounds['upper2'])
                    mask = cv2.bitwise_or(mask1, mask2)
                else:
                    mask = cv2.inRange(hsv_frame, bounds['lower'], bounds['upper'])
                
                # Morphological opening to remove small noise
                kernel = np.ones((5,5), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for cnt in contours:
                    if cv2.contourArea(cnt) > 800: # Slightly higher threshold for live video
                        x, y, w, h = cv2.boundingRect(cnt)
                        
                        # Create ROS detection message
                        detection = Detection2D()
                        detection.bbox.center.position.x = float(x + w/2.0)
                        detection.bbox.center.position.y = float(y + h/2.0)
                        detection.bbox.size_x = float(w)
                        detection.bbox.size_y = float(h)
                        
                        hypothesis = ObjectHypothesisWithPose()
                        # We use the color name as the class_id string
                        hypothesis.hypothesis.class_id = color_name
                        hypothesis.hypothesis.score = 1.0 # Boolean detection
                        detection.results.append(hypothesis)
                        
                        detection_array.detections.append(detection)
                        self.get_logger().info(f"Found {color_name} bin!")

            self.bin_pub.publish(detection_array)

        except Exception as e:
            self.get_logger().error(f"Bin Detection Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BinDetector()
    rclpy.spin(node)
    rclpy.shutdown()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import cv2
# import numpy as np
# import os

# class BinDetector(Node):
#     def __init__(self):
#         super().__init__('bin_detector')
        
#         # Color ranges (H, S, V)
#         self.colors = {
#             'red': {
#                 'lower1': np.array([0, 100, 50]), 'upper1': np.array([10, 255, 255]),
#                 'lower2': np.array([160, 100, 50]), 'upper2': np.array([180, 255, 255])
#             },
#             'green': {'lower': np.array([35, 100, 50]), 'upper': np.array([85, 255, 255])},
#             'blue': {'lower': np.array([100, 150, 50]), 'upper': np.array([140, 255, 255])},
#             'yellow': {'lower': np.array([20, 100, 100]), 'upper': np.array([35, 255, 255])}
#         }

#         # Path to your test image
#         # Using relative path from where you run the command
#         image_path = 'src/bin_detection/bin_detection/bins.png'
        
#         if os.path.exists(image_path):
#             self.process_static_image(image_path)
#         else:
#             self.get_logger().error(f"Image not found at {image_path}")

#     def process_static_image(self, path):
#         frame = cv2.imread(path)
#         if frame is None:
#             self.get_logger().error("Failed to decode image.")
#             return

#         hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
#         for color_name, bounds in self.colors.items():
#             if color_name == 'red':
#                 # Special case: Combine two red masks
#                 mask1 = cv2.inRange(hsv_frame, bounds['lower1'], bounds['upper1'])
#                 mask2 = cv2.inRange(hsv_frame, bounds['lower2'], bounds['upper2'])
#                 mask = cv2.bitwise_or(mask1, mask2)
#             else:
#                 mask = cv2.inRange(hsv_frame, bounds['lower'], bounds['upper'])
#             contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
#             for cnt in contours:
#                 if cv2.contourArea(cnt) > 500:
#                     x, y, w, h = cv2.boundingRect(cnt)
#                     cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
#                     cv2.putText(frame, color_name, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)
#                     self.get_logger().info(f"SUCCESS: Found {color_name} bin!")

#         cv2.imshow("Static Test Results", frame)
#         self.get_logger().info("Press any key on the image window to close.")
#         cv2.waitKey(0) # Waits forever until you press a key
#         cv2.destroyAllWindows()

# def main(args=None):
#     rclpy.init(args=args)
#     node = BinDetector()
#     # No need for rclpy.spin(node) since we aren't using subscribers yet
#     rclpy.shutdown()