#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class ObjectClassifier(Node):
    def __init__(self):
        super().__init__('object_classifier')
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()

        # Publisher for bounding boxes
        self.bbox_pub = self.create_publisher(Detection2DArray, '/objbbox', 10)

        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.listener_callback, 10)

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame, classes=[0, 46, 47], conf=0.5, verbose=False)

        # Initialize the array message
        detection_array = Detection2DArray()
        detection_array.header = msg.header # Sync timestamp and frame_id with the image

        for r in results:
            for box in r.boxes:
                # 1. Create individual detection
                detection = Detection2D()
                
                # 2. Set Bounding Box (YOLO uses [x1, y1, x2, y2], vision_msgs uses center + size)
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                detection.bbox.center.position.x = float((x1 + x2) / 2.0)
                detection.bbox.center.position.y = float((y1 + y2) / 2.0)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)

                # 3. Set Class ID and Confidence
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(int(box.cls[0]))
                hypothesis.hypothesis.score = float(box.conf[0])
                detection.results.append(hypothesis)

                # Add to the array
                detection_array.detections.append(detection)

        # Publish the final list of boxes
        self.bbox_pub.publish(detection_array)
        
        # Optional: Local visualization
        # cv2.imshow("YOLO Live", r.plot())
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectClassifier()
    rclpy.spin(node)
    rclpy.shutdown()



# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy
# from sensor_msgs.msg import Image
# from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
# from cv_bridge import CvBridge
# from ultralytics import YOLO
# import cv2

# class ObjectClassifier(Node):
#     def __init__(self):
#         super().__init__('object_classifier')
        
#         # 1. Initialize YOLOv8 Nano
#         self.model = YOLO('yolov8n.pt') 
#         self.bridge = CvBridge()

#         # 2. Define QoS (Best Effort is better for high-bandwidth camera data)
#         # qos_profile = QoSProfile(
#         #     reliability=ReliabilityPolicy.BEST_EFFORT,
#         #     depth=1
#         # )
#         self.bbox_pub = self.create_publisher(Detection2DArray, '/objbbox', 10)

#         # 3. Create Subscription to the RealSense Topic
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/color/image_raw',
#             self.listener_callback,
#             10)
            
#         self.get_logger().info("YOLOv8 Live Classifier Node Started")

#     def listener_callback(self, msg):
#         try:
#             # Convert ROS Image message to OpenCV format
#             frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
#             # Run YOLOv8 Inference
#             # Using conf=0.5 to reduce false positives
#             results = self.model(frame, classes=[0, 46, 47], conf=0.5, verbose=False)

#             detection_array = Detection2DArray()
#             detection_array.header = msg.header

#             # Process results
#             for r in results:
#                 # Plot the bounding boxes on the frame
#                 annotated_frame = r.plot()
                
#                 # Log detections to terminal
#                 for box in r.boxes:
#                     cls_id = int(box.cls[0])
#                     label = self.model.names[cls_id]
#                     self.get_logger().info(f"Detected: {label} ({box.conf[0]:.2f})")

#             # Display the live feed
#             cv2.imshow("YOLOv8 Real-Time Detection", annotated_frame)
#             cv2.waitKey(1)

#         except Exception as e:
#             self.get_logger().error(f"Failed to process image: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjectClassifier()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

