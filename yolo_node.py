import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8m.pt').to('cpu')  
        self.subscription = self.create_subscription(
            Image,
            '/rgb',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLO-Inferenz mit konfigurierbarem Threshold
        results = self.model.predict(frame, conf=0.6)[0]  # ← hier Threshold setzen

        for box in results.boxes:
            if box.conf[0] < 0.6:
                continue  # ← zusätzliche Sicherheitsprüfung, optional

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            label = f"{self.model.names[cls]} {conf:.2f}"

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.imshow("YOLO Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    print("Starting YOLO Node")
    node = YoloNode()
    print("YOLO Node started")
    rclpy.spin(node)

    node.destroy_node()
    # Clean up OpenCV windows
    rclpy.shutdown()

if __name__ == '__main__':
    main()
