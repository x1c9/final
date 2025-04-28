#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from ultralytics import YOLO

# Load YOLOv8 Nano model
global model
model = YOLO('yolov8n.pt')  # Pre-trained YOLOv8 Nano weights

class CVControl:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.cmd = Twist()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/front_camera/image_raw", Image, self.img_callback)

    def img_callback(self, data):
        person_detected = 0
        area_biggest_detection = 0
        center_biggest_detection = 0

        # Proportional control variables
        K_rotation = 0.002
        K_velocity = 0.0000045
        max_velocity = 0.25

        # Initialize velocities
        v = 0
        w = 0

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Resize for faster processing
        frame = cv_image
        (h, w) = frame.shape[:2]

        # Run YOLOv8 inference with tracking
        results = model.track(frame, classes=[0], conf=0.3, iou=0.5, persist=True)  # Class 0 is 'person'

        # Process detections
        for result in results:
            boxes = result.boxes
            for box in boxes:
                if box.cls == 0:  # Person class
                    person_detected = 1
                    (x1, y1, x2, y2) = box.xyxy[0].cpu().numpy().astype(int)
                    confidence = box.conf.item()

                    # Calculate center and area
                    center_box = int((x1 + x2) / 2)
                    area_box = (x2 - x1) * (y2 - y1)

                    # Draw bounding box and label
                    label = f"Person: {confidence:.2f}"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), [0, 0, 255], 2)
                    y = y1 - 15 if y1 - 15 > 15 else y1 + 15
                    cv2.putText(frame, label, (x1, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, [0, 0, 255], 2)

                    # Track the largest (closest) person
                    if area_box > area_biggest_detection:
                        area_biggest_detection = area_box
                        center_biggest_detection = center_box

        # Follow the person if detected and close enough
        if person_detected and area_biggest_detection > 10000:
            distance_to_the_person = 150000
            v = K_velocity * (distance_to_the_person - area_biggest_detection)
            w = K_rotation * (w/2 - center_biggest_detection)
            v = np.max([-max_velocity, v])
            v = np.min([max_velocity, v])

        # Send velocity command
        self.send_command(v, w)

        cv2.imshow("Image window", frame)
        cv2.waitKey(3)

    def send_command(self, v, w):
        self.cmd.linear.x = v
        self.cmd.angular.z = w
        self.cmd_pub.publish(self.cmd)

def main():
    ctrl = CVControl()
    rospy.init_node('human_tracking')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()