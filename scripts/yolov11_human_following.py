#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import torch
import os

ROOT_PATH = os.path.abspath('src/final')

def checkpoint(target):
    return os.path.join(ROOT_PATH, 'checkpoint', target)

class CVControl:
    def __init__(self):
        # Define the velocity topic
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.cmd = Twist()

        # Define the image topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/front_camera/image_raw", Image, self.img_callback)

        # Load YOLOv5s model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True, force_reload=False)
        self.model.eval()

    def img_callback(self, data):
        person_detected = False
        area_biggest_detection = 0
        center_biggest_detection = 0

        # Proportional control variables
        K_rotation = 0.009
        K_velocity = 0.000020
        max_velocity = 0.25

        # Initialize velocities
        v = 0
        w = 0

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        
        frame = cv_image
        (height, width) = frame.shape[:2]
        image_width = width

        # Perform detection with YOLOv5
        results = self.model(frame)

        # Confidence threshold
        conf_threshold = 0.5

        # Process detections
        detections = results.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2, conf, cls]
        if len(detections) > 0:
            for det in detections:
                x1, y1, x2, y2, conf, cls = det
                if int(cls) == 0 and conf > conf_threshold:  # 0 is 'person'
                    person_detected = True
                    startX, startY, endX, endY = int(x1), int(y1), int(x2), int(y2)
                    center_box = int((startX + endX) / 2)
                    area_box = (endX - startX) * (endY - startY)

                    label = f"person: {conf:.2f}"
                    cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)
                    y = startY - 15 if startY - 15 > 15 else startY + 15
                    cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    if area_box > area_biggest_detection:
                        area_biggest_detection = area_box
                        center_biggest_detection = center_box

        # Decide motion
        if person_detected and area_biggest_detection > 10000:
            # Approach the person
            desired_area = 80000  # Reduce distance by setting a bigger desired_area
            v = K_velocity * (desired_area - area_biggest_detection)
            w = K_rotation * (image_width / 2 - center_biggest_detection)

            # Clamp velocity
            v = np.clip(v, -max_velocity, max_velocity)
        else:
            # Rotate to search for person
            v = 0.0
            w = 0.4  # Rotate slowly to find person

        # Send velocity command
        self.send_command(v, w)

        # Show image
        cv2.imshow("Image window", frame)
        cv2.waitKey(3)

    def send_command(self, v, w):
        self.cmd.linear.x = v
        self.cmd.angular.z = w
        self.cmd_pub.publish(self.cmd)

def main():
    rospy.init_node('human_tracking')
    ctrl = CVControl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()