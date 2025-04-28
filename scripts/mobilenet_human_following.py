#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import os

ROOT_PATH = os.path.abspath('src/final')

def checkpoint(target):
    return os.path.join(ROOT_PATH, 'trained_network', target)

# Load the trained network "MobileNetSSD"
global network 
network = cv2.dnn.readNetFromCaffe(checkpoint('MobileNetSSD_deploy.prototxt.txt'), checkpoint('MobileNetSSD_deploy.caffemodel'))

class CVControl:
    def __init__(self):
        # Define the velocity topic
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.cmd = Twist()

        # Define the image topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/front_camera/image_raw", Image, self.img_callback)

        # Initialize last known center of the person
        self.last_center = None

    def img_callback(self, data):
        person_detected = False
        area_biggest_detection = 0
        center_biggest_detection = 0

        # Proportional control variables
        K_rotation = 0.009
        K_velocity = 0.000020
        max_velocity = 0.25
        desired_area = 80000  # Desired area for maintaining distance
        slow_velocity = 0.1   # Slow forward speed when person is far
        search_rotation_speed = 0.5  # Rotation speed when searching for person

        # Initialize velocities
        v = 0
        w = 0

        # Initialize the list of object labels
        CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
                   "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
                   "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
                   "sofa", "train", "tvmonitor"]
        COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        
        # Preprocessing of the frame
        frame = cv_image
        frame_resized = cv2.resize(frame, (300, 300))
        
        # Get height and width of the real frame
        (height, width) = frame.shape[:2]

        # Get a blob for the network
        blob = cv2.dnn.blobFromImage(frame_resized, 0.007843, (300, 300), 127.5)

        # Pass the blob through the network
        network.setInput(blob)
        detections = network.forward()

        # Loop over all detections
        for i in np.arange(0, detections.shape[2]):
            object_type = detections[0,0,i,1]
            confidence = detections[0, 0, i, 2]
            if object_type == 15 and confidence > 0.5:  # Increased threshold for better accuracy
                person_detected = True
                box = detections[0, 0, i, 3:7] * np.array([width, height, width, height])
                (startX, startY, endX, endY) = box.astype("int")

                center_box = int((startX + endX) / 2)
                area_box = (endX - startX) * (endY - startY)
                
                label = "{}: {:.2f}%".format('human', confidence * 100)
                cv2.rectangle(frame, (startX, startY), (endX, endY), [0,0,255], 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, [0,0,255], 2)

                if area_box > area_biggest_detection:
                    area_biggest_detection = area_box
                    center_biggest_detection = center_box

        # Update last known center if a person is detected
        if person_detected:
            self.last_center = center_biggest_detection

        # Decide motion based on detection
        if person_detected:
            # Always center the person
            w = K_rotation * (width / 2 - center_biggest_detection)
            if area_biggest_detection > 10000:
                # Adjust distance if person is close enough
                v = K_velocity * (desired_area - area_biggest_detection)
                v = np.clip(v, -max_velocity, max_velocity)
            else:
                # Move slowly towards the person if they are far
                v = slow_velocity
        else:
            # No person detected, rotate based on last known position
            if self.last_center is not None:
                if self.last_center < width / 2:
                    w = search_rotation_speed  # Turn left
                else:
                    w = -search_rotation_speed  # Turn right
            else:
                w = search_rotation_speed  # Default turn left
            v = 0  # Stop moving forward

        # Send the velocity command
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