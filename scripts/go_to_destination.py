#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from final.srv import GoToPose, GoToPoseResponse
from geometry_msgs.msg import Quaternion
import tf.transformations as tft

def handle_go_to_pose(req):
    rospy.loginfo(f"Received goal: x={req.x}, y={req.y}, theta={req.theta}")
    
    # Convert theta (yaw) to quaternion
    quat = tft.quaternion_from_euler(0, 0, req.theta)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = req.x
    goal.target_pose.pose.position.y = req.y
    goal.target_pose.pose.orientation = Quaternion(*quat)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base!")

    client.send_goal(goal)
    rospy.loginfo("Goal sent. Waiting for result...")

    client.wait_for_result()
    result = client.get_result()

    if result:
        return GoToPoseResponse(True, "Goal reached successfully.")
    else:
        return GoToPoseResponse(False, "Failed to reach goal.")

def go_to_pose_server():
    rospy.init_node('go_to_pose_server')
    service = rospy.Service('go_to_pose', GoToPose, handle_go_to_pose)
    rospy.loginfo("Service [go_to_pose] ready to receive goals.")
    rospy.spin()

if __name__ == "__main__":
    go_to_pose_server()

