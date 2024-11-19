#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def send_joint_trajectory():
    # Initialize ROS node
    rospy.init_node('send_joint_trajectory', anonymous=True)
    
    # Publisher to the joint trajectory topic
    joint_pub = rospy.Publisher('/robot/joint_trajectory_controller/command', JointTrajectory, queue_size=10)
    
    # Wait for the publisher to connect
    rospy.sleep(1)
    
    # Create a JointTrajectory message
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = ["joint1", "joint2", "joint3"]  # Replace with your robot's joint names
    
    # Create a JointTrajectoryPoint
    point = JointTrajectoryPoint()
    point.positions = [1.0, 0.5, -0.5]  # Target joint positions
    point.velocities = [0.0, 0.0, 0.0]  # Optional: Set joint velocities
    point.time_from_start = rospy.Duration(2.0)  # Time to reach the positions
    
    # Add the point to the trajectory
    trajectory_msg.points.append(point)
    
    # Publish the trajectory
    rospy.loginfo("Publishing joint trajectory...")
    joint_pub.publish(trajectory_msg)
    
    # Keep the node alive for a while
    rospy.sleep(3)

if __name__ == '__main__':
    try:
        send_joint_trajectory()
    except rospy.ROSInterruptException:
        pass
