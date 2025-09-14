#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
import sys

def mock_perception_publisher():
    """
    This node simulates a perception system by publishing a single Pose message
    to the /box_pose topic and then shutting down.
    """
    # Initialize the ROS node
    rospy.init_node('mock_perception_publisher', anonymous=True)

    # Create a publisher for the /box_pose topic
    pub = rospy.Publisher('/box_pose', Pose, queue_size=10)

    # Allow some time for the subscriber to connect
    rospy.sleep(1.0)

    # Define the pose to be published
    # This pose should be a location on the table in the Gazebo world
    mock_pose = Pose(
        position=Point(x=0.6, y=-0.2, z=-0.129),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )

    try:
        # Publish the message once
        rospy.loginfo("Publishing mock box pose...")
        pub.publish(mock_pose)
        rospy.loginfo(f"Published pose: (x={mock_pose.position.x}, y={mock_pose.position.y}, z={mock_pose.position.z})")

        # Give it a moment to ensure the message is sent before shutting down
        rospy.sleep(1.0)

    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Mock perception publisher shutting down.")
        # No need to explicitly shutdown, rospy handles it on script exit

if __name__ == '__main__':
    try:
        mock_perception_publisher()
    except rospy.ROSInterruptException:
        print("ROS Interrupt Exception caught. Shutting down.")
        sys.exit(0)
