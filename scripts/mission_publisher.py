#!/usr/bin/env python
"""Publishes a simulated mission specification for testing."""

import rospy
from std_msgs.msg import Int32MultiArray


if __name__ == '__main__':
    mission = Int32MultiArray()
    mission.data = [3,6,1,3,0]  # Mission specification of target IDs
    pub = rospy.Publisher('/mission', Int32MultiArray, queue_size=10)
    rospy.init_node('mission_publisher', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(mission)
        rate.sleep()