#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String
from geometry_msgs.msg import PoseStamped
import tf
import numpy as np
from SystemFlags import Flags

def pose_to_xyth(pose):
    th = tf.transformations.euler_from_quaternion((pose.orientation.x,
                                                   pose.orientation.y,
                                                   pose.orientation.z,
                                                   pose.orientation.w))[2]
    return [pose.position.x, pose.position.y, th]


class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        self.trans_listener = tf.TransformListener()
        self.trans_broad = tf.TransformBroadcaster()
        self.mission = []
        self.goal_counter = 0 # increments to point at next tag in mission
        self.state = 'INIT'
        self.click_goal = Float32MultiArray() # stores goal for manual exploration
        self.goal = Float32MultiArray() # waypoint coordinates in world frame

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)    # rviz "2D Nav Goal"
        rospy.Subscriber('/mission', Int32MultiArray, self.mission_callback)

        self.goal_pub = rospy.Publisher('turtlebot_controller/nav_goal', Float32MultiArray, queue_size=1)
        self.explore_pub = rospy.Publisher('turtlebot_controller/explore_mode', Float32MultiArray, queue_size=1)
        self.explore_msg = Float32MultiArray()

        self.waypoint_locations = {}    # dictionary that caches the most updated locations of each mission waypoint
        self.waypoint_offset = PoseStamped()
        self.waypoint_offset.pose.position.z = .4    # waypoint is located 40cm in front of the AprilTag, facing it
        quat = tf.transformations.quaternion_from_euler(0., np.pi/2, np.pi/2)
        self.waypoint_offset.pose.orientation.x = quat[0]
        self.waypoint_offset.pose.orientation.y = quat[1]
        self.waypoint_offset.pose.orientation.z = quat[2]
        self.waypoint_offset.pose.orientation.w = quat[3]

    def mission_callback(self, msg): # mission callback
        if not self.mission:
            self.mission = msg.data

    def rviz_goal_callback(self, msg):
        # rospy.loginfo(msg)
        self.click_goal.data = pose_to_xyth(msg.pose)    # example usage of the function pose_to_xyth (defined above)

    def update_waypoints(self):
        if not self.mission:
            rospy.logwarn('Supervisor has not received mission command. Is mission_publisher running?')
        for tag_number in self.mission:
            try:
                self.waypoint_offset.header.frame_id = "/tag_{0}".format(tag_number)
                self.waypoint_locations[tag_number] = self.trans_listener.transformPose("/map", self.waypoint_offset)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    def run(self):
        rate = rospy.Rate(10) # 1 Hz, change this to whatever you like
        while not rospy.is_shutdown():
            self.update_waypoints()

            # STATE MACHINE
            if self.state == 'INIT':
                self.state = 'EXPLORE'

                # FLAG
                # Toggle me to command autonomous exploration
                self.explore_msg.data = [int(Flags.AUTONOMOUS)]
                self.explore_pub.publish(self.explore_msg)

            if self.state == 'EXPLORE':
                if self.click_goal.data:
                   self.goal_pub.publish(self.click_goal) # for manual exploration

                if self.mission and len(self.waypoint_locations) == len(set(self.mission)):
                    self.state = 'EXECUTE_MISSION'
                else:
                    self.state = 'EXPLORE'

            if self.state == 'EXECUTE_MISSION':
                goal_id = self.mission[self.goal_counter] # id of goal tag
                self.goal.data = pose_to_xyth(self.waypoint_locations[goal_id].pose)
                self.goal_pub.publish(self.goal)
                # if close enough to goal
                    # if facing goal
                        # increment goal_counter --> will proceed to next goal
                    # if not facing goal
                        # turn until facing goal
                # if not close enough to goal
                    # if turtlebot is not moving
                        # tell controller to go directly to goal
                    # if turtlebot is still moving
                        # pass


                if self.mission and len(self.waypoint_locations) == len(set(self.mission)):
                    self.state = 'EXECUTE_MISSION'
                else:
                    self.state = 'EXPLORE'
            

            rospy.loginfo(self.state)
            rospy.loginfo(len(self.waypoint_locations)-len(set(self.mission)))

            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
