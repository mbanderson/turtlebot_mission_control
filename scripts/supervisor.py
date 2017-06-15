#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String, UInt8
from geometry_msgs.msg import PoseStamped
import tf
import numpy as np
from std_msgs.msg import Bool

def pose_to_xyth(pose):
    th = tf.transformations.euler_from_quaternion((pose.orientation.x,
                                                   pose.orientation.y,
                                                   pose.orientation.z,
                                                   pose.orientation.w))[2]
    return [pose.position.x, pose.position.y, th]

class MissionStates:
        INIT, EXPLORE, EXECUTE_MISSION, END_OF_MISSION = range(4)

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

        self.pub = rospy.Publisher('/success', Bool, queue_size=10)

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)    # rviz "2D Nav Goal"
        rospy.Subscriber('/mission', Int32MultiArray, self.mission_callback)

        self.goal_pub = rospy.Publisher('turtlebot_controller/nav_goal', Float32MultiArray, queue_size=1)
        # self.vel_control_pub = rospy.Publisher('/turtlebot_control/velocity_goal',
        #                  Float32MultiArray, queue_size=1)
        self.mission_state_pub = rospy.Publisher('/turtlebot_control/mission_state',
                         UInt8, queue_size=1)

        self.MissionStates = MissionStates()
        self.missionState = self.MissionStates.INIT

        self.waypoint_locations = {}    # dictionary that caches the most updated locations of each mission waypoint
        self.waypoint_offset = PoseStamped()
        self.waypoint_offset.pose.position.z = .4    # waypoint is located 40cm in front of the AprilTag, facing it
        quat = tf.transformations.quaternion_from_euler(0., np.pi/2, np.pi/2)
        self.waypoint_offset.pose.orientation.x = quat[0]
        self.waypoint_offset.pose.orientation.y = quat[1]
        self.waypoint_offset.pose.orientation.z = quat[2]
        self.waypoint_offset.pose.orientation.w = quat[3]

        self.DIST_THRESH = 0.5 # Distance to fiducial tag required for turtlebot to progress to next waypoint


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

    def get_robot_state(self):
        """Queries robot state from map."""
        try:
            (self.robot_translation,
             self.robot_rotation) = self.trans_listener.lookupTransform("/map","/base_footprint",rospy.Time(0))
            self.has_robot_location = True
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            self.robot_translation = (0, 0, 0)
            self.robot_rotation = (0, 0, 0, 1)
            self.has_robot_location = False

        theta = tf.transformations.euler_from_quaternion(self.robot_rotation)[2]
        return [self.robot_translation[0], self.robot_translation[1] , theta]

    def publish_mission_state(self, missionState):
        self.mission_state_pub.publish(missionState)
        self.missionState = missionState

    def run(self):
        rate = rospy.Rate(10) # 1 Hz, change this to whatever you like
        while not rospy.is_shutdown():
            self.update_waypoints()

            # STATE MACHINE
            if self.state == 'INIT':
                self.publish_mission_state(self.MissionStates.INIT)
                self.state = 'EXPLORE'

            if self.state == 'EXPLORE':
                self.publish_mission_state(self.MissionStates.EXPLORE)
                if self.click_goal.data:
                   self.goal_pub.publish(self.click_goal) # for manual exploration

                if self.mission and len(self.waypoint_locations) == len(set(self.mission)):
                    self.state = 'EXECUTE_MISSION'
                else:
                    self.state = 'EXPLORE'

            if self.state == 'EXECUTE_MISSION':
                self.publish_mission_state(self.MissionStates.EXECUTE_MISSION)
                goal_id = self.mission[self.goal_counter] # id of goal tag
                self.goal.data = pose_to_xyth(self.waypoint_locations[goal_id].pose)
                self.goal_pub.publish(self.goal)

                currentPos = self.get_robot_state() # Extract current position of robot
                goalPos = pose_to_xyth(self.waypoint_locations[goal_id].pose)
                distToGoal = np.linalg.norm(np.asarray(goalPos[0:2]) - np.asarray(currentPos[0:2]))

                # Check if we're close enough to goal
                if distToGoal < self.DIST_THRESH:
                    self.goal_counter += 1
                    # If we've completed the mission
                    if self.goal_counter > (len(self.mission)-1):
                        # Publish final mission state update
                        self.publish_mission_state(self.MissionStates.END_OF_MISSION)
                        msg = Bool()
                        msg.data = True
                        self.pub.publish(msg)
                        self.state = "END_OF_MISSION"
                        rospy.signal_shutdown('End of Mission. Shutting down supervisor.')
  
                rospy.logwarn('Current goal is {}'.format(self.goal_counter))
                rospy.logwarn('Distance to goal is {}'.format(distToGoal))

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
