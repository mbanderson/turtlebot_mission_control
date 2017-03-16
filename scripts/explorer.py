#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import numpy as np
import matplotlib.pyplot as plt
import tf
from std_msgs.msg import Float32MultiArray
from astar import AStar, DetOccupancyGrid2D, StochOccupancyGrid2D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from enum import Enum

class Flags(Enum):
    AUTONOMOUS = 1
    MANUAL = 2


class Explorer:
    def __init__(self):
        # FLAG: ADD ME TO LAUNCH FILE
        rospy.init_node('turtlebot_explorer', anonymous=True)

        # Send comamnds to navigator
        self.goal_pub = rospy.Publisher('/turtlebot_controller/nav_goal',
                                        Float32MultiArray, queue_size=10)

        # Map status
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0, 0]
        self.map_probs = []
        self.occupancy = None

        # Robot status
        self.has_robot_location = False
        self.robot_translation = (0, 0, 0)
        self.robot_rotation = (0, 0, 0, 1)
        self.trans_listener = tf.TransformListener()


        # FLAG: ADD PUBLISHER IN SUPERVISOR
        rospy.Subscriber('/turtlebot_controller/explore_mode',
                         Float32MultiArray, self.explore_callback)
        # Live map updates
        rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("map_metadata", MapMetaData, self.map_md_callback)



    def explore_callback(self, msg):
        # Supervisor has control; do nothing
        if Flags(msg.data) == Flags.MANUAL:
            return

        # Plan where to explore
        elif Flags(msg.data) == Flags.AUTONOMOUS:
            if self.occupancy and self.has_robot_location:
                self.explore_area()

    def explore_area(self):
        pass


    def map_md_callback(self,msg):
        self.map_width = msg.width
        self.map_height = msg.height
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.position.x,msg.origin.position.y)

    def map_callback(self,msg):
        self.map_probs = msg.data
        if self.map_width>0 and self.map_height>0 and len(self.map_probs)>0:
            self.occupancy = StochOccupancyGrid2D(self.map_resolution,
                                                  self.map_width,
                                                  self.map_height,
                                                  self.map_origin[0],
                                                  self.map_origin[1],
                                                  int(self.plan_resolution / self.map_resolution) * 2,
                                                  self.map_probs)

    def robot_state(self):
        """Queries robot state from map."""
        try:
            (self.robot_translation,
             self.robot_rotation) = self.trans_listener.lookupTransform(
                "/map", "/base_footprint", rospy.Time(0))
            self.has_robot_location = True
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            self.robot_translation = (0, 0, 0)
            self.robot_rotation = (0, 0, 0, 1)
            self.has_robot_location = False
        return