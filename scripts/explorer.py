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
from SystemFlags import Flags

"""
from enum import Enum

class Flags(Enum):
    AUTONOMOUS = 1
    MANUAL = 2
"""


class Explorer:
    def __init__(self):
        # FLAG: ADD ME TO LAUNCH FILE
        rospy.init_node('turtlebot_explorer', anonymous=True)

        # Send comamnds to navigator
        self.goal_pub = rospy.Publisher('/turtlebot_controller/nav_goal',
                                        Float32MultiArray, queue_size=10)

        # Last known explore flag
        self.explore_mode = Flags.MANUAL

        # Map status
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0, 0]
        self.map_probs = []
        self.occupancy = None

        # Navigator couldn't find any path to these coordinates
        self.banned_coords = []

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

        # Listen to feedback from navigator
        rospy.Subscriber("/turtlebot_controller/explore_fail",
                         Float32MultiArray, self.explore_fail_callback)
        rospy.Subscriber("/turtlebot_controller/explore_success",
                         Float32MultiArray, self.explore_success_callback)


    def explore_callback(self, msg):
        # Update knowledge of robot
        self.robot_state()
        self.explore_mode = Flags(msg.data)

        # Supervisor has control; do nothing
        if self.explore_mode == Flags.MANUAL:
            return

        # Plan where to explore
        elif self.explore_mode == Flags.AUTONOMOUS:
            if self.occupancy and self.has_robot_location:
                self.explore_area()

    def explore_fail_callback(self, msg):
        # Never try to explore this coordinate again, no path
        self.banned_coords.append(msg.data)

        # Force explore replan to new coordinate
        self.explore_callback(self.explore_mode)
        return

    def explore_success_callback(self, msg):
        success = msg.data

        # Request new exploration target if in autonomous control
        if success:
            self.explore_callback(self.explore_mode)
        return

    def explore_area(self):
        # Note on grid shape: vectorized probs stored column-first
        prob_grid = np.reshape(self.map_probs, (self.map_height, self.map_width))
        prob_grid = np.flipud(prob_grid.T)

        # Grab unknown areas
        unseen_inds = np.where(prob_grid == -1)

        # Find furthest unknown
        furthest_dist = None
        furthest_unseens = []
        for i,j in zip(*unseen_inds):
            x = (i * self.map_resolution) + self.map_origin[0]
            y = (j * self.map_resolution) + self.map_origin[1]

            # Avoid coordinates impossible to reach
            if (x,y) in self.banned_coords:
                continue

            dist = self.distance_to_pt((x,y))
            if furthest_dist is None or dist < furthest_dist:
                furthest_dist = dist
                furthest_unseens.append((x,y))

        # Grab the furthest unknown
        explore_coord = None
        while explore_coord is None and len(furthest_unseens) > 0:
            explore_coord = furthest_unseens.pop()

        # Didn't find any unknowns but supervisor commanded exploration:
        # pick a random coordinate
        if explore_coord is None:
            explore_coord = np.random.randint((-self.map_width,-self.map_height),
                                              (self.map_width,self.map_height),
                                              size=2)

        # Publish exploration coordinate to navigator
        # Assume final orientation same as robot's current
        robot_th = tf.transformations.euler_from_quaternion(self.robot_rotation)[2]
        explore_goal = [explore_coord[0], explore_coord[1], robot_th]
        self.goal_pub.publish(explore_goal)
        return


    def distance_to_pt(self, x):
        return np.linalg.norm(np.array(self.robot_translation[:2]) - np.array(x))


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