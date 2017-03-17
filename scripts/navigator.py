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

import pdb


class Navigator:
    def __init__(self):
        rospy.init_node('turtlebot_navigator', anonymous=True)

        # Occupancy grid parameters
        self.plan_resolution = 0.25
        self.plan_horizon = 15

        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0, 0]
        self.map_probs = []
        self.occupancy = None

        self.nav_sp = None
        self.short_path_len = 2 # don't go below 2 or else always head to path start
        self.trans_listener = tf.TransformListener()

        # Robot status
        self.has_robot_location = False
        self.robot_translation = (0, 0, 0)
        self.robot_rotation = (0, 0, 0, 1)

        # Mark completed waypoints off existing plan
        self.prev_nav_sp = None
        self.prev_astar = None
        self.wp_complete_thresh = 0.15 # m, mark waypoint completed within thresh
        self.path_abandon_thresh = 0.5 # m, replan if off path

        rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("map_metadata", MapMetaData, self.map_md_callback)
        rospy.Subscriber("/turtlebot_controller/nav_goal", Float32MultiArray, self.nav_sp_callback)

        self.pose_sp_pub = rospy.Publisher('/turtlebot_controller/position_goal', Float32MultiArray, queue_size=10)
        self.nav_path_pub = rospy.Publisher('/turtlebot_controller/path_goal', Path, queue_size=10)

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

    def nav_sp_callback(self,msg):
        # Unpack commanded goal
        self.nav_sp = (msg.data[0],msg.data[1],msg.data[2])

        # Update our knowledge of robot (x,y,th)
        self.robot_state()

        # Goal has changed -> Replan
        if self.nav_sp != self.prev_nav_sp:
            self.send_pose_sp()
            return

        # Still moving toward previous goal
        # Check if existing path still valid
        if self.prev_astar:
            if self.check_existing_path() and len(self.prev_astar.path) > self.short_path_len:
                # Old path is valid

                # Abandon this path if robot deviated too much -> Replan
                if distance_to_line(self.prev_astar.path[0], self.prev_astar.path[1],
                                    self.robot_translation[:2]) > self.path_abandon_thresh:
                    rospy.loginfo("Abandoning plan")
                    self.send_pose_sp()
                    return

                # Otherwise, try to mark waypoints as completed
                # Note: We loop through waypoints in order, so we won't throw out
                # important future waypoints that e.g. curve around nearby wall
                changed = False
                while len(self.prev_astar.path) > self.short_path_len:

                    next_wp = self.prev_astar.path[1]
                    # Close enough to next waypoint, finished
                    if self.finished_waypoint(next_wp):
                        changed = True
                        # Note: assign completed waypoint as start; this keeps
                        # the path display in rviz looking normal
                        self.prev_astar.path[0] = self.prev_astar.path.pop(1)
                    # Next waypoint still valid; don't remove anything more
                    else:
                        break

                # Publish modified previous path
                if changed:
                    rospy.loginfo("Updating previous navigation plan")
                else:
                    rospy.loginfo("Using existing navigation plan")
                wp_x, wp_y, wp_th = self.next_waypoint(self.prev_astar)
                self.publish_path(self.prev_astar, wp_x, wp_y, wp_th)

            # Old path invalid -> Replan
            else:
                self.send_pose_sp()
                return
        # No existing plan -> Replan
        else:
            self.send_pose_sp()
        return


    def check_existing_path(self):
        """Check existing path still free of obstacles."""
        for coords in self.prev_astar.path:
            if not self.prev_astar.is_free(coords):
                return False
        return True


    def finished_waypoint(self, x):
        """Check if robot within threshold of planned waypoint"""
        dist = np.linalg.norm(np.array(self.robot_translation[:2]) - np.array(x))
        return dist < self.wp_complete_thresh


    def robot_state(self):
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
        return


    def send_pose_sp(self):

        if self.occupancy and self.has_robot_location and self.nav_sp:
            state_min = (-int(round(self.plan_horizon)), -int(round(self.plan_horizon)))
            state_max = (int(round(self.plan_horizon)), int(round(self.plan_horizon)))
            # Round initial, goal positions to grid resolution
            x_init = round_pt_to_grid(self.robot_translation[:2], self.plan_resolution)
            x_goal = round_pt_to_grid(self.nav_sp[:2], self.plan_resolution)

            astar = AStar(state_min, state_max, x_init, x_goal, self.occupancy,
                          self.plan_resolution)
            
            # uncomment to add buffering to obstacles
            # bufferRadius = 2
            # astar.bufferOccupancy(bufferRadius)

            rospy.loginfo("Computing new navigation plan")
            if astar.solve():
                # If initial state == goal, path len == 1
                # Handle case where A* solves, but no 2nd element -> don't send msg
                if len(astar.path) < self.short_path_len:
                    rospy.loginfo("Path goal matches current state")

                # Typical use case
                else:
                    wp_x, wp_y, wp_th = self.next_waypoint(astar)
                    self.publish_path(astar, wp_x, wp_y, wp_th)

            else:
                rospy.logwarn("Could not find path")


    def next_waypoint(self, astar):
        """Obtains next path waypoint, accounting for intermediate headings"""
        # Next waypoint calculations
        wp_x = astar.path[self.short_path_len - 1][0]
        wp_y = astar.path[self.short_path_len - 1][1]

        # Far from goal - do intermediate heading calcs
        if len(astar.path) > self.short_path_len:
            dx = wp_x - self.robot_translation[0]
            dy = wp_y - self.robot_translation[1]
            wp_th = np.arctan2(dy, dx)
        # Next point on path is the goal - use final goal pose
        else:
            wp_th = self.nav_sp[2]
        return wp_x, wp_y, wp_th


    def publish_path(self, astar, wp_x, wp_y, wp_th):
        """Publishes single waypoint goal and full path"""
        # Publish next waypoint
        pose_sp = (wp_x, wp_y, wp_th)
        msg = Float32MultiArray()
        msg.data = pose_sp
        self.pose_sp_pub.publish(msg)

        # Publish full path
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for state in astar.path:
            pose_st = PoseStamped()
            pose_st.pose.position.x = state[0]
            pose_st.pose.position.y = state[1]
            pose_st.header.frame_id = 'map'
            path_msg.poses.append(pose_st)
        self.nav_path_pub.publish(path_msg)

        self.prev_nav_sp = self.nav_sp
        self.prev_astar = astar


    def run(self):
        rospy.spin()


def round_pt_to_grid(pt, grid_res):
    """Rounds coordinate point to nearest grid coordinates"""
    steps = 1 / grid_res
    return tuple([round(coord*steps) / steps for coord in pt])

def distance_to_line(start, end, x):
    x1,y1 = start
    x2,y2 = end
    x0,y0 = x

    # Distance from x to line defined by 2 points, start/end
    # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    num = abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1)
    denom = np.sqrt((y2-y1)**2 + (x2-x1)**2)
    dist = num/denom
    return dist


if __name__ == '__main__':
    nav = Navigator()
    nav.run()