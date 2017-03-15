#!/usr/bin/env python

# FILL ME IN!

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
import tf
import numpy as np
from std_msgs.msg import Float32MultiArray, UInt8
# from std_msgs.msg import uint8


def wrapToPi(a):
    # isinstance(object, classinfo) Return true if the object argument is an instance of the classinfo argument,
    if isinstance(a, list):    # backwards compatibility for lists (distinct from np.array)
        return [(x + np.pi) % (2*np.pi) - np.pi for x in a]
    return (a + np.pi) % (2*np.pi) - np.pi


class Controller:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        # rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.trans_listener = tf.TransformListener()

        self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.x_g = 0.0
        self.y_g = 0.0
        self.th_g = 0.0
        self.Mode = 0.0
        # self.Vel = 0
        self.V = 0
        self.om = 0

        # subscribt to another topic called /turtlebot_control/position_goal
        # to grab "self.callback_Position" of type
        rospy.Subscriber('/turtlebot_controller/position_goal', Float32MultiArray, self.callback_Position)

        # rospy.Subscriber('/turtlebot_control/path_goal', Float32MultiArray, self.callback_path_goal)
 		

        rospy.Subscriber('/turtlebot_control/velocity_goal', Float32MultiArray, self.callback_Velocity)
        rospy.Subscriber('/turtlebot_control/control_mode', UInt8, self.callback_Mode)


    def callback_path_goal(self, data): # it publishes the whole path 
    	self.x_g, self.y_g =  data.poses[0].pose.position.x, data.poses[0].pose.position.y/

    # def callback(self, data):
    #     pose = data.pose[data.name.index("mobile_base")]
    #     twist = data.twist[data.name.index("mobile_base")]
    #     self.x = pose.position.x
    #     self.y = pose.position.y
    #     quaternion = (
    #         pose.orientation.x,
    #         pose.orientation.y,
    #         pose.orientation.z,
    #         pose.orientation.w)
    #     euler = tf.transformations.euler_from_quaternion(quaternion)
    #     self.theta = euler[2]


    def callback_Position(self, data): #Here we handle the logic of the state machine 

        self.x_g = data.data[0] # assuming data from state machine is [x,y,th] 
        self.y_g = data.data[1]
        self.th_g = data.data[2]


    def callback_Velocity(self, data): #Here we handle the logic of the state machine 

        self.V = data.data[0] # assuming data from state machine is [x,y,th] 
        self.om = data.data[1]

    def callback_Mode(self, data): #Here we handle the logic of the state machine 

        self.Mode = data.data # assuming data from state machine is [x,y,th] 

    def get_ctrl_output(self):
        # use self.x self.y and self.theta to compute the right control input here
        # x_g = 1.0
        # y_g = 1.0 
        # th_g = 0.0

        # x_g = self.xg
        # y_g = self.yg 
        # th_g = self.thg

        # x_g = self.Goal[0]
        # y_g = self.Goal[1]
        # th_g = self.Goal[2]


        try:
            # tf knows where the tag is
            (translation,rotation) = self.trans_listener.lookupTransform("/map", "base_footprint", rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.x = translation[0]
            self.y = translation[1]
            self.theta = euler[2]


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # tf doesn't know where the tag is
            translation = (0,0,0)
            rotation = (0,0,0,1)
            # self.has_tag_location = False
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.x = translation[0]
            self.y = translation[1]
            self.theta = euler[2]


        rho = ((self.x_g-self.x)**2+(self.y_g-self.y)**2)**0.5

        alpha_noPi = np.arctan2((self.y_g-self.y), (self.x_g-self.x)) - (self.theta)
   
        alpha = wrapToPi([alpha_noPi])[0] #adding + pi
    
        #delta = alpha + th-th_g 
        delta_noPi = alpha + self.theta-self.th_g
        #wrapeToPi creates an array and we're looking at the first element of it [0]
        delta = wrapToPi([delta_noPi])[0] 

        k1 = 0.5
        k2 = 0.5
        k3 = 0.5

        #Define control inputs (V,om) - without saturation constraints
        if self.Mode == 0:       #'velocity_control'
            rospy.loginfo("Velocity Mode")
            V = self.V
            om = self.om
        else:
            rospy.loginfo("Position Mode")
            V = k1*rho*np.cos(alpha)
            om = k2*alpha + k1* np.sinc(alpha/np.pi)*np.cos(alpha) * (alpha + k3*delta)

#    rho_dot   =-V*np.cos(alpha)
#    alpha_dot = V*sin(alpha)/rho - om
#    delta_dot = V*sin(alpha)/rho


    # Apply saturation limits
        V = np.sin(V)*min(0.5, np.abs(V))
        om = np.sin(om)*min(1, np.abs(om))

        cmd_x_dot = V # forward velocity
        cmd_theta_dot = om
        # end of what you need to modify
        cmd = Twist()
        cmd.linear.x = cmd_x_dot
        cmd.angular.z = cmd_theta_dot


        return cmd

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            ctrl_output = self.get_ctrl_output()
            self.pub.publish(ctrl_output)
            rate.sleep()

if __name__ == '__main__':
    ctrl = Controller()
    ctrl.run()

pass