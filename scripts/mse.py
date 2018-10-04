#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from sklearn.metrics import mean_squared_error as mse
from math import *
import numpy as np
from tf.transformations import *


class Server:
    def __init__(self):
        self.position = None
        self.target = None
        self.MSE_z = None
        self.Z = np.array([])
        self.Z_des = np.array([])
        self.error_pub = rospy.Publisher('/deviations', PoseStamped, queue_size=10)

    def position_callback(self, msg):
        # "Store" message received.
        self.position = msg

        # Compute stuff.
        self.compute_stuff()

    def target_callback(self, msg):
        # "Store" the message received.
        self.target = msg

        # Compute stuff.
        self.compute_stuff()

    def compute_stuff(self):
        if self.position is not None and self.target is not None:
            z = self.position.pose.position.z
            z_des = self.target.position.z
            self.Z = np.append(self.Z, z)
            self.Z_des = np.append(self.Z_des, z_des)
            err_msg = PoseStamped()
            err_msg.header = self.position.header
            err_msg.pose.position.x = abs(self.position.pose.position.x - self.target.position.x )
            err_msg.pose.position.y = abs(self.position.pose.position.y - self.target.position.y )
            err_msg.pose.position.z = abs(self.position.pose.position.z - self.target.position.z )
            self.error_pub.publish(err_msg)
            n_samples = 20
            if len(self.Z)==n_samples and len(self.Z_des)==n_samples:
                self.MSE_z = mse(self.Z, self.Z_des)
                self.Z = np.array([])
                self.Z_des = np.array([])

                #print 'MSE(z, z_des) = '+str(self.MSE_z)
            q = np.array([self.position.pose.orientation.x,
                          self.position.pose.orientation.y,
                          self.position.pose.orientation.z,
                          self.position.pose.orientation.w])
            _,_,yaw = euler_from_quaternion(q)
            print '|x-x_des| = '+str(round(err_msg.pose.position.x,3))
            print '|y-y_des| = '+str(round(err_msg.pose.position.y,3))
            print '|z-z_des| = '+str(round(err_msg.pose.position.z,3))
            print '|yaw-yaw_des| = '+str(round(abs(yaw-self.target.yaw),3))



if __name__ == '__main__':
    rospy.init_node('listener')

    server = Server()

    rospy.Subscriber('/mavros/local_position/pose', PoseStamped , server.position_callback)
    rospy.Subscriber('/mavros/setpoint_raw/target_local', PositionTarget, server.target_callback)

    rospy.spin()