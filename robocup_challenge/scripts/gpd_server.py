#!/usr/bin/env python

import rospy
import numpy as np
import tf
from tf.transformations import quaternion_from_matrix
from gpd_ros.msg import GraspConfigList, GraspConfig
import tf2_geometry_msgs
import copy
from sensor_msgs.msg import PointCloud, Image
from geometry_msgs.msg import Point32


class PostGPD():
    def __init__(self):
        self.pub = rospy.Publisher("/jp", tf2_geometry_msgs.PoseStamped, queue_size=1)
        self.pub_grasp = rospy.Publisher("/jp_grasp", PointCloud, queue_size=1)
        self.p = tf2_geometry_msgs.PoseStamped()
        self.pp_pre = tf2_geometry_msgs.PoseStamped()
        self.score = 0
        self.sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, self.select_best_grasp)
        self.flag = False
        self.width = 0

    def get_flag(self):
        return self.flag

    def set_flag(self, value):
        self.flag = value

    def get_best_grasp(self):
        return self.p, self.pp_pre, self.score, self.width

    def select_best_grasp(self, gpd_message):
        """

        """
        if gpd_message.grasps != []:
            best_grasp = copy.deepcopy(gpd_message.grasps[0])
            best_grasp.position.z = 1

            jp_cloud = PointCloud()
            jp_cloud.header.frame_id = 'base_link'
            jp_cloud.header.stamp = rospy.Time.now()

            for grasp in gpd_message.grasps:
                point = Point32()
                point.x = grasp.position.x
                point.y = grasp.position.y
                point.z = grasp.position.z
                jp_cloud.points.append(point)

            self.pub_grasp.publish(jp_cloud)

            for grasp in gpd_message.grasps:
                #print('&&&&&&&&&&&&&&&&&&&&&&&&&')
                #print(grasp)
                if grasp.approach.x >= 0 and grasp.approach.z <= 0:
                    print('changed best grasp')
                    best_grasp = copy.deepcopy(grasp)
                    self.score = best_grasp.score
                    break
            
            self.p = tf2_geometry_msgs.PoseStamped()
            print(self.p)
            self.p, self.pp_pre, self.width = self.transform(best_grasp)
            print(self.p)
            #self.pub.publish(self.p)
            self.flag = True

    def transform(self, gpd_grasp):
        x_approach,y_approach,z_approach = gpd_grasp.approach.x, gpd_grasp.approach.y, gpd_grasp.approach.z
        x_binormal,y_binormal,z_binormal = gpd_grasp.binormal.x, gpd_grasp.binormal.y, gpd_grasp.binormal.z
        x_axis,y_axis,z_axis = gpd_grasp.axis.x, gpd_grasp.axis.y, gpd_grasp.axis.z
        #R = [[x_approach,y_approach,z_approach], [x_binormal,y_binormal,z_binormal], [x_axis,y_axis,z_axis]]
        #R = [[x_axis,y_axis,z_axis], [x_binormal,y_binormal,z_binormal], [x_approach,y_approach,z_approach]]
        #R = [[x_approach,x_binormal,x_axis], 
        #    [y_approach,y_binormal,y_axis], 
        #    [z_approach,z_binormal,z_axis]]
        #R = [[x_axis,x_binormal,x_approach], 
        #    [y_axis,y_binormal,y_approach], 
        #    [z_axis,z_binormal,z_approach]]
        R = [[-x_axis,x_binormal,x_approach], 
            [-y_axis,y_binormal,y_approach], 
            [-z_axis,z_binormal,z_approach]]
        #R = [[1,0,0], 
        #    [0,1,0], 
        #    [0,0,1]]
        #R = [[x_axis,0,0], 
        #    [y_axis,1,0], 
        #    [z_axis,0,1]]
        #R = [[1,0,x_approach], 
        #    [0,1,y_approach], 
        #    [0,0,z_approach]]
        #R = [[x_axis,x_binormal,x_approach], [y_axis,y_binormal,y_approach], [z_axis,z_binormal,z_approach]]
        R = np.asarray(R)
        euler_R = tf.transformations.euler_from_matrix(R)

        pp = tf2_geometry_msgs.PoseStamped()

        pp.header.frame_id = "base_link"
        pp.pose.position = copy.deepcopy(gpd_grasp.position)

        width = gpd_grasp.width

        delta = -0.005
        pp.pose.position.x = pp.pose.position.x + (x_approach)*delta
        pp.pose.position.y = pp.pose.position.y + (y_approach)*delta
        pp.pose.position.z = pp.pose.position.z + (z_approach)*delta

        ori = tf.transformations.quaternion_from_euler(euler_R[0],euler_R[1],euler_R[2])
        f_rot = tf.transformations.quaternion_from_euler(0, 3*np.pi/2, 0)
        s_rot = tf.transformations.quaternion_from_euler(0, 0, np.pi)
        f_mul = tf.transformations.quaternion_multiply(ori, f_rot)
        s_mul = tf.transformations.quaternion_multiply(f_mul, s_rot) 
        choice = ori
        pp.pose.orientation.x = choice[0] 
        pp.pose.orientation.y = choice[1] 
        pp.pose.orientation.z = choice[2] 
        pp.pose.orientation.w = choice[3] 

        pp_pre = copy.deepcopy(pp)
        delta_pre = -0.15
        pp_pre.pose.position.x = pp_pre.pose.position.x + x_approach*delta_pre
        pp_pre.pose.position.y = pp_pre.pose.position.y + y_approach*delta_pre
        pp_pre.pose.position.z = pp_pre.pose.position.z + z_approach*delta_pre

        return copy.deepcopy(pp), copy.deepcopy(pp_pre), width
        

"""
rospy.init_node('get_grasps')
m = PostGPD()
while not rospy.is_shutdown():
    m.select_best_grasp()

# global variable to store grasps
grasps = []


# Callback function to receive grasps.
def callback(msg):
    global grasps
    grasps = msg.grasps


# ==================== MAIN ====================
# Create a ROS node.
rospy.init_node('get_grasps')

# Subscribe to the ROS topic that contains the grasps.
sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback)

# Wait for grasps to arrive.
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    print '.'
    if len(grasps) > 0:
        rospy.loginfo('Received %d grasps.', len(grasps))
        break
    rate.sleep()
"""