#!/usr/bin/env python

import rospy
import numpy as np
import tf
from tf.transformations import quaternion_from_matrix
from gpd_ros.msg import GraspConfigList, GraspConfig
import tf2_geometry_msgs
import copy


class PostGPD():
    def __init__(self):
        self.pub = rospy.Publisher("/jp", tf2_geometry_msgs.PoseStamped, queue_size=1)
        self.p = tf2_geometry_msgs.PoseStamped()
        self.pp_pre = tf2_geometry_msgs.PoseStamped()
        self.sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, self.select_best_grasp)

    def get_best_grasp(self):
        return self.p, self.pp_pre

    def select_best_grasp(self, gpd_message):
        """

        """
        best_grasp = ''
        best_grasp = gpd_message.grasps[0]
        self.p = tf2_geometry_msgs.PoseStamped()
        print(self.p)
        self.p, self.pp_pre = self.transform(best_grasp)
        print(self.p)
        #self.pub.publish(self.p)

    def transform(self, gpd_grasp):
        x_approach,y_approach,z_approach = gpd_grasp.approach.x, gpd_grasp.approach.y, gpd_grasp.approach.z
        x_binormal,y_binormal,z_binormal = gpd_grasp.binormal.x, gpd_grasp.binormal.y, gpd_grasp.binormal.z
        x_axis,y_axis,z_axis = gpd_grasp.axis.x, gpd_grasp.axis.y, gpd_grasp.axis.z
        #R = [[x_approach,y_approach,z_approach], [x_binormal,y_binormal,z_binormal], [x_axis,y_axis,z_axis]]
        #R = [[x_axis,y_axis,z_axis], [x_binormal,y_binormal,z_binormal], [x_approach,y_approach,z_approach]]
        R = [[x_approach,x_binormal,x_axis], 
            [y_approach,y_binormal,y_axis], 
            [z_approach,z_binormal,z_axis]]
        #R = [[x_axis,x_binormal,x_approach], [y_axis,y_binormal,y_approach], [z_axis,z_binormal,z_approach]]
        R = np.asarray(R)
        euler_R = tf.transformations.euler_from_matrix(R)

        pp = tf2_geometry_msgs.PoseStamped()

        pp.header.frame_id = "head_rgbd_sensor_rgb_frame"
        pp.pose.position = copy.deepcopy(gpd_grasp.position)

        delta = 0.082
        pp.pose.position.x = pp.pose.position.x + x_approach*delta
        pp.pose.position.y = pp.pose.position.y + y_approach*delta
        pp.pose.position.z = pp.pose.position.z + z_approach*delta

        ori = tf.transformations.quaternion_from_euler(euler_R[0],euler_R[1],euler_R[2])
        f_rot = tf.transformations.quaternion_from_euler(0, 3*np.pi/2, 0)
        s_rot = tf.transformations.quaternion_from_euler(0, 0, np.pi)
        f_mul = tf.transformations.quaternion_multiply(ori, f_rot)
        s_mul = tf.transformations.quaternion_multiply(f_mul, s_rot) 
        choice = f_mul
        pp.pose.orientation.x = choice[0] 
        pp.pose.orientation.y = choice[1] 
        pp.pose.orientation.z = choice[2] 
        pp.pose.orientation.w = choice[3] 

        pp_pre = copy.deepcopy(pp)
        delta_pre = 0.16
        pp_pre.pose.position.x = pp_pre.pose.position.x + x_approach*delta_pre
        pp_pre.pose.position.y = pp_pre.pose.position.y + y_approach*delta_pre
        pp_pre.pose.position.z = pp_pre.pose.position.z + z_approach*delta_pre


        return pp, pp_pre
        

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