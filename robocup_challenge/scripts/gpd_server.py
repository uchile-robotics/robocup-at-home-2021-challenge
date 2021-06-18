#!/usr/bin/env python

import rospy
import numpy as np
import tf
from tf.transformations import quaternion_from_matrix
from gpd_ros.msg import GraspConfigList, GraspConfig
import tf2_geometry_msgs


class PostGPD():
    def __init__(self):
        self.pub = rospy.Publisher("/jp", tf2_geometry_msgs.PoseStamped, queue_size=1)
        self.p = tf2_geometry_msgs.PoseStamped()

    def select_best_grasp(self):
        """

        """
        msg = rospy.wait_for_message("/detect_grasps/clustered_grasps",  GraspConfigList)
        best_grasp = msg.grasps[0]
        self.transform(best_grasp)
        print(self.p)
        self.pub.publish(self.p)

    def transform(self, gpd_grasp):
        x_approach,y_approach,z_approach = gpd_grasp.approach.x, gpd_grasp.approach.y, gpd_grasp.approach.z
        x_binormal,y_binormal,z_binormal = gpd_grasp.binormal.x, gpd_grasp.binormal.y, gpd_grasp.binormal.z
        x_axis,y_axis,z_axis = gpd_grasp.axis.x, gpd_grasp.axis.y, gpd_grasp.axis.z
        R = [[x_approach,y_approach,z_approach], [x_binormal,y_binormal,z_binormal], [x_axis,y_axis,z_axis]]
        R = np.asarray(R)
        euler_R = tf.transformations.euler_from_matrix(R)

        self.p.header.frame_id = "/head_rgbd_sensor_rgb_frame"
        self.p.pose.position = gpd_grasp.position 
        ori = tf.transformations.quaternion_from_euler(euler_R[0],euler_R[1],euler_R[2])
        self.p.pose.orientation.x = ori[0] 
        self.p.pose.orientation.y = ori[1] 
        self.p.pose.orientation.z = ori[2] 
        self.p.pose.orientation.w = ori[3] 
        

 
rospy.init_node('get_grasps')
m = PostGPD()
while not rospy.is_shutdown():
    m.select_best_grasp()
"""
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