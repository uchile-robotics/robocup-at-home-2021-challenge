#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import math
import numpy as np
import smach
import cv2
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf, tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

import smach_ros

import utils_hb

import look_object

import manipulation

import detection

class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "aborted"])
    def execute(self,userdata):
        utils_hb.move_arm_init()
        return 'succeeded'

class PanHead(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"])
    def execute(self,userdata):
        utils_hb.move_head_tilt(-0.8)
        rospy.sleep(1.)
        utils_hb.move_base_vel(0 , 0, 60)
        rospy.sleep(3.)
        utils_hb.move_base_vel(0 , 0, -90)
        rospy.sleep(3.)
        return 'succeeded'

class DropObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"])
    def execute(self,userdata):
        rel_cord = utils_hb.get_relative_coordinate('base_link','hand_palm_link')

        p = tf2_geometry_msgs.PoseStamped()

        p.header.frame_id = "base_link"

        p.pose.position = rel_cord.translation
        p.pose.orientation = rel_cord.rotation

        p.pose.position.x = rel_cord.translation.x + 0.2
        p.pose.position.z = rel_cord.translation.z - 0.2

        utils_hb.arm.set_pose_target(p)
        utils_hb.arm.go()
        utils_hb.move_hand(0.8)

        utils_hb.move_arm_init()

        return 'succeeded'

class MoveSM(smach.State):
    def __init__(self, place):
        smach.State.__init__(self, outcomes=["succeeded"])
        self.place = place

    def execute(self,userdata):

        start = rospy.get_time()
        
        if self.place == 'PICKUP':
            try:
                m = utils_hb.Move()
                m.set_pose(0.8, 0.9, 90)
                #m.get_pose()
                m.go()
            except:
                rospy.logerr('fail to move')
                sys.exit()

        if self.place == 'DROP':
            try:
                m = utils_hb.Move()
                m.set_pose(1.8, -0.1, -90)
                #m.get_pose()
                m.go()
            except:
                rospy.logerr('fail to move')
                sys.exit()

        print(start-rospy.get_time())

        return 'succeeded'

class SetPose(smach.State):
    def __init__(self, vision_model):
        smach.State.__init__(self, outcomes=["succeeded", "failed"], io_keys=['object_pose', 'grab_pose', 'selected_object'])
        self.vision_model = vision_model
    def execute(self,userdata):

        # get updated object pose
        objects = self.vision_model.detect()
        print(objects)

        print('CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC')
        planning_frame = utils_hb.whole_body.get_planning_frame()
        print("============ Reference frame: {}".format(planning_frame))

        if object != []:
            obj_index = objects[-1].index(userdata.selected_object)
            selec_pose_raw = objects[0][obj_index]
            selec_pose = utils_hb.make_pose_from_camera(selec_pose_raw)

            print(type(selec_pose))

            utils_hb.move_arm_neutral()
            # transformar a pose para manip
            userdata.grab_pose = utils_hb.get_pose_relative_coordinate('odom', selec_pose)
            #userdata.grab_pose.pose.position.z = userdata.grab_pose.pose.position.z + 0.09

            print(type(userdata.grab_pose))
            
            utils_hb.rviz_marker('/odom', userdata.grab_pose.pose.position.x, userdata.grab_pose.pose.position.y, userdata.grab_pose.pose.position.z)
            
            #ori = quaternion_from_euler(0, 0, 0)
            
            #userdata.grab_pose.pose.orientation.x = ori[0]
            #userdata.grab_pose.pose.orientation.y = ori[1]
            #userdata.grab_pose.pose.orientation.z = ori[2]
            #userdata.grab_pose.pose.orientation.w = ori[3]
            #userdata.grab_pose.header.stamp = rospy.Time.now()

            print('BBBBBBBBBBBBB')
            print(userdata.grab_pose)

            #rospy.sleep(1)

            #userdata.grab_pose = utils_hb.get_pose_relative_coordinate('odom', userdata.grab_pose)

            

            pose_pub = rospy.Publisher("/nico", PoseStamped, queue_size=5)
            rospy.sleep(1)
            pose_pub.publish(userdata.grab_pose)

            

            
            
            return 'succeeded'

        return 'failed'

def getInstance():

    """
    parameters of the test:
        *Places of interaction:
            -PICKUP = 0.8 , 0.9 , 90
            -DROP = 1.8, -0.1, -90
    """
    # Se crea el modelo 
    print('CARGANDO MODELO')
    vis_model = detection.RGBD()

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])

    sm.userdata.grab_pose = tf2_geometry_msgs.PoseStamped()
    sm.userdata.object_pose = []
    sm.userdata.selected_object = ''

    with sm:

        smach.StateMachine.add('SETUP', Setup(),
            transitions={
                'succeeded': 'GO_TO_PICKUP', 
                'aborted': 'aborted'
            }
        )

        smach.StateMachine.add('GO_TO_PICKUP', MoveSM('PICKUP'),
            transitions={
                'succeeded': 'LOOK_OBJECT'                
            }
        )

        smach.StateMachine.add('LOOK_OBJECT', look_object.getInstance(vis_model),
            transitions={
                'succeeded': 'GET_POSE', 
                'failed': 'GET_POSE'             
            }
        )

        smach.StateMachine.add('GET_POSE', SetPose(vis_model),
            transitions={
                'succeeded': 'GRAB_OBJECT',
                'failed': 'GET_POSE'                
            }
        )

        smach.StateMachine.add('GRAB_OBJECT', manipulation.getInstance(),
            transitions={
                'succeeded': 'GO_TO_DROP'                
            }
        )

        smach.StateMachine.add('GO_TO_DROP', MoveSM('DROP'),
            transitions={
                'succeeded': 'DROP_OBJECT'                
            }
        )

        smach.StateMachine.add('DROP_OBJECT', DropObject(),
            transitions={
                'succeeded': 'succeeded'                
            }
        )

        return sm

if __name__ == '__main__':

    rospy.init_node('CLEANUP')

    sm = getInstance()

    outcome = sm.execute() # here is where the test begin