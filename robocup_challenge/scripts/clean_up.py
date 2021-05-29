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

import smach_ros

import utils_hb

import look_object

import manipulation

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
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], io_keys=['in_pose'])
    def execute(self,userdata):
        
        utils_hb.move_arm_neutral()

        rel_cord = utils_hb.get_relative_coordinate('base_link','hand_palm_link')

        userdata.in_pose.header.frame_id = "base_link"

        userdata.in_pose.pose.position = rel_cord.translation
        userdata.in_pose.pose.orientation = rel_cord.rotation

        #userdata.in_pose.header.frame_id = "base_link"
        userdata.in_pose.pose.position.x = rel_cord.translation.x + 0.15
        #userdata.in_pose.pose.position.y = 0.103366
        #userdata.in_pose.pose.position.z = rel_cord.translation.z + 0.05
        #userdata.in_pose.pose.orientation.x = -0.70401285
        #userdata.in_pose.pose.orientation.y = -0.0639018
        #userdata.in_pose.pose.orientation.z = -0.70438379
        #userdata.in_pose.pose.orientation.w = 0.06425529
        return 'succeeded'

def getInstance():

    """
    parameters of the test:
        *Places of interaction:
            -PICKUP = 0.8 , 0.9 , 90
            -DROP = 1.8, -0.1, -90
    """

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])

    sm.userdata.in_pose = tf2_geometry_msgs.PoseStamped()

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

        smach.StateMachine.add('LOOK_OBJECT', look_object.getInstance(),
            transitions={
                'succeeded': 'GET_POSE', 
                'failed': 'GET_POSE'             
            }
        )

        smach.StateMachine.add('GET_POSE', SetPose(),
            transitions={
                'succeeded': 'GRAB_OBJECT'                
            }
        )

        smach.StateMachine.add('GRAB_OBJECT', manipulation.getInstance(),
            transitions={
                'succeeded': 'GO_TO_DROP'                
            }
        )

        smach.StateMachine.add('GO_TO_DROP', MoveSM('DROP'),
            transitions={
                'succeeded': 'PAN_HEAD'                
            }
        )

        smach.StateMachine.add('PAN_HEAD', PanHead(),
            transitions={
                'succeeded': 'succeeded'                
            }
        )

        return sm

if __name__ == '__main__':

    rospy.init_node('CLEANUP')

    sm = getInstance()

    outcome = sm.execute() # here is where the test begin