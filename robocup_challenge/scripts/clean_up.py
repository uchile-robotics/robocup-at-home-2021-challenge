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
import tf
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist

import smach_ros

from utils_hb import Move
import utils

import look_object

class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "aborted"])
    def execute(self,userdata):
        utils.move_arm_init()
        return 'succeeded'

class PanHead(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"])
    def execute(self,userdata):
        utils.move_head_tilt(-0.8)
        rospy.sleep(1.)
        utils.move_base_vel(0 , 0, 60)
        rospy.sleep(3.)
        utils.move_base_vel(0 , 0, -90)
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
                m = Move()
                m.set_pose(0.8, 0.9, 90)
                #m.get_pose()
                m.go()
            except:
                rospy.logerr('fail to move')
                sys.exit()

        if self.place == 'DROP':
            try:
                m = Move()
                m.set_pose(1.8, -0.1, -90)
                #m.get_pose()
                m.go()
            except:
                rospy.logerr('fail to move')
                sys.exit()

        print(start-rospy.get_time())

        return 'succeeded'

def getInstance():

    """
    parameters of the test:
        *Places of interaction:
            -PICKUP = 0.8 , 0.9 , 90
            -DROP = 1.8, -0.1, -90
    """

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])

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
                'succeeded': 'GO_TO_DROP', 
                'failed': 'GO_TO_DROP'             
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