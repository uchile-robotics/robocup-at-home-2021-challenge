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

import utils

class LookTo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], io_keys=['counter'])
    def execute(self,userdata):
        if userdata.counter == 0:
            utils.move_head_tilt(-0.8)
            userdata.counter += 1
        elif userdata.counter == 1:
            utils.move_head_tilt(-0.4)
            userdata.counter += 1
        elif userdata.counter == 2:
            utils.move_head_tilt(-0.8)
            utils.move_base_vel(0 , 0, 60)
            userdata.counter += 1
        elif userdata.counter == 3:
            utils.move_head_tilt(-0.4)
            userdata.counter += 1
        elif userdata.counter == 4:
            utils.move_head_tilt(-0.8)
            utils.move_base_vel(0 , 0, -90)
            userdata.counter += 1
        elif userdata.counter == 5:
            utils.move_head_tilt(-0.4)
            userdata.counter = 0
        return 'succeeded'

class FindObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed", "continue"], input_keys=['counter'])
    def execute(self,userdata):
        print('Looking For Object')
        rospy.sleep(3.)
        if userdata.counter == 0: return 'failed'
        return 'continue'

def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    sm.userdata.counter = 0
    sm.userdata.pos = [0,0,0]

    with sm:

        smach.StateMachine.add('LOOK_TO', LookTo(),
            transitions={
                'succeeded': 'FIND_OBJECT'                
            }
        )

        smach.StateMachine.add('FIND_OBJECT', FindObject(),
            transitions={
                'succeeded': 'succeeded', 
                'failed': 'failed',
                'continue': 'LOOK_TO'           
            }
        )

        return sm

if __name__ == '__main__':

    rospy.init_node('LOOK_OBJECT')

    sm = getInstance()

    outcome = sm.execute() # here is where the test begin