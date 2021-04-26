#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import math
import numpy as np
import smach
import cv2
import octomap_msgs

import smach_ros
from smach_ros import IntrospectionServer

class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "aborted"])
    def execute(self,userdata):
        print('yay')
        return 'succeeded'

def getInstance():
    
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])

    with sm:

        smach.StateMachine.add('WELCOME_SECOND_GUEST', Setup(),
            transitions={
                'succeeded': 'succeeded', 
                'aborted': 'aborted'
            }
        )

        return sm

if __name__ == '__main__':

    print('yay')

    rospy.init_node('CLEANUP')

    sm = getInstance()

    outcome = sm.execute() # here is where the test begin
    print(outcome)
