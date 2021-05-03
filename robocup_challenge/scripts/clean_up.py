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

class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "aborted"])
    def execute(self,userdata):
        print('yay')
        return 'succeeded'

class Move(smach.State):
    def __init__(self, place):
        smach.State.__init__(self, outcomes=["succeeded"])
        self.place = place
        self.navclient = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.navclient.wait_for_server()

    def quaternion_from_euler(self, roll, pitch, yaw):

        # ロール、ピッチ、ヨーの順番で回転
        q = tf.transformations.quaternion_from_euler(roll / 180.0 * math.pi,
                                                    pitch / 180.0 * math.pi,
                                                    yaw / 180.0 * math.pi, 'rxyz')
        return Quaternion(q[0], q[1], q[2], q[3])

    def move_base_goal(self, x, y, theta):

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "map"

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        goal.target_pose.pose.orientation = self.quaternion_from_euler(0, 0, theta)

        self.navclient.send_goal(goal)
        self.navclient.wait_for_result()
        state = self.navclient.get_state()

        return True if state == 3 else False

    def execute(self,userdata):
        
        print('move')
        if self.place == 'PICKUP':
            try:
                print('move')
                self.move_base_goal(1, 0.5, 90)
            except:
                rospy.logerr('fail to move')
                sys.exit()
        print('move')
        return 'succeeded'

def getInstance():

    """
    parameters of the test:
        *Places of interaction:
            -PICKUP = 0.8 , 0.9 , 90
            -DROP
    """

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])

    with sm:

        smach.StateMachine.add('SETUP', Setup(),
            transitions={
                'succeeded': 'GO_TO_PICKUP', 
                'aborted': 'aborted'
            }
        )

        smach.StateMachine.add('GO_TO_PICKUP', Move('PICKUP'),
            transitions={
                'succeeded': 'succeeded'                
            }
        )

        return sm

if __name__ == '__main__':

    rospy.init_node('CLEANUP')

    sm = getInstance()

    outcome = sm.execute() # here is where the test begin