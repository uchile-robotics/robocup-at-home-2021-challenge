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

import utils_hb

import detection

class LookTo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], io_keys=['counter'])
    def execute(self,userdata):
        if userdata.counter == 0:
            utils_hb.move_head_tilt(-0.8)
            userdata.counter += 1
        elif userdata.counter == 1:
            utils_hb.move_head_tilt(-0.4)
            userdata.counter += 1
        elif userdata.counter == 2:
            utils_hb.move_head_tilt(-0.8)
            utils_hb.move_base_vel(0 , 0, 60)
            userdata.counter += 1
        elif userdata.counter == 3:
            utils_hb.move_head_tilt(-0.4)
            userdata.counter += 1
        elif userdata.counter == 4:
            utils_hb.move_head_tilt(-0.8)
            utils_hb.move_base_vel(0 , 0, -90)
            userdata.counter += 1
        elif userdata.counter == 5:
            utils_hb.move_head_tilt(-0.4)
            userdata.counter = 0
        return 'succeeded'

class FindObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed", "continue"], input_keys=['counter'])
    def execute(self,userdata):
        print('Looking For Object')
        model = detection.RGBD()
        objects = model.detect()
        print(objects)

        return 'succeeded'

class GetCloseObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], input_keys=['pos'])

    def execute(self,userdata):
        print('Getting Close to Object')
        try:
            # se tiene que usar la pose obtenida por vision
            grab_x = userdata.pos[0] 
            grab_y = userdata.pos[1] - 0.1
            m = utils_hb.Move()
            m.set_pose(grab_x, grab_y, 90)
            #m.get_pose()
            m.go()
            utils_hb.move_head_tilt(-0.8)
            rospy.sleep(7.)
        except:
            rospy.logerr('fail to move')
            sys.exit()
        return 'succeeded'

def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    sm.userdata.counter = 0
    sm.userdata.pos = [1.5, 1.2, 90]

    with sm:

        smach.StateMachine.add('LOOK_TO', LookTo(),
            transitions={
                'succeeded': 'FIND_OBJECT'                
            }
        )

        smach.StateMachine.add('FIND_OBJECT', FindObject(),
            transitions={
                'succeeded': 'GET_CLOSE', 
                'failed': 'failed',
                'continue': 'LOOK_TO'           
            }
        )

        smach.StateMachine.add('GET_CLOSE', GetCloseObject(),
            transitions={
                'succeeded': 'succeeded'                
            }
        )

        return sm

if __name__ == '__main__':

    rospy.init_node('LOOK_OBJECT')

    sm = getInstance()

    outcome = sm.execute() # here is where the test begin