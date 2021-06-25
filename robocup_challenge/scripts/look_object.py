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
import copy

import smach_ros

import utils_hb

import detection

class LookTo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], io_keys=['counter'])
    def execute(self,userdata):
        userdata.counter = 10
        utils_hb.move_arm_init()
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
    def __init__(self, vision_model):
        smach.State.__init__(self, outcomes=["succeeded", "failed", "continue"], io_keys=['counter', 'object_pose', 'selected_object', 'ban_list', 'look_tries'])
        self.vision_model = vision_model
    def execute(self,userdata):
        print('Looking For Object')
        
        objects = self.vision_model.detect()
        print(objects)

        if objects[-1] != []:
            for i in range(len(objects[-1])):
                if not objects[-1][i] in userdata.ban_list: 
                    get_object = objects[-1][i]
                    userdata.selected_object = get_object
                    pose = objects[0][i]
                    print('Going for object: {} with pose {}'.format(get_object, pose))
                    real_pose = utils_hb.make_pose_from_camera(pose)
                    userdata.object_pose = real_pose
                    return 'succeeded'
                print('item is banned')

        print('Object not found')
        userdata.look_tries += 1
        if userdata.look_tries == 5:
            return 'continue'
        return 'failed'
        

class GetCloseObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], input_keys=['object_pose', 'floor'], output_keys=['floor'])

    def execute(self,userdata):
        print('Getting Close to Object')
        try:
            # se tiene que usar la pose obtenida por vision
            map_pose = utils_hb.get_pose_relative_coordinate('map', userdata.object_pose)
            print('AAAAAAAAAAAAAAAAAAAAAA')
            print(map_pose)

            utils_hb.rviz_marker('map', map_pose.pose.position.x, map_pose.pose.position.y, map_pose.pose.position.z)
            grab_x = map_pose.pose.position.x + 0.1
            grab_y = map_pose.pose.position.y - 0.9

            if grab_y + 0.9 > 1.6 and map_pose.pose.position.z < 0.8:
                userdata.floor = True

            print(grab_x)
            print(grab_y)
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

def getInstance(vision_model):

    sm = smach.StateMachine(outcomes=['succeeded', 'failed', 'continue'], input_keys=['object_pose', 'selected_object', 'ban_list', 'floor', 'look_tries'], output_keys=['object_pose', 'selected_object', 'floor', 'look_tries'])
    sm.userdata.counter = 0

    with sm:

        smach.StateMachine.add('LOOK_TO', LookTo(),
            transitions={
                'succeeded': 'FIND_OBJECT'                
            }
        )

        smach.StateMachine.add('FIND_OBJECT', FindObject(vision_model),
            transitions={
                'succeeded': 'GET_CLOSE', 
                'failed': 'failed',
                'continue': 'continue'           
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

    sm = getInstance(None)

    outcome = sm.execute() # here is where the test begin
