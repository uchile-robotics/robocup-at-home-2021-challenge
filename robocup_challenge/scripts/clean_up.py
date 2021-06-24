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
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist, WrenchStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker
import copy

import smach_ros

import utils_hb

import look_object

import manipulation

import detection

import gpd_server

class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "aborted"], io_keys=['timer'])
    def execute(self,userdata):
        utils_hb.move_arm_init()
        userdata.timer = rospy.get_time()
        return 'succeeded'

class CheckTime(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["continue", "finish"], io_keys=['timer'])
    def execute(self,userdata):
        actual_time = rospy.get_time()
        delta = actual_time - userdata.timer
        print('It has been {} seconds'.format(delta))
        if delta > 300:
            return 'finish'
        return 'continue'

class ResetData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], io_keys=['object_pose', 'grab_pose', 'selected_object'])
    def execute(self,userdata):
        sm.userdata.grab_pose = tf2_geometry_msgs.PoseStamped()
        sm.userdata.object_pose = []
        sm.userdata.selected_object = ''
        sm.userdata.gpd_tries = 0
        sm.userdata.banned_items = []
        
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
        wrench_raw = rospy.wait_for_message("/hsrb/wrist_wrench/raw",  WrenchStamped)
        torque_offset = 0.4
        init_height = 0.43
        init_joints = [init_height, -2.1, 0.0, 0.4, 0.0, 0]
        utils_hb.arm.set_joint_value_target(init_joints)
        utils_hb.arm.go()
        rospy.sleep(0.1)
        utils_hb.move_hand(1.0)
        utils_hb.arm.set_joint_value_target([init_height+0.05, -2.1, 0.0, 0.4, 0.0, 0])
        utils_hb.arm.go()
        rospy.sleep(0.1)
        utils_hb.arm.set_joint_value_target([init_height, -2.1, 0.0, 0.4, 0.0, 0])
        utils_hb.arm.go()
        rospy.sleep(0.1)
        utils_hb.arm.set_joint_value_target([init_height+0.05, -2.1, 0.0, 0.4, 0.0, 0])
        utils_hb.arm.go()
        rospy.sleep(0.1)
        utils_hb.arm.set_joint_value_target([init_height, -2.1, 0.0, 0.4, 0.0, 0])
        utils_hb.arm.go()
        rospy.sleep(0.1)
        utils_hb.move_arm_neutral()
        utils_hb.move_hand(0.0)


        return 'succeeded'

class MoveSM(smach.State):
    def __init__(self, place):
        smach.State.__init__(self, outcomes=["succeeded"], io_keys=['drop_counter'])
        self.place = place

    def execute(self,userdata):

        start = rospy.get_time()
        
        if self.place == 'PICKUP':
            try:
                m = utils_hb.Move()
                m.set_pose(0.8, 0.5, 90)
                #m.get_pose()
                m.go()
            except:
                rospy.logerr('fail to move')
                sys.exit()

        if self.place == 'DROP':
            drop_poses = [[1.8, 0.0, -90],
                        [1.6, 0.0, -90],
                        [1.4, 0.0, -90]]
            drop_pose = drop_poses[userdata.drop_counter]
            try:
                m = utils_hb.Move()
                m.set_pose(drop_pose[0], drop_pose[1], drop_pose[2])
                #m.get_pose()
                m.go()
            except:
                rospy.logerr('fail to move')
                sys.exit()
            userdata.drop_counter += 1
            if userdata.drop_counter > 2:
                userdata.drop_counter = 0

        print(start-rospy.get_time())

        return 'succeeded'

class GoToGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"], io_keys=['counter', 'object_pose', 'selected_object', 'drop_counter'])
        self.food_items = ["sugar_box","peach","apple","lemon","mustard_bottle","potted_meat_can","gelatin_box","tuna_fish_can","tomato_soup_can","pear","plum","pudding_box","master_chef_can","orange","banana","strawberry","cracker_box"]
        self.kitchen_items = ["pitcher_base","spatula","knife","mug","skillet_lid","skillet","windex_bottle","fork","spoon","bowl","sponge","plate","cleanser_bottle"]
        self.tool_items = ["marker","power_drill","adjustable_wrench","hammer","padlock","scissors","screwdriver","wood_block","clamps"]
        self.shape_items = ["tennis_ball","marbles_a","cup","foam_brick","marbles_b","golf_ball","soccer_ball","chain","marbles_c","dice","racquetball","soft_ball","baseball"]
        self.task_items = ["nine_hole_peg_test","b_lego","colored_wood_blocks","c_lego","a_lego","c_toy_airplane","d_toy_airplane","rubiks_cube","d_lego","a_toy_airplane","e_lego","f_lego","b_toy_airplane"]
        
    
    def execute(self,userdata):
        print('Choosing Goal')

        object_p = copy.deepcopy(userdata.selected_object)

        if object_p in self.food_items:
            _class = 'food_items'
            drop_poses = [[1.8, 0.0, -90],
                    [1.6, 0.0, -90],
                    [1.4, 0.0, -90]]
            drop_pose = drop_poses[userdata.drop_counter]
            userdata.drop_counter += 1
            if userdata.drop_counter > 2:
                userdata.drop_counter = 0
        elif object_p in self.kitchen_items:
            _class = 'kitchen_items'
            drop_pose = [2.4, 0.0, -90]
        elif object_p in self.tool_items:
            _class = 'tool_items'
            drop_pose = [2.4, 0.0, -90]
        elif object_p in self.shape_items:
            _class = 'shape_items'
            drop_pose = [2.4, 0.0, -90]
        elif object_p in self.task_items:
            _class = 'task_items'
            drop_pose = [2.4, 0.0, -90]
        else:
            _class = 'other_items'
            drop_pose = [2.4, 0.0, -90]

        print('Class: {}'.format(_class))

        # move to goal
        try:
            m = utils_hb.Move()
            m.set_pose(drop_pose[0], drop_pose[1], drop_pose[2])
            #m.get_pose()
            m.go()
        except:
            rospy.logerr('fail to move')
            return 'failed'

        return 'succeeded'
        
        

class SetPose(smach.State):
    def __init__(self, vision_model):
        smach.State.__init__(self, outcomes=["succeeded", "failed", 'ban'], io_keys=['object_pose', 'grab_pose', 'selected_object', 'pre_pose', 'width', 'gpd_tries', 'ban_list'])
        self.vision_model = vision_model
    def execute(self,userdata):
        utils_hb.move_arm_init()
        
        # check if mask exists
        obj_mask = self.vision_model.segmentation()
        print('yay')

        if obj_mask != []:
            
            # create GPD receiver
            gpd_receiver = gpd_server.PostGPD()
            print('yay2')

            timeout = 6
            print('waiting')
            while not gpd_receiver.get_flag():
                print('No GPD Answer')
                rospy.sleep(0.1)
                timeout -= 0.1
                if timeout < 0:
                    userdata.gpd_tries += 1
                    if userdata.gpd_tries == 3:
                        print('banning {}'.format(userdata.selected_object))
                        userdata.ban_list.append(userdata.selected_object)
                        return 'ban'
                    return 'failed'
            print('done waiting')

            
            # reset flag
            gpd_receiver.set_flag(False)

            # receive best pose

            best_pose, pre_grasp, score, width = gpd_receiver.get_best_grasp()
            userdata.width = width
            print('###################')
            print('best pose')
            print(best_pose)
            print('score: {}'.format(score))

            # preparar mano
            utils_hb.move_arm_neutral()

            # check pose erronea en camara
            if best_pose.pose.position.z > 0.8:
                print('pose en camara')
                return 'failed'

            # transformar a pose para manip
            #best_pose.pose.position.z = best_pose.pose.position.z - 0.09
            try:
                userdata.grab_pose = utils_hb.get_pose_relative_coordinate('map', best_pose)
                userdata.pre_pose = utils_hb.get_pose_relative_coordinate('map', pre_grasp)
            except: 
                return 'failed'
            
            # check por abajo
            #if userdata.grab_pose.pose.position.z > userdata.pre_pose.pose.position.z:
            #    print('por abajo')
            #    return 'failed'
            
            utils_hb.rviz_marker('/map', userdata.grab_pose.pose.position.x, userdata.grab_pose.pose.position.y, userdata.grab_pose.pose.position.z)

            print('BBBBBBBBBBBBB')
            print(userdata.grab_pose) 
            print('PREEEEEEEEEEEE')
            print(userdata.pre_pose)  
            print('BBBBBBBBBBBBB')         

            pose_pub = rospy.Publisher("/pre_nico", PoseStamped, queue_size=5)
            rospy.sleep(1)
            pose_pub.publish(userdata.pre_pose)

            pose_pub = rospy.Publisher("/nico", PoseStamped, queue_size=5)
            rospy.sleep(1)
            pose_pub.publish(userdata.grab_pose)

            #assert(0==1)

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

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'finish'])

    sm.userdata.grab_pose = tf2_geometry_msgs.PoseStamped()
    sm.userdata.pre_pose = tf2_geometry_msgs.PoseStamped()
    sm.userdata.object_pose = []
    sm.userdata.selected_object = ''
    sm.userdata.drop_counter = 0
    sm.userdata.timer = 0
    sm.userdata.width = 0
    sm.userdata.gpd_tries = 0
    sm.userdata.ban_list = ["nine_hole_peg_test","dice","chain","skillet"]

    with sm:

        smach.StateMachine.add('SETUP', Setup(),
            transitions={
                'succeeded': 'CT1', 
                'aborted': 'aborted'
            }
        )

        smach.StateMachine.add('CT1', CheckTime(),
            transitions={
                'continue': 'RESET', 
                'finish': 'finish'
            }
        )

        smach.StateMachine.add('RESET', ResetData(),
            transitions={
                'succeeded': 'GO_TO_PICKUP'
            }
        )

        smach.StateMachine.add('GO_TO_PICKUP', MoveSM('PICKUP'),
            transitions={
                'succeeded': 'CT2'                
            }
        )

        smach.StateMachine.add('CT2', CheckTime(),
            transitions={
                'continue': 'LOOK_OBJECT', 
                'finish': 'finish'
            }
        )

        smach.StateMachine.add('LOOK_OBJECT', look_object.getInstance(vis_model),
            transitions={
                'succeeded': 'CT5', 
                'failed': 'CT5'             
            }
        )

        smach.StateMachine.add('CT5', CheckTime(),
            transitions={
                'continue': 'GET_POSE', 
                'finish': 'finish'
            }
        )

        smach.StateMachine.add('GET_POSE', SetPose(vis_model),
            transitions={
                'succeeded': 'CT3',
                'failed': 'GET_POSE',
                'ban':'CT2'                
            }
        )

        smach.StateMachine.add('CT3', CheckTime(),
            transitions={
                'continue': 'GRAB_OBJECT', 
                'finish': 'finish'
            }
        )

        smach.StateMachine.add('GRAB_OBJECT', manipulation.getInstance(),
            transitions={
                'succeeded': 'CT4', 
                'failed': 'LOOK_OBJECT'               
            }
        )

        smach.StateMachine.add('CT4', CheckTime(),
            transitions={
                'continue': 'GO_TO_DROP', 
                'finish': 'finish'
            }
        )

        smach.StateMachine.add('GO_TO_DROP', GoToGoal(),
            transitions={
                'succeeded': 'DROP_OBJECT', 
                'failed': 'GO_TO_DROP'               
            }
        )

        smach.StateMachine.add('DROP_OBJECT', DropObject(),
            transitions={
                'succeeded': 'RESET'                
            }
        )

        return sm

if __name__ == '__main__':

    rospy.init_node('CLEANUP')

    sm = getInstance()

    outcome = sm.execute() # here is where the test begin
