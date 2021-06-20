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
        smach.State.__init__(self, outcomes=["succeeded", "aborted"])
    def execute(self,userdata):
        utils_hb.move_arm_init()
        return 'succeeded'

class ResetData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], io_keys=['object_pose', 'grab_pose', 'selected_object', 'floor'])
    def execute(self,userdata):
        sm.userdata.grab_pose = tf2_geometry_msgs.PoseStamped()
        sm.userdata.object_pose = []
        sm.userdata.selected_object = ''
        sm.userdata.floor = False
        sm.userdata.under = False
        
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
        init_height = 0.35
        init_joints = [init_height, -2.1, 0.0, 0.4, 0.0, 0]
        utils_hb.arm.set_joint_value_target(init_joints)
        utils_hb.arm.go()
        utils_hb.move_hand(0.8)
        utils_hb.move_arm_neutral()
        utils_hb.move_hand(0.0)
        
        '''
        init_torque = wrench_raw.wrench.torque.y
        actual_height = init_height
        height_step = 0.005
        while not rospy.is_shutdown():
            actual_height = actual_height - height_step
            init_joints = [actual_height, -2.1, 0.0, 0.4, 0.0, 0]
            utils_hb.arm.set_joint_value_target(init_joints)
            utils_hb.arm.go()
            wrench_raw = rospy.wait_for_message("/hsrb/wrist_wrench/raw",  WrenchStamped)
            if np.abs(wrench_raw.wrench.torque.y) > np.abs(init_torque) + torque_offset:
                utils_hb.move_hand(0.8)
                utils_hb.move_arm_neutral()
                utils_hb.move_hand(0.0)
                utils_hb.move_arm_init()
                break
            '''

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
                m.set_pose(0.8, 0.9, 90)
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

class SetPose(smach.State):
    def __init__(self, vision_model):
        smach.State.__init__(self, outcomes=["succeeded", "failed"], io_keys=['object_pose', 'grab_pose', 'selected_object', 'pre_pose', 'floor', 'under'])
        self.vision_model = vision_model
    def execute(self,userdata):
        print('&&&&&&&&&&&&&&&&&&&&&&&&&')
        print(userdata.floor)
        print(userdata.under)
        print('&&&&&&&&&&&&&&&&&&&&&&&&&')
        utils_hb.move_arm_init()
        if userdata.floor or userdata.under:
            print('yay3')
             # get updated object pose
            objects = self.vision_model.detect()
            print(objects)

            print('CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC')
            planning_frame = utils_hb.whole_body.get_planning_frame()
            print("============ Reference frame: {}".format(planning_frame))

            if objects[-1] != []:
                try:
                    obj_index = objects[-1].index(userdata.selected_object)
                except:
                    print('object not refound')
                    return 'failed'
                selec_pose_raw = objects[0][obj_index]
                selec_pose = utils_hb.make_pose_from_camera(selec_pose_raw)

                print(type(selec_pose))

                utils_hb.move_arm_neutral()
                # transformar a pose para manip
                if userdata.floor:
                    grab_pose_precopy = utils_hb.get_pose_relative_coordinate('odom', selec_pose)
                elif userdata.under:
                    grab_pose_precopy = utils_hb.get_pose_relative_coordinate('odom', selec_pose)

                userdata.grab_pose = copy.deepcopy(grab_pose_precopy)
                userdata.grab_pose.pose.position.z += 0.07

                ori = tf.transformations.quaternion_from_euler(np.pi, 0, 0)
                s_rot = tf.transformations.quaternion_from_euler(0, 0, np.pi/2)
                f_mul = tf.transformations.quaternion_multiply(ori, s_rot)
                userdata.grab_pose.pose.orientation.x = f_mul[0]
                userdata.grab_pose.pose.orientation.y = f_mul[1]
                userdata.grab_pose.pose.orientation.z = f_mul[2]
                userdata.grab_pose.pose.orientation.w = f_mul[3]

                pre_grasp = copy.deepcopy(userdata.grab_pose)
                pre_grasp.pose.position.z = userdata.grab_pose.pose.position.z + 0.15
                if userdata.floor:
                    userdata.pre_pose = utils_hb.get_pose_relative_coordinate('odom', pre_grasp)
                elif userdata.under:
                    #pre_grasp.pose.position.y = 1.45
                    userdata.pre_pose = utils_hb.get_pose_relative_coordinate('odom', pre_grasp)


                print(type(userdata.grab_pose))
                
                utils_hb.rviz_marker('/odom', userdata.grab_pose.pose.position.x, userdata.grab_pose.pose.position.y, userdata.grab_pose.pose.position.z)

                print('BBBBBBBBBBBBB')
                print(userdata.grab_pose)           

                pose_pub = rospy.Publisher("/pre_nico", PoseStamped, queue_size=5)
                rospy.sleep(1)
                pose_pub.publish(userdata.pre_pose)

                pose_pub = rospy.Publisher("/nico", PoseStamped, queue_size=5)
                rospy.sleep(1)
                pose_pub.publish(userdata.grab_pose)

                #assert(0==1)

                return 'succeeded'

        else:
            # check if mask exists
            obj_mask = self.vision_model.segmentation()
            print('yay')

            if obj_mask != []:
                
                # create GPD receiver
                gpd_receiver = gpd_server.PostGPD()
                print('yay2')

                print('sleeping')
                rospy.sleep(10)
                print('done sleeping')

                # receive best pose
                best_pose, pre_grasp, score = gpd_receiver.get_best_grasp()
                print('###################')
                print('best pose')
                print(best_pose)
                print('score: {}'.format(score))

                # preparar mano
                utils_hb.move_arm_neutral()

                # check pose erronea en camara
                if pre_grasp.pose.position.z < 0:
                    print('pose en camara')
                    return 'failed'

                # transformar a pose para manip
                #best_pose.pose.position.z = best_pose.pose.position.z - 0.09
                try:
                    userdata.grab_pose = utils_hb.get_pose_relative_coordinate('odom', best_pose)
                    userdata.pre_pose = utils_hb.get_pose_relative_coordinate('odom', pre_grasp)
                except: 
                    return 'failed'
                
                # check por abajo
                if userdata.grab_pose.pose.position.z > userdata.pre_pose.pose.position.z:
                    print('por abajo')
                    return 'failed'
                
                utils_hb.rviz_marker('/odom', userdata.grab_pose.pose.position.x, userdata.grab_pose.pose.position.y, userdata.grab_pose.pose.position.z)

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

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])

    sm.userdata.grab_pose = tf2_geometry_msgs.PoseStamped()
    sm.userdata.pre_pose = tf2_geometry_msgs.PoseStamped()
    sm.userdata.object_pose = []
    sm.userdata.selected_object = ''
    sm.userdata.floor = False
    sm.userdata.under = False
    sm.userdata.drop_counter = 0

    with sm:

        smach.StateMachine.add('SETUP', Setup(),
            transitions={
                'succeeded': 'RESET', 
                'aborted': 'aborted'
            }
        )

        smach.StateMachine.add('RESET', ResetData(),
            transitions={
                'succeeded': 'GO_TO_PICKUP'
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
                'succeeded': 'GO_TO_DROP', 
                'failed': 'LOOK_OBJECT'               
            }
        )

        smach.StateMachine.add('GO_TO_DROP', MoveSM('DROP'),
            transitions={
                'succeeded': 'DROP_OBJECT'                
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