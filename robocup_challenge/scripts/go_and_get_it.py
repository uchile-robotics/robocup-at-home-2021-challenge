#!/usr/bin/env python

import rospy
import smach

from go_to import *
from listener import *
import copy
import tf2_geometry_msgs
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist, WrenchStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker
#from utils import *

import utils_hb

import detection

import manipulation

class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"])
    def execute(self,userdata):
        utils_hb.move_arm_init()
        return 'succeeded'

class Look():
    def __init__(self):
        self.limit_sup = 1
        self.limit_inf = -1

    def move_head(self):
        move_down = [0, -0.2, -0.4, -0.6, -0.8]
        move_head_tilt(0)
        for value in move_down:
             move_head_tilt(value)
             print("  Mirando...  ")
             rospy.sleep(2)
        move_head_tilt(0)

# #ir a obstacule area
class GoObstacle(smach.State):
    def __init__(self, obc):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.obc = obc

    def execute(self, userdata):
        rospy.loginfo('Going to obstacles area')
        m = Move()
        m.set_pose(2.65, 1.81, 90)
        m.go()
        print(self.obc)
        self.obc.get_data()
        utils_hb.move_head_tilt(-0.9)
        rospy.sleep(0.1)
        return 'succeeded'

class GoGoal(smach.State):
    def __init__(self, obc):
        smach.State.__init__(self, outcomes=['succeeded'], io_keys=['goal_obj', 'goal_person'])
        self.obc = obc

    def execute(self, userdata):
        rospy.loginfo('Going to goal area')
        m = Move()
        m.set_pose(2.2, 4, 90)
        m.go()
        print(self.obc)
        obc_data = self.obc.get_data()
        rospy.sleep(0.1)
        utils_hb.move_head_tilt(0)
        userdata.goal_obj = obc_data.split()[0]
        userdata.goal_person = obc_data.split()[-1]
        print('OBJ: {}     PERSON: {}'.format(userdata.goal_obj, userdata.goal_person))
        return 'succeeded'

class GoPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], io_keys=['goal_obj', 'goal_person'])

    def execute(self, userdata):
        rospy.loginfo('Going to goal person')

        if userdata.goal_person == 'left':
            m = Move()
            m.set_pose(0.5, 2.8, 180)
            m.go()
            rospy.sleep(0.1)

        else:
            m = Move()
            m.set_pose(0.5, 4, 180)
            m.go()
            rospy.sleep(0.1)

        return 'succeeded'

class GoBottom1(smach.State):
    def __init__(self, obc):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.obc = obc

    def execute(self, userdata):
        rospy.loginfo('Going to goal area bottom 1')
        m = Move()
        m.set_pose(1.8, 3, 120)
        m.go()
        print(self.obc)
        self.obc.get_data()
        rospy.sleep(0.1)
        return 'succeeded'

class GoBottom2(smach.State):
    def __init__(self, obc):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.obc = obc

    def execute(self, userdata):
        rospy.loginfo('Going to goal area bottom 2')
        m = Move()
        m.set_pose(1.8, 4.23, 90)
        m.go()
        print(self.obc)
        self.obc.get_data()
        rospy.sleep(0.1)
        return 'succeeded'

class GoFront(smach.State):
    def __init__(self, obc):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.obc = obc

    def execute(self, userdata):
        rospy.loginfo('Going to goal area Front')
        m = Move()
        m.set_pose(2.65, 3.7, 120)
        m.go()
        print(self.obc)
        self.obc.get_data()
        rospy.sleep(0.1)
        return 'succeeded'

class DropObstacle(smach.State):
    def __init__(self, obc):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.obc = obc

    def execute(self, userdata):
        rospy.loginfo('Going to obstacles area')
        m = Move()
        m.set_pose(2.65, 1.81, 270)
        m.go()

        init_height = 0.1
        #init_joints = [init_height, -2.1, 0.0, 0.4, 0.0, 0]
        init_joints = [0.1, -0.7, 0.0, -0.4, 0.0, 0.0]
        utils_hb.arm.set_joint_value_target(init_joints)
        utils_hb.arm.go()

        utils_hb.move_hand(0.8)
        print(self.obc)
        self.obc.get_data()
        utils_hb.move_head_tilt(-0.9)
        rospy.sleep(0.1)
        utils_hb.move_arm_init()
        return 'succeeded'

class ClearObstacles(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self):
        rospy.loginfo('Removing Obstacles')

        rospy.sleep(0.1)
        return 'succeeded'

class Publish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], io_keys=['object_pose', 'grab_pose', 'pre_pose', 'selected_object', 'floor'])
    def execute(self, userdata):
        pose_pub = rospy.Publisher("/pre_nico", PoseStamped, queue_size=5)
        rospy.sleep(1)
        pose_pub.publish(userdata.pre_pose)

        pose_pub = rospy.Publisher("/nico", PoseStamped, queue_size=5)
        rospy.sleep(1)
        pose_pub.publish(userdata.grab_pose)

        return "succeeded"

class GraspTry(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], input_keys=["grab_pose"])
    def execute(self, userdata):
        #Robot grasps
        utils_hb.move_hand(0.8)
        utils_hb.whole_body.set_pose_target(userdata.grab_pose)
        utils_hb.whole_body.go()
        utils_hb.move_hand(0.1)

        return "succeeded"

class SetObstaclePose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], io_keys=['object_pose', 'grab_pose', 'pre_pose', 'selected_object', 'floor'])

    def execute(self,userdata):
        rospy.loginfo('Removing Obstacles')
        sm.userdata.floor = True
        grab_pose_precopy = utils_hb.get_pose_relative_coordinate('map', userdata.object_pose)

        userdata.grab_pose = copy.deepcopy(grab_pose_precopy)
        userdata.grab_pose.pose.position.z += 0.06

        ori = tf.transformations.quaternion_from_euler(np.pi, 0, 0)
        s_rot = tf.transformations.quaternion_from_euler(0, 0, -np.pi/2)
        f_mul = tf.transformations.quaternion_multiply(ori, s_rot)
        userdata.grab_pose.pose.orientation.x = f_mul[0]
        userdata.grab_pose.pose.orientation.y = f_mul[1]
        userdata.grab_pose.pose.orientation.z = f_mul[2]
        userdata.grab_pose.pose.orientation.w = f_mul[3]

        pre_grasp = copy.deepcopy(userdata.grab_pose)
        pre_grasp.pose.position.z = userdata.grab_pose.pose.position.z + 0.15
        userdata.pre_pose = utils_hb.get_pose_relative_coordinate('map', pre_grasp)

        print(type(userdata.grab_pose))
        
        utils_hb.rviz_marker('/map', userdata.grab_pose.pose.position.x, userdata.grab_pose.pose.position.y, userdata.grab_pose.pose.position.z)

        print('BBBBBBBBBBBBB')
        print(userdata.grab_pose)           

        pose_pub = rospy.Publisher("/pre_nico", PoseStamped, queue_size=5)
        rospy.sleep(1)
        pose_pub.publish(userdata.pre_pose)

        pose_pub = rospy.Publisher("/nico", PoseStamped, queue_size=5)
        rospy.sleep(1)
        pose_pub.publish(userdata.grab_pose)

        #assert(0==1)

        rospy.sleep(0.1)
        return 'succeeded'

class FindObject(smach.State):
    def __init__(self, vision_model, check_flag=False):
        smach.State.__init__(self, outcomes=["succeeded", "failed", "continue", 'check'], io_keys=['object_pose', 'all_objects', 'selected_object'])
        self.vision_model = vision_model
        self.flag = check_flag
    def execute(self,userdata):
        print('Looking For Object')
        
        objects = self.vision_model.detect()
        print(objects)

        if objects[-1] != []:

            if self.flag:
                userdata.all_objects = objects
                return 'check'

            for i, pose_single in enumerate(objects[0]):

                real_pose = utils_hb.make_pose_from_camera(pose_single)
                map_pose = utils_hb.get_pose_relative_coordinate('map', real_pose)
                xx = map_pose.pose.position.x
                
                if xx > 2.9:
                    continue

                get_object = objects[-1][i]
                userdata.selected_object = get_object
                pose = objects[0][i]
                print('Going for object: {} with pose {}'.format(get_object, pose))
                real_pose = utils_hb.make_pose_from_camera(pose)
                userdata.object_pose = real_pose
                return 'succeeded'

            return 'continue'

        print('Object not found')
        return 'failed'

class CheckObstacles(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", 'crucial', 'manip', 'bottom', 'front'], io_keys=['object_pose', 'all_objects'])

    def execute(self,userdata):
        print('Checking obstacles')
        front_problems = 0
        bottom_problems = 0
        print('inicial values {} {}'.format(front_problems, bottom_problems))
        n_prob = len(userdata.all_objects[0])

        for pose in userdata.all_objects[0]:

            real_pose = utils_hb.make_pose_from_camera(pose)
            map_pose = utils_hb.get_pose_relative_coordinate('map', real_pose)
            xx = map_pose.pose.position.x
            yy = map_pose.pose.position.y

            print(xx)
            print(yy)

            up_bound = 2.9
            low_bound = 2.35
            left_bound = 2.8

            # check if obstacle in vital position
            if xx < up_bound:
                if xx > low_bound:
                    if yy < left_bound:
                        return 'crucial'
                    else:
                        print('front p')
                        front_problems += 1
                else:
                    print('bottom p')
                    bottom_problems += 1

        print('final values {} {}'.format(front_problems, bottom_problems))
            
        if front_problems > 0 and bottom_problems > 0:
            return 'manip'

        if front_problems > 0: 
            return 'bottom'
        if bottom_problems > 0:
            return 'front'

        return 'succeeded'

class GetCloseObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], input_keys=['object_pose'])

    def execute(self,userdata):
        print('Getting Close to Object')
        try:
            # se tiene que usar la pose obtenida por vision
            map_pose = utils_hb.get_pose_relative_coordinate('map', userdata.object_pose)
            utils_hb.rviz_marker('map', map_pose.pose.position.x, map_pose.pose.position.y, map_pose.pose.position.z)
            grab_x = map_pose.pose.position.x + 0.1
            grab_y = map_pose.pose.position.y - 0.65
            print(grab_x)
            print(grab_y)
            m = utils_hb.Move()
            m.set_pose(grab_x, grab_y, 90)
            m.go()
            utils_hb.move_head_tilt(-0.9)
            rospy.sleep(0.1)
        except:
            rospy.logerr('fail to move')
            sys.exit()
        return 'succeeded'

class Ir_delivery(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Ir_delivery')
        print("  Ir a area de entrega  ")
        m = Move()
        m.set_pose(0.48, 3.46, 180)
        #m.get_pose()
        m.go()
        obc.get_data()
        #l = Look()
        #l.move_head()
        rospy.sleep(0.1)
        return 'outcome2'

class Ir_goal_area(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Ir_goal_area')
        print("  ir a area de goal  ")
        m = Move()
        m.set_pose(2.36, 3.41, 160)
        #m.get_pose()
        m.go()
        obc.get_data()
        #l = Look()
        #l.move_head()
        rospy.sleep(0.1)
        return 'succeeded'

class Ir_food_area(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Ir_food_area')
        print("  Ir a area de la comida  ")
        m = Move()
        m.set_pose(2.25, 4.18, 90)
        #m.get_pose()
        m.go()
        obc.get_data()
        #l = Look()
        #l.move_head()
        rospy.sleep(0)
        return 'succeeded'

class Ir_search_area(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Ir_search_area')
        print("  Ir a area de busqueda  ")
        m = Move()
        m.set_pose(0.6, 0.6, 90)
        #m.get_pose()
        m.go()
        obc.get_data()
        l = Look()
        l.move_head()
        rospy.sleep(5)
        return 'succeeded'

class Ir_deposit_area(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Ir_deposit_area')
        print("  Ir a area de deposito  ")
        m = Move()
        m.set_pose(1.58, 0.37, 270)
        #m.get_pose()
        m.go()
        obc.get_data()
        #l = Look()
        #l.move_head()
        rospy.sleep(0.1)
        return 'succeeded'

class PanHead(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], io_keys=['head_counter'])
    def execute(self,userdata):
        utils_hb.move_head_tilt(-0.7)
        userdata.head_counter = 1
        return 'succeeded'

class DropObstacle2(smach.State):
    def __init__(self, obc):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.obc = obc

    def execute(self, userdata):
        rospy.loginfo('Dropping to person')

        init_height = 0.1
        #init_joints = [init_height, -2.1, 0.0, 0.4, 0.0, 0]
        init_joints = [0.1, -0.7, 0.0, -0.4, 0.0, 0.0]
        utils_hb.arm.set_joint_value_target(init_joints)
        utils_hb.arm.go()

        utils_hb.move_hand(0.8)
        print(self.obc)
        self.obc.get_data()
        utils_hb.move_head_tilt(-0.9)
        rospy.sleep(0.1)
        utils_hb.move_arm_init()
        return 'succeeded'

class ShelfCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'oclu'], io_keys=['object_pose', 'all_objects', 'goal_obj', 'grab_pose', 'pre_pose', 'head_counter'])

    def calc_poses(self, raw_pose, frame_link):
        selec_pose = utils_hb.make_pose_from_camera(raw_pose)

        print('EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE')
        print(selec_pose)

        grab_pose_precopy = utils_hb.get_pose_relative_coordinate(frame_link, selec_pose)

        grab_pose = copy.deepcopy(grab_pose_precopy)
        #grab_pose.pose.position.y += 0.07

        ori = tf.transformations.quaternion_from_euler(0, np.pi/2, 0)
        s_rot = tf.transformations.quaternion_from_euler(0, 0, np.pi)
        ss_rot = tf.transformations.quaternion_from_euler(np.pi/2, 0, 0)
        f_mul = tf.transformations.quaternion_multiply(ori, s_rot)
        ff_mul = tf.transformations.quaternion_multiply(f_mul, ss_rot)
        grab_pose.pose.orientation.x = ff_mul[0]
        grab_pose.pose.orientation.y = ff_mul[1]
        grab_pose.pose.orientation.z = ff_mul[2]
        grab_pose.pose.orientation.w = ff_mul[3]

        pre_grasp = copy.deepcopy(grab_pose)
        pre_grasp.pose.position.y = grab_pose.pose.position.y - 0.15

        pre_pose = utils_hb.get_pose_relative_coordinate(frame_link, pre_grasp)

        return grab_pose, pre_pose



    def execute(self,userdata):
        print('Checking Shelf')
        
        objects = userdata.all_objects
        names = objects[-1]
        print('Object in Shelf: {}'.format(objects))
        print('Looking for object: {}'.format(userdata.goal_obj))

        if userdata.goal_obj in names:
            print('yay')
            obj_index = objects[-1].index(userdata.goal_obj)
            print('yay2')
            selec_pose_raw = objects[0][obj_index]

            userdata.grab_pose, userdata.pre_pose = self.calc_poses(selec_pose_raw, 'map')

            grab_x = userdata.grab_pose.pose.position.x + 0.1
            grab_y = userdata.grab_pose.pose.position.y - 0.7
            print(grab_x)
            print(grab_y)
            m = utils_hb.Move()
            m.set_pose(grab_x, grab_y, 90)
            m.go()

            pose_pub = rospy.Publisher("/pre_nico", PoseStamped, queue_size=5)
            rospy.sleep(1)
            pose_pub.publish(userdata.pre_pose)

            pose_pub = rospy.Publisher("/nico", PoseStamped, queue_size=5)
            rospy.sleep(1)
            pose_pub.publish(userdata.grab_pose)


            for i, obj in enumerate(names):
                if obj != userdata.goal_obj:
                    selec_pose_raw_oclu = objects[0][i]
                    oclu_pose, oclu_pre = self.calc_poses(selec_pose_raw_oclu, 'map')

                    deltax = np.abs(oclu_pose.pose.position.x - userdata.grab_pose.pose.position.x)
                    deltaz = np.abs(oclu_pose.pose.position.z - userdata.grab_pose.pose.position.z)
                    epsilon = 0.1
                    if deltax < epsilon and deltaz < epsilon:
                        userdata.grab_pose = copy.deepcopy(oclu_pose)
                        userdata.pre_pose = copy.deepcopy(oclu_pre)
                        return 'oclu'

            return 'succeeded'

        if userdata.head_counter == 1:
            print('choosing random object')
            selec_pose_raw_rand = objects[0][0]
            rand_pose, rand_pre = self.calc_poses(selec_pose_raw_rand, 'map')
            userdata.grab_pose = copy.deepcopy(rand_pose)
            userdata.pre_pose = copy.deepcopy(rand_pre)
            return 'succeeded'
                

        return 'failed'
        
class ShelfCheck2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], io_keys=['object_pose', 'all_objects', 'goal_obj', 'grab_pose', 'pre_pose'])

    def execute(self,userdata):
        print('Checking Shelf')
        
        objects = userdata.all_objects
        names = objects[-1]
        print('Object in Shelf: {}'.format(objects))
        print('Looking for object: {}'.format(userdata.goal_obj))

        if userdata.goal_obj in names:
            print('yay')
            obj_index = objects[-1].index(userdata.goal_obj)
            print('yay2')
            selec_pose_raw = objects[0][obj_index]
            selec_pose = utils_hb.make_pose_from_camera(selec_pose_raw)

            print('EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE')
            print(selec_pose)

            grab_pose_precopy = utils_hb.get_pose_relative_coordinate('base_link', selec_pose)

            userdata.grab_pose = copy.deepcopy(grab_pose_precopy)

            rel_cord = utils_hb.get_relative_coordinate('base_link','hand_palm_link')
            userdata.grab_pose.pose.y = rel_cord.translation.y

            ori = tf.transformations.quaternion_from_euler(0, np.pi/2, 0)
            s_rot = tf.transformations.quaternion_from_euler(0, 0, np.pi)
            ss_rot = tf.transformations.quaternion_from_euler(np.pi/2, 0, 0)
            f_mul = tf.transformations.quaternion_multiply(ori, s_rot)
            ff_mul = tf.transformations.quaternion_multiply(f_mul, ss_rot)
            userdata.grab_pose.pose.orientation.x = ff_mul[0]
            userdata.grab_pose.pose.orientation.y = ff_mul[1]
            userdata.grab_pose.pose.orientation.z = ff_mul[2]
            userdata.grab_pose.pose.orientation.w = ff_mul[3]

            pre_grasp = copy.deepcopy(userdata.grab_pose)
            pre_grasp.pose.position.x = userdata.grab_pose.pose.position.x - 0.15

            userdata.pre_pose = utils_hb.get_pose_relative_coordinate('base_link', pre_grasp)

            pose_pub = rospy.Publisher("/pre_nico", PoseStamped, queue_size=5)
            rospy.sleep(1)
            pose_pub.publish(userdata.pre_pose)

            pose_pub = rospy.Publisher("/nico", PoseStamped, queue_size=5)
            rospy.sleep(1)
            pose_pub.publish(userdata.grab_pose)

            assert(0==1)

            #for i, obj in enumerate(names):
            #    if obj != userdata.goal_obj:
            #        pass

            return 'succeeded'

        return 'failed'

# main
def getInstance():

    obc = Message_listener()

    # Se crea el modelo 
    print('CARGANDO MODELO')
    vis_model = detection.RGBD()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

    sm.userdata.object_pose = ''
    sm.userdata.selected_object = ''
    sm.userdata.grab_pose = tf2_geometry_msgs.PoseStamped()
    sm.userdata.pre_pose = tf2_geometry_msgs.PoseStamped()
    sm.userdata.floor = False
    sm.userdata.under = False
    sm.userdata.goal_obj = ''
    sm.userdata.goal_person = ''
    sm.userdata.goal_obj_pose = 0
    sm.userdata.all_objects = []
    sm.userdata.head_counter = 0

    # Open the container
    with sm:
        # Add states to the container

        smach.StateMachine.add('SETUP', Setup(),
            transitions={
                'succeeded': 'GO_OBSTACLE', 
            }
        )

        smach.StateMachine.add('GO_OBSTACLE', GoObstacle(obc),
            transitions={
                'succeeded':'FIND_OBJECT_CHECK'
                }
        )

        smach.StateMachine.add('FIND_OBJECT_CHECK', FindObject(vis_model, check_flag=True),
            transitions={
                'succeeded': 'SET_OBSTACLES_POSE', 
                'failed': 'GO_GOAL',
                'continue': 'GO_GOAL', 
                'check': 'CHECK_PATHS'
            }
        )

        smach.StateMachine.add('CHECK_PATHS', CheckObstacles(),
            transitions={
                'succeeded':'GO_GOAL',
                'crucial': 'FIND_OBJECT',
                'manip': 'FIND_OBJECT',
                'bottom': 'GO_BOTTOM1',
                'front': 'GO_FRONT'
                }
        )

        smach.StateMachine.add('FIND_OBJECT', FindObject(vis_model),
            transitions={
                'succeeded': 'SET_OBSTACLES_POSE', 
                'failed': 'FIND_OBJECT',
                'continue': 'GO_GOAL', 
                'check': 'CHECK_PATHS'
            }
        )

        
        smach.StateMachine.add('SET_OBSTACLES_POSE', SetObstaclePose(),
            transitions={
                'succeeded':'CLEAR_OBSTACLES'
                }
        )

        
        smach.StateMachine.add('CLEAR_OBSTACLES', manipulation.getInstance(),
            transitions={
                'succeeded': 'DROP_OBSTACLE', 
                'failed': 'GO_OBSTACLE'               
            }
        )

        smach.StateMachine.add('DROP_OBSTACLE', DropObstacle(obc),
            transitions={
                'succeeded': 'GO_OBSTACLE'             
            }
        )

        #################### GOAL #############################

        smach.StateMachine.add('GO_BOTTOM1', GoBottom1(obc),
            transitions={
                'succeeded':'GO_BOTTOM2'
                }
        )

        smach.StateMachine.add('GO_BOTTOM2', GoBottom2(obc),
            transitions={
                'succeeded':'GO_GOAL'
                }
        )

        smach.StateMachine.add('GO_FRONT', GoFront(obc),
            transitions={
                'succeeded':'GO_GOAL'
                }
        )

        smach.StateMachine.add('GO_GOAL', GoGoal(obc),
            transitions={
                'succeeded':'FIND_OBJECT_SHELF'
                }
        )

        ######### GRAB OBJECT ############################

        smach.StateMachine.add('SHELF_PAN', PanHead(),
            transitions={
                'succeeded':'FIND_OBJECT_SHELF'
                }
        )
        
        smach.StateMachine.add('FIND_OBJECT_SHELF', FindObject(vis_model, check_flag=True),
            transitions={
                'succeeded': 'FIND_OBJECT_SHELF', 
                'failed': 'FIND_OBJECT_SHELF',
                'continue': 'FIND_OBJECT_SHELF', 
                'check': 'CHECK_SHELF'
                }
        )

        smach.StateMachine.add('CHECK_SHELF', ShelfCheck(),
            transitions={
                'succeeded':'GRAB_GOAL',
                'failed':'SHELF_PAN', 
                'oclu': 'GRAB_GOAL'
                }
        )

        smach.StateMachine.add('GRAB_GOAL', manipulation.getInstance(),
            transitions={
                'succeeded': 'GO_PERSON', 
                'failed': 'failed'               
            }
        )

        smach.StateMachine.add('GO_PERSON', GoPerson(),
            transitions={
                'succeeded':'DROP_OBSTACLE2'
                }
        )

        smach.StateMachine.add('DROP_OBSTACLE2', DropObstacle2(obc),
            transitions={
                'succeeded': 'succeeded'             
            }
        )
        


        
        '''
        smach.StateMachine.add('Ir_goal_area', Ir_goal_area(),
            transitions={
                'succeeded':'Ir_food_area',
                'outcome2':'succeeded'}
        )

        smach.StateMachine.add('Ir_food_area', Ir_food_area(),
            transitions={
                'succeeded':'Ir_delivery',
                'outcome2':'succeeded'}
        )

        smach.StateMachine.add('Ir_delivery', Ir_delivery(),
            transitions={'outcome2':'succeeded'}
        )

        '''

        return sm


if __name__ == '__main__':

    rospy.init_node('smach_example_state_machine')

    sm = getInstance()

    outcome = sm.execute() # here is where the test begin
