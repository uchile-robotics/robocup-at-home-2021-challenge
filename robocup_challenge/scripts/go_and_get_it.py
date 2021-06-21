#!/usr/bin/env python

import rospy
import smach

from go_to import *
from listener import *
import copy
#from utils import *

import utils_hb

import detection

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
        m.set_pose(2.65, 2.01, 105)
        m.go()
        print(self.obc)
        self.obc.get_data()
        utils_hb.move_head_tilt(-0.8)
        rospy.sleep(0.1)
        return 'succeeded'

class ClearObstacles(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self):
        rospy.loginfo('Removing Obstacles')

        rospy.sleep(0.1)
        return 'succeeded'

class FindObject(smach.State):
    def __init__(self, vision_model):
        smach.State.__init__(self, outcomes=["succeeded", "failed", "continue"], io_keys=['object_pose', 'selected_object'])
        self.vision_model = vision_model
    def execute(self,userdata):
        print('Looking For Object')
        
        objects = self.vision_model.detect()
        print(objects)

        if objects[-1] != []:
            get_object = objects[-1][0]
            userdata.selected_object = get_object
            pose = objects[0][0]
            print('Going for object: {} with pose {}'.format(get_object, pose))
            real_pose = utils_hb.make_pose_from_camera(pose)
            userdata.object_pose = real_pose
            return 'succeeded'

        print('Object not found')
        return 'failed'

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
            utils_hb.move_head_tilt(-0.8)
            rospy.sleep(0.1)
        except:
            rospy.logerr('fail to move')
            sys.exit()
        return 'succeeded'


# define state Ir_delivery
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

# ir a goal_area
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

# define state Ir_food_area
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

# main
def getInstance():

    obc = Message_listener()

    # Se crea el modelo 
    print('CARGANDO MODELO')
    vis_model = detection.RGBD()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded'])

    sm.userdata.object_pose = ''
    sm.userdata.selected_object = ''

    # Open the container
    with sm:
        # Add states to the container

        smach.StateMachine.add('GO_OBSTACLE', GoObstacle(obc),
            transitions={
                'succeeded':'FIND_OBJECT'
                }
        )

        smach.StateMachine.add('FIND_OBJECT', FindObject(vis_model),
            transitions={
                'succeeded': 'GET_CLOSE', 
                'failed': 'FIND_OBJECT',
                'continue': 'FIND_OBJECT'           
            }
        )
        
        smach.StateMachine.add('GET_CLOSE', GetCloseObject(),
            transitions={
                'succeeded': 'CLEAR_OBSTACLES'                
            }
        )

        
        smach.StateMachine.add('CLEAR_OBSTACLES', ClearObstacles(),
            transitions={
                'succeeded':'succeeded'
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
