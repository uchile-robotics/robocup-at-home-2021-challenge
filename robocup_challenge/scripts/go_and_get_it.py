#!/usr/bin/env python

import rospy
import smach

from go_to import *
from message_listener import *
#from utils import *

obc = Message_listener()
#rospy.init_node('Message_listener', anonymous=True)
obc.get_data()

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
class Ir_obstacle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Ir_obstacle')
        self.counter += 1
        print("  ir area de obstaculos  ")
        m = Move()
        m.set_pose(2.65, 2.01, 105)
        #m.get_pose()
        m.go()
        obc.get_data()
        #l = Look()
        #l.move_head()
        move_head_tilt(-0.8)
        rospy.sleep(0.1)
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
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'outcome2'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Ir_obstacle', Ir_obstacle(),
                               transitions={'succeeded':'Ir_goal_area',
                                            'outcome2':'succeeded'})
        smach.StateMachine.add('Ir_goal_area', Ir_goal_area(),
                               transitions={'succeeded':'Ir_food_area',
                                            'outcome2':'succeeded'})
        smach.StateMachine.add('Ir_food_area', Ir_food_area(),
                               transitions={'succeeded':'Ir_delivery',
                                            'outcome2':'succeeded'})
        smach.StateMachine.add('Ir_delivery', Ir_delivery(),
                               transitions={'outcome2':'succeeded'})


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
