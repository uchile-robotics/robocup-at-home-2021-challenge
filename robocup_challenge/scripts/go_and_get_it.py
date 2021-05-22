#!/usr/bin/env python
from utils_hb import *

class Goal_area(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Goal_area')
        m = Move()
        m.set_pose(2, 3, 60)
        m.go()
        return 'outcome1'

class Delivery_area(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Delivery_area')
        m = Move()
        m.set_pose(0.5, 3, 60)
        m.go()
        return 'outcome2'

class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 2:
            self.counter += 1
            rospy.init_node('fish')
            m = Move()
            m.set_pose(2, 3, 60)
            m.go()
            return 'outcome1'
        else:
            return 'outcome2'
            rospy.init_node('fish')
            m = Move()
            m.set_pose(2, 3, 150)
            m.go()

class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        rospy.init_node('fish')
        m = Move()
        m.set_pose(1, 3, 180)
        m.go()
        return 'outcome1'


def main():
    rospy.init_node('fish')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(),
                               transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(),
                               transitions={'outcome1':'FOO'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
