#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import moveit_commander
import rospy
import tf
import utils_hb
import smach
import smach_ros
from geometry_msgs.msg import Pose, WrenchStamped
import numpy as np
from moveit_msgs.msg import (
    RobotTrajectory,
    PlaceLocation,
    Constraints,
    RobotState,
)
from moveit_msgs.msg import (
    MoveItErrorCodes,
    TrajectoryConstraints,
    PlannerInterfaceDescription,
    MotionPlanRequest,
)


class PreGrasp(smach.State):
    def __init__(self, open=True):
        smach.State.__init__(self, outcomes=["succeeded"], input_keys=["pre_pose", 'width'])
        self.open = open
    def execute(self, userdata):
        #Robot movement to pregrasping by IK
        #constraints = TrajectoryConstraints()
        #constraints.constraints[0].joint_constraints[0].joint_name = 'arm_roll_joint'
        #constraints.constraints[0].joint_constraints[0].position = 0
        #constraints.constraints[0].joint_constraints[0].tolerance_above = 0.2
        #constraints.constraints[0].joint_constraints[0].tolerance_below = -0.2

        #utils_hb.whole_body.set_trajectory_constraints(constraints)
        if self.open:
            utils_hb.move_hand(1)
        utils_hb.whole_body.set_pose_target(userdata.pre_pose)
        utils_hb.whole_body.go()

        return "succeeded"

class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], input_keys=["grab_pose", 'width', 'simple_flag'])
    def execute(self, userdata):
        #Robot grasps
        #if userdata.floor:
        #    utils_hb.move_hand(1.0)
        #else:
        #    utils_hb.move_hand(0.8)
        #if userdata.simple_flag:
        #    final_width = 1.0
        #    utils_hb.move_hand(final_width)
        #    utils_hb.whole_body.set_pose_target(userdata.grab_pose)
        #    utils_hb.whole_body.go()
        #    utils_hb.move_hand(0)
        #else:
        w_offset = 0.1
        print('OFFSET: {}'.format(userdata.width.data/0.126))
        final_width = min(userdata.width.data/0.126 + w_offset, 1.0)
        utils_hb.move_hand(final_width)
        utils_hb.whole_body.set_pose_target(userdata.grab_pose)
        utils_hb.whole_body.go()
        utils_hb.move_hand(max(userdata.width.data/0.126 - 5*w_offset, 0))

        #utils_hb.whole_body.clear_trajectory_constraints()

        return "succeeded"

class GraspCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"])
    def execute(self, userdata):
        #NOT IMPLEMENTED
        return "succeeded"

class Neutral(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"])
    def execute(self, userdata):
        #Robot to Neutral
        utils_hb.move_arm_init()

        return "succeeded"

class Torque(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", 'failed'])
    def execute(self, userdata):
        #torque_counter = 0
        for _ in range(10):
            wrench_raw = rospy.wait_for_message("/hsrb/wrist_wrench/raw",  WrenchStamped)
            torque = np.abs(wrench_raw.wrench.torque.x)
            print(torque)
            if torque > 0.2:
        #        torque_counter += 1
                return "succeeded"
            rospy.sleep(0.1)
        #print('torque counter: {}'.format(torque_counter))
        #if torque_counter >= 4:
            

        return "failed"

class GetSafe(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], input_keys=['object_pose'])

    def execute(self,userdata):
        print('Getting Safe')
        start_time = utils_hb.get_current_time_sec()  
        while utils_hb.get_current_time_sec() - start_time < 1:  
            utils_hb.move_base_vel(-0.2, 0, 0)
        return 'succeeded'
            
def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded', 'failed'], input_keys=['grab_pose', 'pre_pose', 'object_pose', 'width', 'simple_flag'])

    with sm:

        smach.StateMachine.add('GO_TO_PREGRASP', PreGrasp(),
            transitions={
                'succeeded': 'GRASP'                
            }
        )

        smach.StateMachine.add('GRASP', Grasp(),
            transitions={
                'succeeded': 'PREGRASP2',         
            }
        )

        smach.StateMachine.add('PREGRASP2', PreGrasp(open=False),
            transitions={
                'succeeded': 'NEUTRAL',         
            }
        )
        
        smach.StateMachine.add('SAFE', GetSafe(),
            transitions={
                'succeeded': 'NEUTRAL',         
            }
        )

        smach.StateMachine.add('NEUTRAL', Neutral(),
            transitions={
                'succeeded': 'TORQUE'                
            }
        )

        smach.StateMachine.add('TORQUE', Torque(),
            transitions={
                'succeeded': 'succeeded',
                'failed': 'failed'               
            }
        )

        return sm

if __name__ == '__main__':

    rospy.init_node('MANIP')

    sm = getInstance()

    outcome = sm.execute() # here is where the test begin
