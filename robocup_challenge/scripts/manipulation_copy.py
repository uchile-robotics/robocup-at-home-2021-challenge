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


class PreGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], input_keys=["pre_pose"])
    def execute(self, userdata):
        #Robot movement to pregrasping by IK
        utils_hb.arm.set_pose_target(userdata.pre_pose)
        utils_hb.arm.go()

        return "succeeded"

class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], input_keys=["grab_pose", 'floor'])
    def execute(self, userdata):
        #Robot grasps
        #if userdata.floor:
        #    utils_hb.move_hand(1.0)
        #else:
        #    utils_hb.move_hand(0.8)
        utils_hb.move_hand(1.0)
        utils_hb.arm.set_pose_target(userdata.grab_pose)
        utils_hb.arm.go()
        utils_hb.move_hand(0)

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
        utils_hb.move_arm_neutral()

        return "succeeded"

class Torque(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", 'failed'])
    def execute(self, userdata):
        wrench_raw = rospy.wait_for_message("/hsrb/wrist_wrench/raw",  WrenchStamped)
        torque = np.abs(wrench_raw.wrench.torque.x)
        if torque > 0.5:
            return "succeeded"
        return "failed"

class GetSafe(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], input_keys=['object_pose'])

    def execute(self,userdata):
        print('Getting Safe')
        start_time = utils_hb.get_current_time_sec()  
        while utils_hb.get_current_time_sec() - start_time < 0.5:  
            utils_hb.move_base_vel(-0.1, 0, 0)
        return 'succeeded'
            
def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded', 'failed'], input_keys=['grab_pose', 'pre_pose', 'object_pose', 'floor'])

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

        smach.StateMachine.add('PREGRASP2', PreGrasp(),
            transitions={
                'succeeded': 'SAFE',         
            }
        )
        
        smach.StateMachine.add('SAFE', GetSafe(),
            transitions={
                'succeeded': 'NEUTRAL',         
            }
        )

        smach.StateMachine.add('NEUTRAL', Neutral(),
            transitions={
                'succeeded': 'succeeded'                
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
