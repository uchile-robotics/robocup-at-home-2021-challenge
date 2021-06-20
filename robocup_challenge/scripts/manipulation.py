#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import moveit_commander
import rospy
import tf
import utils_hb
import smach
import smach_ros
from geometry_msgs.msg import Pose

class PoseConvertions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], input_keys=["in_pose"], output_keys=["pg_pose","g_pose"])
    def execute(self, userdata):
        #Get grasp position relative to the 'odom' frame
        final_pose = tf2gm.PoseStamped()
        final_pose.pose = get_pose_relative_coordinate('odom',userdata.in_pose)
        final_pose.header = 'odom'

        #Get PreGrasping position
        pg_pose = tf2gm.PoseStamped()
        #pg_pose = pregrasp_calc(userdata.in_pose) #has to be "tf2gm.PoseStamped()"

        #Get pregrasp position relative to the 'odom' frame
        obj_pose = tf2gm.PoseStamped()
        obj_pose.pose = get_pose_relative_coordinate('odom',pg_pose)
        obj_pose.header = 'odom'

        return "succeeded"

class PreGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], input_keys=["pre_pose"])
    def execute(self, userdata):
        #Robot movement to pregrasping by IK
        utils_hb.whole_body.set_pose_target(userdata.pre_pose)
        utils_hb.whole_body.go()

        return "succeeded"

class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], input_keys=["grab_pose"])
    def execute(self, userdata):
        #Robot grasps
        utils_hb.move_hand(1.0)
        utils_hb.whole_body.set_pose_target(userdata.grab_pose.pose)
        utils_hb.whole_body.go()
        utils_hb.move_hand(0.2)
        
        '''
        pose_goal = Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4
        utils_hb.whole_body.set_pose_target(pose_goal)
        utils_hb.whole_body.go()

        utils_hb.move_hand(0.0)
        '''

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

def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded'], input_keys=['grab_pose', 'pre_pose'])

    with sm:

        smach.StateMachine.add('GO_TO_PREGRASP', PreGrasp(),
            transitions={
                'succeeded': 'GRASP'                
            }
        )

        smach.StateMachine.add('GRASP', Grasp(),
            transitions={
                'succeeded': 'NEUTRAL',         
            }
        )

        smach.StateMachine.add('NEUTRAL', Neutral(),
            transitions={
                'succeeded': 'succeeded'                
            }
        )

        return sm

if __name__ == '__main__':

    rospy.init_node('MANIP')

    sm = getInstance()

    outcome = sm.execute() # here is where the test begin
