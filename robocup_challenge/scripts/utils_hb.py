#!/usr/bin/env python

import rospy
import actionlib
import roslib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist
import moveit_commander
import tf2_ros

class Move():
    def __init__(self):
        self.pose = self

    def set_pose(self,x,y,theta):
        self.pose.x = x
        self.pose.y = y
        self.pose.theta = theta

    def get_pose(self):
        print(self.pose.x, self.pose.y, self.pose.theta)

    def euler_to_cuater(self):
        q = tf.transformations.quaternion_from_euler(0, 0, self.pose.theta/180.0*math.pi, 'rxyz')
        return Quaternion(q[0], q[1], q[2], q[3])

    def movebase_client(self):

        client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = self.pose.x #2
        goal.target_pose.pose.position.y = self.pose.y #3

        goal.target_pose.pose.orientation = self.euler_to_cuater()

        client.send_goal(goal)

        wait = client.wait_for_result()

        if not wait:
            # fail
            rospy.logerr("server not available!")
            rospy.signal_shutdown("server not available!")
        else:
            # go
            return client.get_result()

    def go(self):
        try:
            result = self.movebase_client()
            if result:
                rospy.loginfo("Goal execution done!")
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")


head = moveit_commander.MoveGroupCommander("head")


def move_head_tilt(v):

    head.set_joint_value_target("head_tilt_joint", v)
    return head.go()
    
def get_pose_relative_coordinate(targ_frame, p):

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.transform(p, targ_frame, rospy.Duration(4.0))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
            rospy.loginfo("waiting...")
            rate.sleep()
            continue

    return trans.pose


arm = moveit_commander.MoveGroupCommander('arm')

def move_arm_ik(x, y, z, roll, pitch, yaw):

    p = PoseStamped()

    p.header.frame_id = "/base_link"

    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z

    p.pose.orientation = quaternion_from_euler(roll, pitch, yaw)

    arm.set_pose_target(p)
    return arm.go()

def move_arm_neutral():

    arm.set_named_target('neutral')
    return arm.go()


def move_arm_init():

    arm.set_named_target('go')
    return arm.go()

base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)


def move_base_vel(vx, vy, vw):

    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.angular.z = vw / 180.0 * math.pi 
    base_vel_pub.publish(twist)  

gripper = moveit_commander.MoveGroupCommander("gripper")

def move_hand(v):
    gripper.set_joint_value_target("hand_motor_joint", v)
    success = gripper.go()
    rospy.sleep(6)
    return success

def get_relative_coordinate(parent, child):

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    trans = TransformStamped()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(parent, child,
                                              rospy.Time().now(),rospy.Duration(1.0))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
            rospy.loginfo("waiting...")
            rate.sleep()
            continue

    return trans.transform