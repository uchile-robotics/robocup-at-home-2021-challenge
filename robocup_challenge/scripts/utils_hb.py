#!/usr/bin/env python

import rospy
import actionlib
import roslib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist
import moveit_commander
import tf2_ros
import tf, tf2_geometry_msgs
from visualization_msgs.msg import Marker

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
            trans = tfBuffer.transform(p, targ_frame, rospy.Duration(10.0))
            break
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
            rospy.loginfo("waiting...")
            rate.sleep()
            continue

    return trans


arm = moveit_commander.MoveGroupCommander('arm')
#arm.allow_replanning(True)
arm.set_workspace([-3, -3, -1, 3, 3, 5])
whole_body = moveit_commander.MoveGroupCommander("whole_body_light")
whole_body.allow_replanning(True)
whole_body.set_workspace([-9.0, -9.0, 9.0, 9.0])

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

def make_pose_from_camera(simple_pose):
    pose = tf2_geometry_msgs.PoseStamped()    

    pose.header.frame_id = "head_rgbd_sensor_rgb_frame"  

    pose.pose.position.x = simple_pose[0]
    pose.pose.position.y = simple_pose[1]
    pose.pose.position.z = simple_pose[2]
    ori = quaternion_from_euler(0, 0, -90)
    pose.pose.orientation.x = ori[0]
    pose.pose.orientation.y = ori[1]
    pose.pose.orientation.z = ori[2]
    pose.pose.orientation.w = ori[3]

    return pose

def rviz_marker(frame_id_name, x_in, y_in, z_in):
    marker = Marker()
    marker.header.frame_id = frame_id_name
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x_in
    marker.pose.position.y = y_in
    marker.pose.position.z = z_in

    rviz_publisher = rospy.Publisher("/visualization_marker", Marker, queue_size=5)
    rospy.sleep(1)
    rviz_publisher.publish(marker)

def get_current_time_sec():
    return rospy.Time.now().to_sec()
