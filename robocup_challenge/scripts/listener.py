#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String

class Message_listener():
    def __init__(self):
        self.data  = ''
        self.sub = rospy.Subscriber("/message", String, self.callback)

    def callback(self, data):
        self.data = data.data

    def get_data(self):
        print(self.data)
        return self.data
