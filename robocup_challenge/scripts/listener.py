#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String

#print("pescado")

class Message_listener():
    def __init__(self):
        self.data  = self
        self.sub = rospy.Subscriber("message", String, self.get_data)

    def get_data(self,data):
        self.data = data.data
        print(self.data)
        return self.data

if __name__ == '__main__':
    rospy.init_node('simple_class', anonymous=True)
    obc = Message_listener()
    
