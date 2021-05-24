#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String

#print("pescado")

class Message_listener():
    def __init__(self):
        self.data  = self
        self.sub = rospy.Subscriber("message", String, self.get_data)

    def callback(self,data):
        self.data = data.data
        #print(self.data)
        return self.data

    def get_data(self,data):
        print(self.data)
        return self.data

def main(args):
  obc = Message_listener()
  rospy.init_node('Message_listener', anonymous=True)
  obc.get_data()
  print(type(obc.get_data()))

if __name__ == '__main__':
    main(sys.argv)
