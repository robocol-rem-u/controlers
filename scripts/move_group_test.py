#!/usr/bin/env python3
import sys
print(sys.version)
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import std_msgs
# path = "/opt/ros/melodic/lib/python2.7/dist-packages/moveit_commander/"
# sys.path.append(path)

# import importlib.util
# spec = importlib.util.spec_from_file_location("roscpp_initialize", "/opt/ros/melodic/lib/python2.7/dist-packages/moveit_commander/roscpp_initializer.py")
# foo = importlib.util.module_from_spec(spec)

# print(sys.modules)
# print(foo.__name__)

def move_group():
    # moveit_commander.roscpp_initializer(sys.argv)
    moveit_commander.robot()
    # print(dir(moveit_commander))
    #print(moveit_commander.__file__)
    rospy.init_node('move_group',anonymous=True)
    # robot = moveit_commander.robot()
    # SUSCRIBERS
    # PUBLISHERS
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        move_group()
    except rospy.ROSInterruptException:
        pass
