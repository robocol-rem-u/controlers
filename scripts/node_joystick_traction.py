#!/usr/bin/env python
import rospy
from master_msgs.msg import traction_Orders


def node_joystick_traction():

    rospy.init_node('node_joystick_traction',anonymous=True)
    

    #publica en el topico traction orders
    pub_Traction_Orders = rospy.Publisher('topic_traction_orders', traction_Orders, queue_size=10)
    rate = rospy.Rate (10)
    while not rospy.is_shutdown ():
        rate.sleep ()


if __name__ == '__main__':
    try:
       
        node_joystick_traction()
        rate = rospy.Rate (10)

    except rospy.ROSInterruptException:
        pass
