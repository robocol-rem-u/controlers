#!/usr/bin/env python3
import rospy,math
from master_msgs.msg import traction_Orders,arm_Orders
from sensor_msgs.msg import Joy
from std_msgs.msg import Header


arm_msg = arm_Orders()
order = traction_Orders()

butNam = ['A','B','X','Y','LB','RB','WIN','MENU','PA1','PA2']
butts = []
axs = []

vel = 40

aj = 0
j = ['B','C','D','E','F','G']

press = 0

# Referencia del objeto del joystick
joystick_ref = None
# Referenc de eventos generados de eje Movido:
axis_moved = False
# Referencia de eje #0,#1,#2,#3 del joystick
axis0,axis1,axis2,axis3 = 0,0,0,0
# Maximas rpm de las llantas
max_rpm = 70


def callback(data):
    global butts,axs,press
    axs = data.axes
    butts = data.buttons
    press = 1

def identify():
    global butts,axs,press
    l = list(butts)
    if axs[4] == -1:
        return 'C'
    if axs[5] == -1:
        return 'O'
    for i in range(len(l)):
        if l[i] == 1:
            return butNam[i]
    else:
        return ''

def steering(x, y, sensibilidad_rcv):
    # Convierte a polar
    r = math.hypot(-x,-y)
    t = math.atan2(-y,-x)
    # Rota 45 grados
    t += math.pi / 4
    # Retorna a cartesianas
    left = r * math.cos(t)
    right = r * math.sin(t)
    # Reescala a nuevas coordenadas
    left = left * math.sqrt(2)
    right = right * math.sqrt(2)
    # clamp to -1/+1
    left = max(min(left, 1), -1)
    right = max(min(right, 1), -1)
    r1 = int(sensibilidad_rcv*left)
    l1 = int(sensibilidad_rcv*right)
    return r1,l1

def move_rover():
    global axs
    axis0 = axs[0]
    axis1 = axs[1]
    axis2 = axs[2]
    axis3 = axs[3]
    if axis2 == 0:
        left, rigth = steering(axis1, axis0, max_rpm*(-axis3+1)/2)
    else:
        left, rigth = int(((max_rpm+20)*(-axis3+1)/2)*axis2), int(-((max_rpm+20)*(-axis3+1)/2)*axis2)
    return left,rigth

def node_xbox_controller():
    global butts,axs,press,vel,max_rpm
    global arm_msg
    global j,aj
    global joystick_ref, axis_moved, axis0, axis1
    rospy.init_node('node_xbox_controller', anonymous=True)
    rospy.Subscriber("joy",Joy,callback)
    pub_traction_orders = rospy.Publisher('topic_traction_orders', traction_Orders, queue_size=10)
    pub_Arm_Orders = rospy.Publisher('topic_arm_orders',arm_Orders, queue_size=10)

    print(' ')
    print('Init XboX Controller...')
    print('Actual Joint:',aj)
    print(' ')
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if press:
            msg = identify()
            #print(axs[4])
            left,rigth = move_rover()
            order.rpm_r, order.rpm_l = rigth, left
            order.header.stamp = rospy.Time.now()
            order.header.seq = order.header.seq + 1
            order.sensibility = int(max_rpm*(-axis3+1)/2)
            pub_traction_orders.publish(order)
            #print(l1)
            # axis0 = axs[0]
            # axis1 = axs[1]
            # axis2 = axs[2]
            # axis3 = axs[3]
            # if axis2 == 0:
            #     left, rigth = steering(axis1*1024, axis0*1024, max_rpm*(-axis3+1)/2)
            # else:
            #     left, rigth = int((max_rpm*(-axis3+1)/2)*axis2), int(-(max_rpm*(-axis3+1)/2)*axis2)
            #     print(left)
            # print(left)
            # print(rigth)
            if msg != '':
                print('Press:',msg)

                if(msg=='Y'):
                    print('Changing Actual Joint...')
                    if aj < 5:
                        aj += 1
                    else:
                        aj = 0
                    print('Actual Joint:',aj)
                if(msg=='A'):
                    print('Changing Actual Joint...')
                    if aj > 0:
                        aj -= 1
                    else:
                        aj = 5
                    print('Actual Joint:',aj)
                if msg == 'C':
                    print('Closing claw...')
                    arm_msg.header = Header()
                    m = 'S0#'
                    arm_msg.message = m
                    pub_Arm_Orders.publish(arm_msg)
                    print("Closed.")
                if msg == 'O':
                    print('Opening claw...')
                    arm_msg.header = Header()
                    m = 'S255#'
                    arm_msg.message = m
                    pub_Arm_Orders.publish(arm_msg)
                    print("Opened.")
                if(msg=='LB'):
                    print('Moving joint',str(j[aj]),'to CW.')
                    arm_msg.header = Header()
                    m = str(j[aj]) + str(vel) + '#'
                    arm_msg.message = m
                    pub_Arm_Orders.publish(arm_msg)
                if(msg=='RB'):
                    print('Moving joint',str(j[aj]),'to CCW.')
                    arm_msg.header = Header()
                    m = str(j[aj]) + str(vel) + '!'
                    arm_msg.message = m
                    pub_Arm_Orders.publish(arm_msg)
                if(msg=='X'):
                    print('Stopping joint',str(j[aj]))
                    arm_msg.header = Header()
                    m = str(j[aj]) + '0#'
                    arm_msg.message = m
                    pub_Arm_Orders.publish(arm_msg)
                    print('Stop joint',str(j[aj]))
                if(msg=='B'):
                    print('Stopping All Motors...')
                    arm_msg.header = Header()
                    arm_msg.message = 'B0#C0#D0#E0#F0#G0#'
                    pub_Arm_Orders.publish(arm_msg)
                    print('Published STOP!')
                print(' ')
            press = 0
        rate.sleep()

if __name__ == '__main__':
    try:
        node_xbox_controller()
    except rospy.ROSInterruptException:
        pass
