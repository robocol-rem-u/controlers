#!/usr/bin/env python3
import rospy,math
from sensor_msgs.msg import JointState
from master_msgs.msg import traction_Orders,arm_Orders,arm_auto_status
from sensor_msgs.msg import Joy
from std_msgs.msg import Header
from std_msgs.msg import Empty
from moveit_msgs.msg import RobotState
from actionlib_msgs.msg import GoalStatusArray
from moveit_msgs.msg import MoveGroupActionFeedback

joints = JointState()
arm_msg = arm_Orders()
order = traction_Orders()

butNam = ['A','B','X','Y','LB','RB','WIN','MENU','PA1','PA2']
butts = []
axs = []

vel = 40

aj = 0
j = ['B','C','D','E','F','G']

rest = [-0.931,-1.57,2.1871,0.1311,-1.2,0.3565]
#pick = [-1.124,-1.57,-0.3449,3.2422,-1.4288,-1.7818]
pick = [-0.931,-1.57,-0.11,0.2025,1.24,0.0]
ready = [-0.931,-1.57,2.1831,0.99,-1.47,-0.36]

press = 0
planned = 0

# Referencia del objeto del joystick
joystick_ref = None
# Referenc de eventos generados de eje Movido:
axis_moved = False
# Referencia de eje #0,#1,#2,#3 del joystick
axis0,axis1,axis2,axis3 = 0,0,0,0
# Maximas rpm de las llantas
max_rpm = 70

## --- CALLBACKS --- ##
def callback_control(data):
    global butts,axs,press
    axs = data.axes
    butts = data.buttons
    press = 1
def callback_status(data):
    global planned
    try:
        t = data.status_list[0].status
        # if t == 1:
        #     print('NEW')
        if t == 3 and planned == 0:
             planned = 1
        # print(t)
    except:
        pass
# Joints State Callback.
def joints_Callback(param):
  global joints
  joints = param.position
def callback_feedback(data):
    global planned
    s = data.feedback.state 
    if s == 'PLANNING':
        print('Planeando...')
        planned = 0
    elif s == 'IDLE':
        print('Terminó Planeación.')
        planned = 1

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

def load_pose(pose):
    p = RobotState()
    p.joint_state.header = Header()
    js = []
    for i in range(1,7):
        js.append('robocol_joint'+str(i))
    p.joint_state.name = js
    p.joint_state.position = pose
    p.joint_state.velocity = [0.0]
    p.joint_state.effort = [0.0]
    return p

def node_xbox_controller_arm():
    global press,planned
    global rest,pick,ready
    global joints
    rospy.init_node('node_xbox_controller_arm', anonymous=True)
    # --- SUBSCRIBE --- #
    rospy.Subscriber("joy",Joy,callback_control)
    rospy.Subscriber('move_group/feedback',MoveGroupActionFeedback,callback_feedback)
    rospy.Subscriber('joint_states',JointState,joints_Callback)
    # rospy.Subscriber('/move_group/status',GoalStatusArray,callback_status)
    #rospy.Subscriber('/rviz/moveit/update_custom_goal_state',RobotState,call)
    # --- PUBLISH --- #
    pub_traction_orders = rospy.Publisher('topic_traction_orders', traction_Orders, queue_size=10)
    pub_Arm_Orders = rospy.Publisher('topic_arm_orders',arm_Orders, queue_size=10)
    pub_plan = rospy.Publisher('/rviz/moveit/plan',Empty, queue_size=10)
    pub_exec = rospy.Publisher('/rviz/moveit/execute',Empty, queue_size=10)
    pub_goal = rospy.Publisher('/rviz/moveit/update_custom_goal_state',RobotState,queue_size=10)
    pub_start = rospy.Publisher('/rviz/moveit/update_custom_start_state',RobotState,queue_size=10)
    pub_Arm_Status = rospy.Publisher('topic_arm_status',arm_auto_status,queue_size=10)
    rate = rospy.Rate(100)
    print(' ')
    print('Init XboX Controller...')
    print(' ')
    end = 0
    while not rospy.is_shutdown():
        if press:
            msg = identify()

            # Uncomment to move rover with sticks.
            # left,rigth = move_rover()
            # order.rpm_r, order.rpm_l = rigth, left
            # order.header.stamp = rospy.Time.now()
            # order.header.seq = order.header.seq + 1
            # order.sensibility = int(max_rpm*(-axis3+1)/2)
            # pub_traction_orders.publish(order)


            if(msg=='A'):
                planned = 0
                print("Going to rest pose...")
                s = load_pose(joints)
                g = load_pose(rest)
                pub_start.publish(s)
                pub_goal.publish(g)
                pub_plan.publish(Empty())
            if(msg=='X'):
                planned = 0
                print("Going to ready pose...")
                s = load_pose(joints)
                g = load_pose(ready)
                pub_start.publish(s)
                pub_goal.publish(g)
                pub_plan.publish(Empty())
            if(msg=='Y'):
                planned = 0
                print("Going to pick pose...")
                s = load_pose(joints)
                g = load_pose(pick)
                pub_start.publish(s)
                pub_goal.publish(g)
                pub_plan.publish(Empty())
            if(msg=='B'):
                planned = 0
                print('Stopping motors...')
                st = arm_auto_status()
                st.header = Header()
                st.message = 'Stop'
                pub_Arm_Status.publish(st)
            if(msg=='MENU'):
                print('Quiting auto node...')
                st = arm_auto_status()
                st.header = Header()
                st.message = 'Quit'
                pub_Arm_Status.publish(st)
            if(msg=='WIN'):
                print('Executing movement...')
                st = arm_auto_status()
                st.header = Header()
                st.message = 'Exec'
                pub_exec.publish(Empty())
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
            press = 0
        if planned:
            print('Ejecutando movimiento...')
            st = arm_auto_status()
            st.header = Header()
            st.message = 'Execute'
            pub_Arm_Status.publish(st)
            planned = 0
        rate.sleep()

if __name__ == '__main__':
    try:
        node_xbox_controller_arm()
    except rospy.ROSInterruptException:
        pass
