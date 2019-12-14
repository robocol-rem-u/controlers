#!/usr/bin/env python3

import rospy,math
from std_msgs.msg import Header
from moveit_msgs.msg import RobotState

robot_state = RobotState()

class Vector3:
  def __init__(self, x, y, z):
    self.x = x
    self.y = y
    self.z = z
    # Used for debugging. This method is called when you print an instance  
    def __str__(self):
        return "(" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) + ")"
    def get_length(self):
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

def goal_state_callback(param):
    print(' ')
    print(param.multi_dof_joint_state)
    print(' ')

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

def load_pose_2(pose):
    p = RobotState()
    p.multi_dof_joint_state.header = Header()
    js = []
    for i in range(1,7):
        js.append('robocol_joint'+str(i))
    p.multi_dof_joint_state.joint_names = js
    # p.multi_dof_joint_state[0].transforms.translations.x = 0.0
    # p.multi_dof_joint_state.transforms = [0.0,0.0,0.0]
    v1 = Vector3(1, 3, 5)
    # print(len(p.multi_dof_joint_state.transforms))
    # p.multi_dof_joint_state.transforms = [0.0,0.0,0.0]
    # p.multi_dof_joint_state = [0.0,0.0,0.0]
    # p.joint_state.velocity = [0.0]
    # p.joint_state.effort = [0.0]
    return p

def inverse_kinematics():
    rospy.init_node('inverse_kinematics',anonymous=True)
    # SUSCRIBERS
    rospy.Subscriber('/rviz/moveit/update_custom_goal_state',RobotState,goal_state_callback)
    # PUBLISHERS
    pub_start = rospy.Publisher('/rviz/moveit/update_custom_goal_state',RobotState,queue_size=10)
    rate = rospy.Rate(100)
    joints = [0.0,0.0,0.0,0.0,0.0,0.0]
    while not rospy.is_shutdown():
        s = load_pose_2(joints)
        pub_start.publish(s)
        rate.sleep()

if __name__ == '__main__':
    try:
        inverse_kinematics()
    except rospy.ROSInterruptException:
        pass
