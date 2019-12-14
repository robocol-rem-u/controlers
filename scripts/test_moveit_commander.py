#!/usr/bin/env python2
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def cartesianPaths(move_group):
    scale = 1.0
    waypoints = []
    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 2  # First move up (z)
    # wpose.position.y += scale * 1  # and sideways (y)
    # waypoints.append(copy.deepcopy(wpose))
    # wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    # waypoints.append(copy.deepcopy(wpose))
    # wpose.position.y -= scale * 0.1  # Third move sideways (y)
    # waypoints.append(copy.deepcopy(wpose))
    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(waypoints,0.01,0.0)
    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robocol_commander',anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "robocol_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    # SUSCRIBERS
    # PUBLISHERS
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
    rate = rospy.Rate(100)
    # print(robot.get_current_state())

    while not rospy.is_shutdown():
        # box_pose = geometry_msgs.msg.PoseStamped()
        # box_pose.header.frame_id = "robocol_hand"
        # box_pose.pose.orientation.w = 1.0
        # box_pose.pose.position.z = 2 # slightly above the end effector
        # box_name = "box"
        # scene.add_box(box_name, box_pose, size=(1,1,1))
        # print('box')
        plan = cartesianPaths(move_group)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # display_trajectory_publisher.publish(display_trajectory)
        # print(plan)
        # move_group.execute(plan,wait=True)
        # print('disp')
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
