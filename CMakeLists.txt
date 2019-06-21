cmake_minimum_required(VERSION 2.8.3)
project(controlers)
find_package(catkin REQUIRED 
  roscpp 
  rospy 
  std_msgs 
  geometry_msgs
#  message_generation
  master_msgs
)


#add_service_files(
#  FILES
#  enable.srv
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS 
#  message_runtime
  roscpp
  rospy
  std_msgs
  geometry_msgs
  master_msgs
)




