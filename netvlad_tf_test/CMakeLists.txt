cmake_minimum_required(VERSION 2.8.3)
project(netvlad_tf_test)

#find_package(catkin_simple REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
    message_generation 
    std_msgs 
    sensor_msgs 
    rospy
    rospy_tutorials
)


add_service_files(DIRECTORY srv FILES CompactImg.srv)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS 
    message_runtime 
    std_msgs 
    sensor_msgs 
    rospy
    rospy_tutorials
)





