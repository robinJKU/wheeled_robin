cmake_minimum_required(VERSION 2.8.3)
project(wheeled_robin_node)

find_package(catkin REQUIRED COMPONENTS wheeled_robin_driver
                                        diagnostic_msgs
                                        dynamic_reconfigure
                                        geometry_msgs
                                        message_generation
                                        nav_msgs
                                        rospy
                                        tf)

catkin_python_setup()

add_message_files(DIRECTORY msg
                  FILES WheeledRobinSensorState.msg)
                        
add_service_files(DIRECTORY srv
                  FILES SetDigitalOutputs.srv
                        SetWheeledRobinMode.srv)

generate_messages(DEPENDENCIES diagnostic_msgs
                               geometry_msgs
                               nav_msgs)
                               
generate_dynamic_reconfigure_options(cfg/WheeledRobin.cfg)

catkin_package(CATKIN_DEPENDS diagnostic_msgs
                              geometry_msgs
                              message_runtime
                              nav_msgs)

install(PROGRAMS
                nodes/wheeled_node.py
		nodes/test_driver.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
