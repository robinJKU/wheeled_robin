cmake_minimum_required(VERSION 2.8.3)
project(wheeled_robin_bringup)

find_package(catkin REQUIRED)

catkin_package(
)


install(DIRECTORY apps
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY concert
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY icons
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
