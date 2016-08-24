cmake_minimum_required(VERSION 2.8.3)
project(roboy_models)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY
        legs_with_muscles_simplified/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/legs_with_muscles_simplified
        )