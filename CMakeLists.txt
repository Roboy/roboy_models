cmake_minimum_required(VERSION 2.8.3)
project(roboy_models)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY
        arm/
        legs_with_muscles_simplified/
        plate_with_muscle/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )