cmake_minimum_required(VERSION 2.8.3)
project(roboy_models)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY
        arm_simplified/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/arm_simplified
        )
install(DIRECTORY
        legs_with_muscles_simplified/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/legs_with_muscles_simplified/
        )
install(DIRECTORY
        legs_with_upper_body_simplified/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/legs_with_upper_body_simplified/
        )
install(DIRECTORY
        legs_with_upper_body_with_camera_simplified/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/legs_with_upper_body_with_camera_simplified/
        )
install(DIRECTORY
        makeHuman_avatar/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/makeHuman_avatar
        )
install(DIRECTORY
        myoarm_small/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/myoarm_small
        )
install(DIRECTORY
        PaBiRoboy_demo_simplified/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/PaBiRoboy_demo_simplified
        )
install(DIRECTORY
        PaBiRoboy_simplified/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/PaBiRoboy_simplified
        )
install(DIRECTORY
        plate_with_muscle/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plate_with_muscle
        )
install(DIRECTORY
        Roboy_simplified/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/Roboy_simplified
        )



install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

