cmake_minimum_required(VERSION 2.8.3)
project(roboy_models)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY
        arm/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/arm
        )
install(DIRECTORY
        legs_with_muscles_simplified/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/legs_with_muscles_simplified/
        )
install(DIRECTORY
        legs_with_upper_body/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/legs_with_upper_body/
        )
install(DIRECTORY
        plate_with_muscle/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plate_with_muscle
        )
install(DIRECTORY
        Roboy/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/Roboy
        )
install(DIRECTORY
        Roboy_simplified/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/Roboy_simplified
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

