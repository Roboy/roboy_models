#!/bin/bash
export MYROBOT_NAME="roboy_xylophone_left_arm"
export PLANNING_GROUP="hand_left"
export IKFAST_PRECISION="3"
export IKFAST_OUTPUT_PATH=`pwd`/ikfast61_"$PLANNING_GROUP".cpp
export BASE_LINK="0"
#export FREE_INDEX="4"
export EEF_LINK="6"
export MOVEIT_IK_PLUGIN_PKG="$MYROBOT_NAME"_ikfast_"$PLANNING_GROUP"_plugin

rosrun collada_urdf urdf_to_collada model.urdf "$MYROBOT_NAME".dae
rosrun moveit_kinematics round_collada_numbers.py "$MYROBOT_NAME".dae "$MYROBOT_NAME".dae "$IKFAST_PRECISION"
openrave-robot.py "$MYROBOT_NAME".dae --info links
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot="$MYROBOT_NAME".dae --iktype=TranslationDirection5D --baselink=0 --eelink=6 --savefile="$IKFAST_OUTPUT_PATH"
catkin_create_pkg "$MOVEIT_IK_PLUGIN_PKG"
catkin_make
