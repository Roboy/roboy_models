rosrun collada_urdf urdf_to_collada model.urdf model.dae 
openrave-robot.py model.dae --info links
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=model.dae --iktype=translation3d --baselink=0 --eelink=3 --savefile=ikfast.cpp

