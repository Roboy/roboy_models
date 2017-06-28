#!/usr/bin/env python
__author__ = 'igor, camilo, schulze'

import time
import rospy
from std_msgs.msg import String, Float64, Header
from geometry_msgs.msg import Vector3, Point
import numpy as np
from pykbm.kinematics import KinematicsModel
from pykbm.polynomial import BezierModel
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import GetJointProperties, GetLinkState, GetModelState, SetModelState
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelState


class ArmExtController:

    # def angle_random_generator(self, n_angles, n_samples):
    #     sampled_angles = np.random.rand(n_angles, n_samples) * np.pi / 2 - np.pi / 4
    #     return sampled_angles

    def angle_random_generator(self, n_angles, n_samples):
        sampled_angles = np.random.rand(n_angles, n_samples) * np.pi / -2
        sampled_angles[0,0] = np.random.rand(1,1) * np.pi - np.pi / 2
        sampled_angles[2,0] = np.random.rand(1,1) * np.pi / 2
        angle_3_limit = np.pi / 2 - abs(sampled_angles[1,0])
        if sampled_angles[2,0] > angle_3_limit:
            sampled_angles[2,0] = angle_3_limit
        return sampled_angles

    def __init__(self):

        rospy.loginfo("Hi from external controller.")

        state_topic = '/HoLLiE/state'
        target_topic = '/HoLLiE/target'
        self.waiting_time = 0.5
        self.num_angles = 3
        self.world_reference_frame = 'world'
        self.state = "LEARNING"
        self.target_pos = None
        self.learned_model = None  # this will be KBM learning result
        #TODO: make sure you included leading '/'
        self.last_tcp_position = None

        sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.tcp_callback)

        robot_ns = '/robot'
        # robot_ns = '/HoLLiE'

        #self.marker_position = rospy.Publisher('/gazebo/set_model_state', ModelState)
        self.state_sub = rospy.Subscriber(state_topic, String, self.state_callback)
        self.target_sub = rospy.Subscriber(target_topic, Vector3, self.target_callback)
        arm_1_topic = robot_ns + '/hollie_real_left_arm_1_joint/cmd_pos'
        arm_2_topic = robot_ns + '/hollie_real_left_arm_2_joint/cmd_pos'
        arm_3_topic = robot_ns + '/hollie_real_left_arm_3_joint/cmd_pos'

        self.arm_1_pub = rospy.Publisher(arm_1_topic, Float64, queue_size=10)
        self.arm_2_pub = rospy.Publisher(arm_2_topic, Float64, queue_size=10)
        self.arm_3_pub = rospy.Publisher(arm_3_topic, Float64, queue_size=10)
        self.error_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        self.joint_angle_srv = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        self.link_srv = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        self.model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        #self.set_model_state_svr = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.kbm = KinematicsModel(np.pi / 4)


    def tcp_callback(self, models):
        # iterate through list of models in Gazebo to find hand object
        found_hand_object = None
        for i, model in enumerate(models.name):
            if 'real.007' in model:
                hand_pos = self.link_srv(model, self.world_reference_frame).link_state.pose.position
                #rospy.loginfo("hand_pos: {0}".format(hand_pos))
                self.last_tcp_position = np.array([hand_pos.x, hand_pos.y, hand_pos.z]).reshape(3,1)
                break


    def state_callback(self, state):
        self.state = state.data

    def target_callback(self, target):
        self.target_pos = target.data

    def move_marker(self, target_position):
        m = ModelState()
        m.model_name = 'marker_sphere'
        m.pose.orientation.x = m.pose.orientation.y = m.pose.orientation.z = 0
        m.reference_frame = self.world_reference_frame
        m.pose.position.x = target_position[0]
        m.pose.position.y = target_position[1]
        m.pose.position.z = target_position[2]

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state_svr = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_model_state_svr(m)
            # time.sleep(0.005)
            # resp = set_model_state_svr(m)
            # rospy.sleep(0.005)
            # resp = set_model_state_svr(m)
            print("----------- set_model staus: ", resp.status_message)
        except rospy.ServiceException, e:
            print("----------- Service call failed: %s"%e)

    def run(self):

        while True:

            if self.state == "LEARNING":

                angle_sample = self.angle_random_generator(self.num_angles, 1)
                rospy.loginfo("Publishing random angles: {0}".format(angle_sample))

                self.arm_1_pub.publish(Float64(angle_sample[0]))
                self.arm_2_pub.publish(Float64(angle_sample[1]))
                self.arm_3_pub.publish(Float64(angle_sample[2]))
                broadcast_time = rospy.Time.now()

                broadcast_time = rospy.Time.now()
                angle_sample = self.angle_random_generator(self.num_angles, 1)
                i = 0
                while self.state == "LEARNING":
                    curr_time = rospy.Time.now()
                    if (curr_time - broadcast_time).secs > self.waiting_time:
                        break
                    if self.last_tcp_position is not None:
                        #rospy.loginfo("Learning...")
                        # get joint angles
                        joint_3 = self.joint_angle_srv("hollie_real_left_arm_3_joint").position
                        joint_2 = self.joint_angle_srv("hollie_real_left_arm_2_joint").position
                        joint_1 = self.joint_angle_srv("hollie_real_left_arm_1_joint").position
                        kbm_input = np.array([joint_1, joint_2, joint_3]).reshape(3,1)
                        self.kbm.incremental(kbm_input, self.last_tcp_position, 1.0)

                    rospy.sleep(0.01)

                rospy.loginfo("Evaluating random crap...")
                angle_sample = self.angle_random_generator(self.num_angles, 1)
                kbm_predicted_tcp_position = self.kbm.evaluate(angle_sample)
                # summon sphere on this position: kbm_predicted_tcp_position
                rospy.loginfo("------ MARKER TARGET POSITION:\n%s" % kbm_predicted_tcp_position)
                self.move_marker(kbm_predicted_tcp_position)
                # self.move_marker(self.last_tcp_position)
                # validate position of marker
                marker_true_position = self.model_srv('marker_sphere', self.world_reference_frame).pose.position

                rospy.loginfo("------ MARKER TRUE POSITION:\n%s" % marker_true_position)
                rospy.loginfo("------ KBM PREDICTED POSITION:\n%s" % kbm_predicted_tcp_position)

                self.arm_1_pub.publish(Float64(angle_sample[0]))
                self.arm_2_pub.publish(Float64(angle_sample[1]))
                self.arm_3_pub.publish(Float64(angle_sample[2]))
                broadcast_time = rospy.Time.now()

                while self.state == "LEARNING":


                    curr_time = rospy.Time.now()
                    if (curr_time - broadcast_time).secs > self.waiting_time:

                        # get joint angles
                        joint_3 = self.joint_angle_srv("hollie_real_left_arm_3_joint").position
                        joint_2 = self.joint_angle_srv("hollie_real_left_arm_2_joint").position
                        joint_1 = self.joint_angle_srv("hollie_real_left_arm_1_joint").position
                        kbm_input = np.array([joint_1, joint_2, joint_3]).reshape(3,1)

                        # kbm_predicted_tcp_position = self.kbm.evaluate(kbm_input)
                        rmse = self.kbm.test(kbm_input, self.last_tcp_position)
                        
                        # print("Current tcp: {0}".format(self.last_tcp_position))
                        # print("Predicted tcp: {0}".format(kbm_predicted_tcp_position))
                        # print("Computed error: {0}".format(rmse))

                        # publish error to topic for plotting in UI
                        msg = JointState()
                        msg.header = Header()
                        msg.header.stamp = rospy.Time.now()
                        msg.name.append("KBM error")
                        msg.position.append(rmse)
                        msg.velocity.append(0.0)
                        msg.effort.append(0.0)
                        self.error_pub.publish(msg)
                        break


                    # rospy.sleep(0.01)

            if self.state == "EXECUTING":

                angle = -1.57
                # self.arm_1_pub.publish(Float64(angle))

                if self.target_pos is not None:
                    # do inverse kinematics from learned model
                    # broadcast to low level controllers
                    pass

                pass

            rospy.sleep(0.01)


if __name__ == '__main__':
    rospy.init_node('arm_ext_controller')

    contrl = ArmExtController()
    contrl.run()

    rospy.spin()

