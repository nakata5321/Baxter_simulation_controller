#!/usr/bin/env python3
# rospy - ROS Python API
import rospy
import baxter_interface


def hello_baxter():
    # initialize our ROS node, registering it with the Master

    # get the right limb's current joint angles
    left_arm = baxter_interface.limb.Limb("left")
    right_arm = baxter_interface.limb.Limb("right")
    # left_joint_angels = left_arm.joint_angles()
    # right_joint_angels = right_arm.joint_angles()

    baxter_interface.RobotEnable().enable()

    rospy.loginfo("Moving to neutral pose...")
    left_arm.move_to_neutral()
    right_arm.move_to_neutral()

    # store the first wave position
    right_wave_1 = {
        "right_s0": -0.459,
        "right_s1": -0.202,
        "right_e0": 1.807,
        "right_e1": 1.714,
        "right_w0": -0.906,
        "right_w1": -1.545,
        "right_w2": -0.276,
    }
    left_wave_1 = {
        "left_s0": -0.459,
        "left_s1": -0.202,
        "left_e0": 1.807,
        "left_e1": 1.714,
        "left_w0": -0.906,
        "left_w1": -1.545,
        "left_w2": -0.276,
    }

    # store the second wave position
    right_wave_2 = {
        "right_s0": -0.395,
        "right_s1": -0.202,
        "right_e0": 1.831,
        "right_e1": 1.981,
        "right_w0": -1.979,
        "right_w1": -1.100,
        "right_w2": -0.448,
    }
    left_wave_2 = {
        "left_s0": -0.395,
        "left_s1": -0.202,
        "left_e0": 1.831,
        "left_e1": 1.981,
        "left_w0": -1.979,
        "left_w1": -1.100,
        "left_w2": -0.448,
    }

    # wave three times
    rospy.loginfo("Hello")
    for _move in range(2):
        left_arm.move_to_joint_positions(left_wave_1)
        right_arm.move_to_joint_positions(right_wave_1)
        left_arm.move_to_joint_positions(left_wave_2)
        right_arm.move_to_joint_positions(right_wave_2)
