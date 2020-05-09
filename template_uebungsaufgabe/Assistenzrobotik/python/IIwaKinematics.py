#!/usr/bin/env python2
import math

import matplotlib.pyplot as plt
import numpy as np
import PyKDL
from scipy.spatial.transform import Rotation as SciRot
from scipy.spatial.transform import Slerp

import rospy
from sensor_msgs.msg import JointState

from AxisTrajectoryGenerator import AxisTrajectoryGenerator
from CartesianTrajectoryGenerator import CartesianTrajectoryGenerator
from KdlHelpers import *


class IIwaKinematics:
    def __init__(self, copeliaSimConnector):
        self.max_speeds = np.deg2rad(np.array([85, 85, 100, 75, 130, 135, 135]))
        self.max_accels = np.array([5, 4, 5, 5, 25, 12, 25])
        self.min_angles_ = np.deg2rad(np.array([-170.0, -120.0, -170.0, -120.0, -170.0, -120.0, -175.0]))
        self.max_angles_ = np.deg2rad(np.array([170.0, 120.0, 170.0, 120.0, 170.0, 120.0, 175.0]))

        self.max_lin_speed = 1
        self.max_lin_accel = 4
        self.ptp_traj_gen = AxisTrajectoryGenerator(self.max_speeds, self.max_accels)
        self.lin_traj_gen = CartesianTrajectoryGenerator(self.max_lin_speed, self.max_lin_accel)
        self.copeliaSimConnector = copeliaSimConnector

        self.chain = PyKDL.Chain()

        # link 0
        self.chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotZ), PyKDL.Frame().DH(0, -np.pi / 2, 0.36, 0)))
        # from KS1 to KS2
        self.chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotZ), PyKDL.Frame().DH(0, np.pi / 2, 0, 0)))
        # from KS2 to KS3
        self.chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotZ), PyKDL.Frame().DH(0, np.pi / 2, 0.42, 0)))
        # from KS3 to KS4
        self.chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotZ), PyKDL.Frame().DH(0, -np.pi / 2, 0, 0)))
        # from KS4 to KS5
        self.chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotZ), PyKDL.Frame().DH(0, -np.pi / 2, 0.4, 0)))
        # from KS4 to KS5
        self.chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotZ), PyKDL.Frame().DH(0, np.pi / 2, 0, 0)))
        # to tool
        self.chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotZ), PyKDL.Frame().DH(0, 0, 0.126, 0)))

        # Forward kinematic solver
        self.FKSolver = PyKDL.ChainFkSolverPos_recursive(self.chain)
        # Inverse kinematic solver
        self.IKSolver = PyKDL.ChainIkSolverPos_LMA(self.chain)

        self.current_joint_poses = None
        self.current_joint_velocities = None
        rospy.Subscriber("/jointStates", JointState, self.joint_callback)

        self.joint_pub = rospy.Publisher('/jointTargtes', JointState, queue_size=10, latch=True)
        self.copeliaSimConnector.step()
        self.copeliaSimConnector.step()
        self.safety_ok = True

    def check_joint_poses(self, joint_poses):
        for i in range(0, len(joint_poses)):
            dist = abs(joint_poses[i] - self.min_angles_[i])
            if (dist < 0.01):
                print("Warning axis " + str(i) + " close to lower limit")

            dist = abs(joint_poses[i] - self.max_angles_[i])
            if (dist < 0.01):
                print("Warning axis " + str(i) + " close to upper limit")

        for i in range(0, len(joint_poses)):
            if joint_poses[i] < self.min_angles_[i]:
                print("Warning axis " + str(i) + " is out of range")
                return False

            if joint_poses[i] > self.max_angles_[i]:
                print("Warning axis " + str(i) + " is out of range")
                return False
        return True

    def joint_callback(self, data):
        old_joint_poses = self.current_joint_poses
        self.current_joint_poses = data.position

        if old_joint_poses is not None:
            self.current_joint_velocities = (np.asarray(self.current_joint_poses) - np.asarray(old_joint_poses)) \
                                            / self.copeliaSimConnector.get_dt()

    def _publish_joint_pose(self, msg):
        while not self.safety_ok:
            self.copeliaSimConnector.step()

        self.check_joint_poses(msg.position)
        self.joint_pub.publish(msg)
        self.copeliaSimConnector.step()

    def safety_stop(self, state):
        self.safety_ok = not state

    def get_current_cartesian_tcp_pose(self):
        current_pose = PyKDL.Frame()
        self.FKSolver.JntToCart(list_to_jntArray(self.current_joint_poses), current_pose)
        return current_pose

    def ptp(self, dest_frame, speed=1.0):
        if speed > 1:
            speed = 1.0

        target_joint_poses = PyKDL.JntArray(self.chain.getNrOfJoints())
        start_config = list_to_jntArray(self.current_joint_poses)
        ret = self.IKSolver.CartToJnt(start_config, dest_frame, target_joint_poses)
        if ret != 0:
            print("Warning no IK solution found")
            exit(-1)

        start_pose = np.array(self.current_joint_poses)
        goal_pose = np.asarray(jntArray_to_list(target_joint_poses))
        time_array, poses = self.ptp_traj_gen.genSyncMove(start_pose, goal_pose, self.copeliaSimConnector.get_dt(),
                                                          speed)

        msg_list = []

        for i in xrange(time_array.shape[0]):
            msg = JointState()
            for joint in poses:
                msg.position.append(joint[i])
            msg_list.append(msg)

        for msg in msg_list:
            self._publish_joint_pose(msg)

    def lin(self, dest_frame, speed=1.0):
        if speed > 1:
            speed = 1.0

        def simple_ptp(dest_frame):
            target_joint_poses = PyKDL.JntArray(self.chain.getNrOfJoints())
            start_config = list_to_jntArray(self.current_joint_poses)
            ret = self.IKSolver.CartToJnt(start_config, dest_frame, target_joint_poses)
            if ret != 0:
                print("Warning no IK solution found")
                exit(-1)

            msg = JointState()
            msg.position = jntArray_to_list(target_joint_poses)
            self._publish_joint_pose(msg)

        current_pose = self.get_current_cartesian_tcp_pose()
        current_quat = current_pose.M.GetQuaternion()
        goal_quat = dest_frame.M.GetQuaternion()
        quat = SciRot.from_quat(quat=[current_quat, goal_quat], normalized=True)

        dt = self.copeliaSimConnector.sim_time_step
        start_pose3 = np.array([current_pose.p[0], current_pose.p[1], current_pose.p[2]])
        end_pose3 = np.array([dest_frame.p[0], dest_frame.p[1], dest_frame.p[2]])
        time, poses3 = self.lin_traj_gen.genMove(start_pose3, end_pose3, dt, speed)

        key_times = [0, time[-1]]
        slerp = Slerp(key_times, quat)

        interp_rots = slerp(time)

        frames = []
        for i in range(len(poses3)):
            rot_matrix = interp_rots.as_euler('zyx')[i]
            rot = PyKDL.Rotation.EulerZYX(rot_matrix[0], rot_matrix[1], rot_matrix[2])
            frame = PyKDL.Frame(rot, PyKDL.Vector(poses3[i][0], poses3[i][1], poses3[i][2]))
            frames.append(frame)

        for frame in frames:
            simple_ptp(frame)
