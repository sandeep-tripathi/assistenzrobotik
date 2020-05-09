#!/usr/bin/env python2
import time

from CoppeliaSimConnector import CoppeliaSimConnector
from IIwaKinematics import IIwaKinematics
import PyKDL
import numpy as np

import rospy
from std_msgs.msg import Bool

connector = CoppeliaSimConnector()
kin = IIwaKinematics(connector)


def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z))


def sensor_callback(data):
    global objectDetected
    objectDetected = data.data


objectDetected = False
sensor = rospy.Subscriber('/objectDetected', Bool, sensor_callback)
gripper = rospy.Publisher('/gripperClosing', Bool, queue_size=2, latch=False)
i=0

while True:
    if i == 5:
        gripper.publish(True)
    if i >= 10:
        gripper.publish(False)
        i = 0
    i = i + 1

    rot = PyKDL.Rotation()
    rot.DoRotX(-np.pi)

    kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(0.5, -0.5, 0.5)))

    kin.lin(PyKDL.Frame(rot, PyKDL.Vector(0.0, -0.5, 0.5)), 1)

    rot.DoRotZ(+np.pi)

    kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(0.5, 0, 0.5)))
