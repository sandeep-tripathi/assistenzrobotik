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

rot = PyKDL.Rotation()
rot.DoRotX(-np.pi)

while True:
    kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(0.25, -0.48, 0.19)), 1.0)
    kin.lin(PyKDL.Frame(rot, PyKDL.Vector(0.25, -0.48, 0.13)), 1.0)

    while not objectDetected:
        connector.step()
    gripper.publish(True)
    for i in xrange(2):
        connector.step()

    # speed is not working with lin() ?!

    # Standalone sink
    #kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(0.25, -0.48, 0.225)), 0.8)
    #kin.lin(PyKDL.Frame(rot, PyKDL.Vector(0.5, 0.45, 0.25)), 1.0)
    # Sink at the end of the conveyor
    kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(0.25, -0.48, 0.225)), 0.8)
    kin.lin(PyKDL.Frame(rot, PyKDL.Vector(0.0, -0.48, 0.225)), 1.0)

    gripper.publish(False)

