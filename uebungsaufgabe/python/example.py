#!/usr/bin/env python2
import time

from CoppeliaSimConnector import CoppeliaSimConnector
from IIwaKinematics import IIwaKinematics
from TensorflowModel import TFmodel
import PyKDL
import numpy as np

import matplotlib.pyplot as plt

from cv_bridge import CvBridge, CvBridgeError

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

connector = CoppeliaSimConnector()
kin = IIwaKinematics(connector)
tfm = TFmodel()


def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z))


def sensor_callback(data):
    global objectDetected
    objectDetected = data.data


def image_callback(image_msg):
    global imageRecieved
    global image
    image = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    plt.imshow(image)
    imageRecieved = True


objectDetected = False
imageRecieved = False
sensor = rospy.Subscriber('/objectDetected', Bool, sensor_callback)
gripper = rospy.Publisher('/gripperClosing', Bool, queue_size=2, latch=False)
image = rospy.Subscriber('/cameraVision', Image, image_callback)

rot = PyKDL.Rotation()
rot.DoRotX(-np.pi)

while True:
    kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(0.255, -0.4755, 0.19)), 1.0)
    kin.lin(PyKDL.Frame(rot, PyKDL.Vector(0.255, -0.4755, 0.17)), 1.0)

    while not objectDetected:
        connector.step()
    x = 0
    while not imageRecieved and x < 25:
        connector.step()
        x = x+1
    imageRecieved = False

    kin.lin(PyKDL.Frame(rot, PyKDL.Vector(0.255, -0.4755, 0.13)), 1.0)
    gripper.publish(True)
    connector.step()

    predictions = tfm.predict_from_image(image)
    print(predictions)
    image_type = np.argmax(predictions)

    # Middle Sink: kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(0.25, -0.48, 0.225)), 1.0)
    # Left Sink:
    # Right Sink:
    if image_type == 0:   # Nagel
        i = 1 # Platzhalter
        print("Nagel")
        # TODO nail_movement
        kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(0.6, 0.45, 0.25)), 0.8)
        gripper.publish(False)

    if image_type == 1:   # Schraube
        i = 1
        print("Schraube")
        # TODO screw_movement
        kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(0.35, 0.25, 0.55)), 0.8)
        kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(0.25, 0.45, 0.45)), 0.8)
        gripper.publish(False)

    if image_type == 2:   # Scheibe
        i = 1
        print("Scheibe")
        # TODO shim_movement
        kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(-0.45, -0.25, 0.45)), 0.8)
        kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(-0.45, 0.15, 0.45)), 0.8)
        gripper.publish(False)

    kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(0.255, -0.4755, 0.225)), 1.0)
