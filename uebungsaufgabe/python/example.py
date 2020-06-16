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
from std_msgs.msg import Int8
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
    #plt.imshow(image)
    imageRecieved = True


def producer_callback(type):
    global prod_list
    prod_list.append(type.data)


def differences(a, b):
    if len(a) != len(b):
        a = a[0:len(b)]
    return sum(i != j for i, j in zip(a, b))


objectDetected = False
imageRecieved = False
sub_sensor = rospy.Subscriber('/objectDetected', Bool, sensor_callback)
gripper = rospy.Publisher('/gripperClosing', Bool, queue_size=2, latch=False)
sub_image = rospy.Subscriber('/cameraVision', Image, image_callback)
sub_producer = rospy.Subscriber('/True_Class', Int8, producer_callback)

rot = PyKDL.Rotation()
rot.DoRotX(-np.pi)

pred_list = []
prod_list = []

while True:
    kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(0.255, -0.4755, 0.19)), 1.0)
    kin.lin(PyKDL.Frame(rot, PyKDL.Vector(0.255, -0.4755, 0.17)), 1.0)

    while not imageRecieved:
        connector.step()

    kin.lin(PyKDL.Frame(rot, PyKDL.Vector(0.255, -0.4755, 0.13)), 1.0)
    gripper.publish(True)

    #plt.imshow(image)
    #plt.show()
    predictions = tfm.predict_from_image(image)
    print(predictions)
    image_type = np.argmax(predictions)

    if image_type == 0:   # Nagel
        i = 1 # Platzhalter
        # TODO nail_movement

    if image_type == 1:   # Schraube
        i = 1
        # TODO screw_movement

    if image_type == 2:   # Scheibe
        i = 1
        # TODO shim_movement

    kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(0.25, -0.48, 0.225)), 1.0)
    kin.lin(PyKDL.Frame(rot, PyKDL.Vector(0.0, -0.48, 0.225)), 1.0)

    #kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(0.255, -0.48, 0.225)), 1.0)
    #kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(0.5, 0.45, 0.225)), 0.75)

    gripper.publish(False)

    pred_list.append(image_type)
    diff = differences(prod_list, pred_list)
    accuracy = 100.0
    if len(pred_list) > 0:
        accuracy = (float(diff) / float(len(pred_list))) * 100.0
    print('Fehlerquote = ', accuracy, '%')

    if (len(prod_list)) > 24:
        plt.pie(x=[100-accuracy, accuracy], explode=(0.1, 0), labels=['Richtig erkannt', 'Falsch erkannt'], autopct='%1.1f%%', shadow=True, colors=['green', 'red'])
        plt.axis('equal')
        plt.legend(title="Objekterkennung")
        plt.show()

    imageRecieved = False

