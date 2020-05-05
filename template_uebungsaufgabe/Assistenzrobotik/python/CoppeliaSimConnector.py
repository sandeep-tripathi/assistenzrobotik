#!/usr/bin/env python2

import rospy
from threading import Condition
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import time


class CoppeliaSimConnector:
    def __init__(self):

        def step_callback(data):
            with self.cv:
                self.cv.notifyAll()

        def time_callback(data):
            if self.current_sim_time is not None:
                step = data.data - self.current_sim_time
                self.sim_time_step = round(step, 2)
            self.current_sim_time = data.data

        self.cv = Condition()
        rospy.init_node('copeliaSimConnector', anonymous=True)

        self.sync_pub = rospy.Publisher('/enableSyncMode', Bool, queue_size=10, latch=True)
        self.startPub = rospy.Publisher('/startSimulation', Bool, queue_size=10, latch=True)
        self.stopPub = rospy.Publisher('/stopSimulation', Bool, queue_size=10, latch=True)
        self.triggerPub = rospy.Publisher('/triggerNextStep', Bool, queue_size=10, latch=True)
        self.subStep = rospy.Subscriber("/simulationStepDone", Bool, step_callback)
        self.subTime = rospy.Subscriber("/simulationTime", Float32, time_callback)
        self.current_sim_time = None
        self.sim_time_step = None

        time.sleep(2)
        self.stopPub.publish(True)
        time.sleep(2)
        self.sync_pub.publish(True)
        time.sleep(2)
        self.startPub.publish(True)
        self.step()

    def get_dt(self):
        return self.sim_time_step

    def get_simulation_time(self):
        return self.current_sim_time

    def step(self):
        with self.cv:
            self.triggerPub.publish(True)
            self.cv.wait()

    def sleep(self, time):
        count = int(time / self.sim_time_step)
        for i in xrange(count):
            self.step()
