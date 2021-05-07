#!/usr/bin/env python3
import rospy

import numpy as np
from geometry_msgs.msg import TransformStamped
from jetracer_controller.msg import optCtrl as JetRacerCarMsg
from collections import namedtuple
from threading import Lock
import subprocess
import math
import threading
from multiprocessing import Process, Queue, Array
import multiprocessing as mp

import time
import ctypes

# Pybind executable
import newexample

#import tf
from car4D import DubinsCar4D
from grid import *
from utilities import *
from tf_utilities import * 

import message_filters

VICON_CAR_TOPIC = "vicon/jetracer/jetracer"
# TODO: Correct this topic name
VICON_OBSTACLES_TOPIC = "vicon/cones"
OPTIMAL_CONTROL_TOPIC = "/jetRacer_optControl"
VICON_CONE1_TOPIC = "/vicon/cone_1/cone_1"
VICON_CONE2_TOPIC = "/vicon/cone_2/cone_2"

DEBUG = False

RADIUS = 0.75

class Controller:
    def __init__(self):
        rospy.init_node("aws_time_controller_node")
        rospy.Subscriber(VICON_CAR_TOPIC, TransformStamped, self.callback, queue_size=1)
        c1 = message_filters.Subscriber(VICON_CONE1_TOPIC, TransformStamped)
        c2 = message_filters.Subscriber(VICON_CONE2_TOPIC, TransformStamped)
        ts = message_filters.TimeSynchronizer([c1, c2], queue_size=1)
        ts.registerCallback(self.callback_time)

        while not rospy.is_shutdown():
            rospy.spin()

    def callback(self, ts_msg):
        rospy.loginfo("recv controller msg")

    def callback_time(self, cone1, cone2):
        ropsy.loginfo("recv cone msg")
        
def main():
    try:
        rospy.logwarn(
            "Remember to check that grid and car function are the same as in user_definer.py"
        )
        controller = Controller()
    except rospy.ROSInterruptException:
        rospy.loginfo("shutdown")
        pass


if __name__ == "__main__":
    main()
