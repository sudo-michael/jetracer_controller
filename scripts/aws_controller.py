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
#VICON_CONE1_TOPIC = "/vicon/cone_1/cone_1"
#VICON_CONE2_TOPIC = "/vicon/cone_2/cone_2"

VICON_CONE1_TOPIC = "/downsampled_vicon_message1"
VICON_CONE2_TOPIC = "/downsampled_vicon_message2"

DEBUG = False

RADIUS = 0.55

class Controller:
    def __init__(self):
        rospy.init_node("aws_controller_node")
        rospy.loginfo("starting AWS controller node...")
        rospy.loginfo("Please make sure that FPGA Image has been loaded")

        # Listens to Vicon
        rospy.loginfo("starting subscriber for {}".format(VICON_CAR_TOPIC))
        # This subscriber sends back opt control based on V
        rospy.Subscriber(VICON_CAR_TOPIC, TransformStamped, self.callback, queue_size=1)
        # This subscriber update the V function
        #rospy.Subscriber(VICON_OBSTACLES_TOPIC, TransformStamped, self.UpdateV, queue_size=1)
        c1 = message_filters.Subscriber(VICON_CONE1_TOPIC, TransformStamped)
        c2 = message_filters.Subscriber(VICON_CONE2_TOPIC, TransformStamped)
        #ts = message_filters.TimeSynchronizer([c1, c2], queue_size=2)
        ts = message_filters.ApproximateTimeSynchronizer([c1, c2], queue_size=1, slop=0.3)
        ts.registerCallback(self.UpdateV)

        # Publish optimal control to car
        self.publisher = rospy.Publisher(
            OPTIMAL_CONTROL_TOPIC, JetRacerCarMsg, queue_size=1
        )

        # Variable initialization
        self.grid = Grid(
            np.array([-3.0, -1.0, 0.0, -math.pi]),
            np.array([3.0, 4.0, 4.0, math.pi]),
            4,
            np.array([60, 60, 20, 36]),
            [3],
        )
        self.V = np.zeros((60, 60, 20, 36))
        
        self.car = DubinsCar4D()
        self.prev_ts_vicon_msg_timestamp = None
        self.Position = namedtuple("Position", "x y")
        self.prev_position = self.Position(x=0.0, y=0.0)
        self.prev_velocity = 0.0
        self.optimal_msg = JetRacerCarMsg()
        # Protect the V function
        self.V_mutex = Lock()
        self.boundary_epsilon = 0.5
        self.velocity_index = np.linspace(2.0, 4.0, 10)
        rospy.loginfo("Initializing the FPGA")
        newexample.InitializeFPGA()

        self.coordinate_list = [[0.0, 0.0], [0.0, 0.0]]
        self.prev_list = [[0.0, 0.0], [0.0, 0.0]]
        self.compute_new = True
        
        #self.p = Process(target=self.computeValueFunc)
        while not rospy.is_shutdown():
            print("Here\n")
            rospy.spin()

    def calculate_heading(self, pose):
        x = pose.rotation.x
        y = pose.rotation.y
        z = pose.rotation.z
        w = pose.rotation.w
        quaternion = (x, y, z, w)

        # add offset to make yaw=0 face the wooden shelves
        # with drones
        # rotation_quaternion = tf.transformations.quaternion_from_euler(0, 0, math.pi/4)
        # rotation_quaternion = tf.transformations.quaternion_from_euler(0, 0, -math.pi/8)
        rotation_quaternion = quaternion_from_euler(0, 0, 0)
        #rotation_quaternion = quaternion_from_euler(0, 0, math.pi)
        # rotation_quaternion = tf.transformations.quaternion_from_euler(0, 0, -math.pi)

        quaternion = quaternion_multiply(
            rotation_quaternion, quaternion
        )

        roll, pitch, yaw = euler_from_quaternion(quaternion)
        return yaw

    def calculate_velocity(self, position):
        current_time = rospy.Time().now().to_sec()
        delta_time = current_time - self.prev_ts_vicon_msg_timestamp
        rospy.logdebug("dt: {}".format(delta_time))

        if delta_time < 0.03:
            return self.prev_velocity

        velocity = (
            math.hypot(
                position.x - self.prev_position.x, position.y - self.prev_position.y
            )
            / delta_time
        )
        self.prev_position = self.Position(x=position.x, y=position.y)
        self.prev_ts_vicon_msg_timestamp = rospy.Time().now().to_sec()
        self.prev_velocity = velocity
        return velocity

    def in_bound(self, state):
        return (-3.0 <= state[0] <= 3.0) and (-1.0 <= state[1] <= 4.0)
    
    def computeValueFunc(self, my_list, q):    
        # Update value function - Pass obstacle list to FPGA computation
        radius_list = [RADIUS, RADIUS]
        #coordinate_lissst = [[1.4, 2.3], [0.0, 0.0]]
        tmp = newexample.hjsolver_test(my_list, radius_list)
        #print(V_global.shape)
        #print(np.sum(V_global < 0))
        q.put(tmp)
        #self.V_mutex.acquire()
        #self.V = np.reshape(tmp_V, (60,60,20,36))
        #self.V_mutex.release()


    def callback(self, ts_msg):
        #print("Publishing optimal control\n")
        #print(threading.current_thread())
        if self.prev_ts_vicon_msg_timestamp == None:
            current_time = rospy.Time().now().to_sec()
            self.prev_ts_vicon_msg_timestamp = current_time

            pose = ts_msg.transform
            self.prev_position = self.Position(
                x=pose.translation.x, y=pose.translation.y
            )

        # get state of jetracer
        pose = ts_msg.transform
        theta = self.calculate_heading(pose)
        position = self.Position(x=pose.translation.x, y=pose.translation.y)
        velocity = self.calculate_velocity(position)
        state = (position.x, position.y, velocity, theta)
        #print(state)
        i, j, k, l = self.grid.get_index(state)

        #rospy.logdebug(
        #    "car's location on the grid\n \
        #    x: {} y: {}, v: {}, theta: {}".format(
        #    self.grid._Grid__grid_points[0][i],
        #    self.grid._Grid__grid_points[1][j],
        #    self.grid._Grid__grid_points[2][k],
        #    self.grid._Grid__grid_points[3][l]))


        self.V_mutex.acquire()
        value = self.grid.get_value(self.V, state)
        print(value)
        #self.V_mutex.release()

        current_time = rospy.Time().now().to_nsec()

        # if we near the bonudary, allow optimal control to take over for 0.5 seconds
        # before handing control back to user
        # near the bonudary of BRT, apply optimal control
        #if value <= self.boundary_epsilon and self.in_bound(state):
        if value <= self.boundary_epsilon:
            #self.V_mutex.acquire()
            dV_dx3_L, dV_dx3_R = spa_derivX3_4d(i, j, k, l, self.V, self.grid)
            dV_dx4_L, dV_dx4_R = spa_derivX4_4d(i, j, k, l, self.V, self.grid)
            #self.V_mutex.release()

            self.optimal_msg.take_over = True
            dV_dx3 = (dV_dx3_L + dV_dx3_R) / 2
            dV_dx4 = (dV_dx4_L + dV_dx4_R) / 2

            opt_a, opt_w = self.car.opt_ctrl(state, (0, 0, dV_dx3, dV_dx4))
            # rospy.loginfo("opt_a: {} opt_w: {}".format(opt_a, opt_w))
            jetracer_msg = JetRacerCarMsg()
            wanted_vel = state[2] + 2.0 * opt_a

            # find throttle closest to velocity
            idx = np.searchsorted(self.velocity_index, wanted_vel)
            if idx > 0 and (
                idx == len(self.velocity_index)
                or math.fabs(wanted_vel - self.velocity_index[idx - 1])
                < math.fabs(wanted_vel - self.velocity_index[idx])
            ):
                idx -= 1

            # if state[2] > 3.0:
            #    jetracer_msg.throttle = 0.0
            #else:
            idx += 10
            jetracer_msg.throttle = idx
            # change sign of opt_w to go in the correct direction
            # -steerAngle -> turn ccw
            # +steerAngle -> turn cw
            jetracer_msg.steerAngle = -1 * opt_w

            #rospy.logwarn("optimal control taking over!")
            #if jetracer_msg.steerAngle < 0:
            #    rospy.loginfo("throttle: {} steerAngle: {} {}".format(jetracer_msg.throttle, jetracer_msg.steerAngle, "left"))
            #else:
            #    rospy.loginfo("throttle: {} steerAngle: {} {}".format(jetracer_msg.throttle, jetracer_msg.steerAngle, "right"))
            self.optimal_msg.throttle = jetracer_msg.throttle
            self.optimal_msg.steerAngle = jetracer_msg.steerAngle
        else:
            # No change to optimal_msg
            self.optimal_msg.take_over = False

        #print(self.optimal_msg)
        self.V_mutex.release()
        # Publish optimal control
        self.publisher.publish(self.optimal_msg)

    def sqrt_dist(self, point1, point2):
        x_diff_sq = (point1[0] - point2[0]) * (point1[0] - point2[0])
        y_diff_sq = (point1[1] - point2[1]) * (point1[1] - point2[1])
        return math.sqrt(x_diff_sq + y_diff_sq)

    def isSame(self, coordinates1, coordinates2):
        #print(self.sqrt_dist(coordinates1[0], coordinates2[0]))
        #print(self.sqrt_dist(coordinates1[1], coordinates2[1]))
        if self.sqrt_dist(coordinates1[0], coordinates2[0]) >= 0.1 or self.sqrt_dist(coordinates1[1], coordinates2[1]) >= 0.1:
            return False
        return True
    
    # Using only 2 cones for now
    def UpdateV(self, cone1, cone2):
        cone1_pose = cone1.transform.translation
        cone2_pose = cone2.transform.translation
        #print(threading.current_thread())
        #print("Updating V\n")
        #print("x={} y={}".format(cone1_pose.x, cone1_pose.y))
        #print("x={} y={}".format(cone2_pose.x, cone2_pose.y))
    
        #print(np.sum(self.V < 0))
        coord_list = []
        radius_list = []

        #print(cone1_pose.z)
        #print(cone2_pose.z)
        if cone1_pose.z <= 1.5:
            coord_list.append([cone1_pose.x, cone1_pose.y])
            radius_list.append(RADIUS)
        if cone2_pose.z <= 1.5:
            coord_list.append([cone2_pose.x, cone2_pose.y])
            radius_list.append(RADIUS)
        #print(coord_list)
        self.coordinate_list = coord_list
        #self.coordinate_list = [[cone1_pose.x, cone1_pose.y], [cone2_pose.x, cone2_pose.y]]
    
        #if self.isSame(coord_list, self.prev_list):
        #    self.compute_new = False
        #else: 
        #    self.compute_new = True
        #     self.prev_list = coord_list
        start = time.time()
        #radius_list = [RADIUS, RADIUS]
        #if self.compute_new == True:
            #print("what\n")
        print("x={} y={}".format(cone1_pose.x, cone1_pose.y))
        print("x={} y={}".format(cone2_pose.x, cone2_pose.y))
        tmp = newexample.hjsolver_test(self.coordinate_list, radius_list)
        self.V_mutex.acquire()
        self.V = np.reshape(tmp, (60,60,20,36))
        self.V_mutex.release()
        end = time.time()
        #print(end - start)
        #np.save('tmp_result.npy', self.V)
        
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
