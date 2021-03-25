#!/usr/bin/env python2
import rospy
import tf
import numpy as np
from geometry_msgs.msg import TransformStamped
from jetracer_controller.msg import optCtrl as JetRacerCarMsg
from collections import namedtuple
from threading import Lock
import subprocess

# Pybind executable
import newexample

from car4D import DubinsCar4D
from grid import *
from utilities import *

VICON_CAR_TOPIC = "vicon/jetracer_1/jetracer_1"
# TODO: Correct this topic name
VICON_OBSTACLES_TOPIC = "vicon/cones"
OPTIMAL_CONTROL_TOPIC = "/jetRacer_optControl"

DEBUG = False

class Controller:
    def __init__(self):
        rospy.init_node("aws_controller_node")
        rospy.loginfo("starting AWS controller node...")

        # Listens to Vicon
        rospy.loginfo("starting subscriber for {}".format(VICON_CAR_TOPIC))
        # This subscriber sends back opt control based on V
        rospy.Subscriber(VICON_CAR_TOPIC, TransformStamped, self.callback, queue_size=1)
        # This subscriber update the V function
        rospy.Subscriber(VICON_OBSTACLES_TOPIC, TransformStamped, self.UpdateV, queue_size=1)

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

        self.optimal_takeover = False
        self.optimal_timestamp = rospy.Time().now().to_sec()
        self.optimal_msg = JetRacerCarMsg()
        self.jetracer_msg = JetRacerCarMsg()

        # Protect the V function
        self.V_mutex = Lock()

        self.boundary_epsilon = 0.15

        # play a sound when optimal control takes over
        self.play_sound = True

        self.velocity_index = np.linspace(2.0, 4.0, 10)

        while not rospy.is_shutdown():
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
        rotation_quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        # rotation_quaternion = tf.transformations.quaternion_from_euler(0, 0, math.pi)
        # rotation_quaternion = tf.transformations.quaternion_from_euler(0, 0, -math.pi)

        quaternion = tf.transformations.quaternion_multiply(
            rotation_quaternion, quaternion
        )

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
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


    def callback(self, ts_msg):
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
        i, j, k, l = self.grid.get_index(state)

        rospy.logdebug(
            "car's location on the grid\n \
            x: {} y: {}, v: {}, theta: {}".format(
            self.grid._Grid__grid_points[0][i],
            self.grid._Grid__grid_points[1][j],
            self.grid._Grid__grid_points[2][k],
            self.grid._Grid__grid_points[3][l]))


        self.V_mutex.aquire()
        value = self.grid.get_value(self.V, state)
        self.V_mutex.release()

        current_time = rospy.Time().now().to_nsec()
        #delta_time = current_time - self.optimal_timestamp

        # if we near the bonudary, allow optimal control to take over for 0.5 seconds
        # before handing control back to user
        # near the bonudary of BRT, apply optimal control
        if value <= self.boundary_epsilon and self.in_bound(state):
            self.V_mutex.aquire()
            dV_dx3_L, dV_dx3_R = spa_derivX3_4d(i, j, k, l, self.V, self.grid)
            dV_dx4_L, dV_dx4_R = spa_derivX4_4d(i, j, k, l, self.V, self.grid)
            self.V_mutex.release()

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

            rospy.logwarn("optimal control taking over!")
            if jetracer_msg.steerAngle < 0:
                rospy.loginfo("throttle: {} steerAngle: {} {}".format(jetracer_msg.throttle, jetracer_msg.steerAngle, "left"))
            else:
                rospy.loginfo("throttle: {} steerAngle: {} {}".format(jetracer_msg.throttle, jetracer_msg.steerAngle, "right"))
            self.optimal_timestamp = rospy.Time().now().to_nsec()
            self.optimal_msg.throttle = jetracer_msg.throttle
            self.optimal_msg.steerAngle = jetracer_msg.steerAngle
            self.optimal_msg.take_over = True
        else:
            # No change to optimal_msg
            self.optimal_msg.take_over = False

        # Publish optimal control
        self.publisher.publish(self.optimal_msg)

    def UpdateV(self, obstacles):
        # TODO: Extract obstacle coordinates to
        # coordinates = [[1.4, 2.15], [0.0, 1.0], [-1.9, 1.55]]
        radius_list = [0.75, 0.75, 0.75]

        # Update value function - Pass obstacle list to FPGA computation
        tmp_V = newexample.hjsolver_test(coordinates, radius_list)
        self.V_mutex.aquire()
        self.V = tmp_V
        self.V_mutex.release()



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
