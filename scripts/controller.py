#!/usr/bin/env python2
import rospy
import tf
import numpy as np
from geometry_msgs.msg import TransformStamped
from jetracer.msg import jetRacerCar as JetRacerCarMsg
from sensor_msgs.msg import Joy
from collections import namedtuple
import math
import time
from threading import Lock
import subprocess

VICON_CAR_TOPIC = "vicon/jetracer_1/jetracer_1"
KEYBOARD_CONTROL_TOPIC = "/jetRacer/keyboard"
JOY_TOPIC = "/joy"
CAR_CONTROL_TOPIC = "/jetRacer_Controller"

DEBUG = False


class DubinsCar4D:
    def __init__(
        self,
        x=[0, 0, 0, 0],
        uMin=[-1.5, -math.pi / 12],
        uMax=[1.5, math.pi / 12],
        dMin=[0.0, 0.0],
        dMax=[0.0, 0.0],
        uMode="max",
        dMode="min",
    ):
        """Creates a Dublin Car with the following states:
           X position, Y position, acceleration, heading

           The first element of user control and disturbance is acceleration
           The second element of user control and disturbance is heading


        Args:
            x (list, optional): Initial state . Defaults to [0,0,0,0].
            uMin (list, optional): Lowerbound of user control. Defaults to [-1,-1].
            uMax (list, optional): Upperbound of user control.
                                   Defaults to [1,1].
            dMin (list, optional): Lowerbound of disturbance to user control, . Defaults to [-0.25,-0.25].
            dMax (list, optional): Upperbound of disturbance to user control. Defaults to [0.25,0.25].
            uMode (str, optional): Accepts either "min" or "max".
                                   * "min" : have optimal control reach goal
                                   * "max" : have optimal control avoid goal
                                   Defaults to "min".
            dMode (str, optional): Accepts whether "min" or "max" and should be opposite of uMode.
                                   Defaults to "max".
        """
        self.x = x
        self.uMax = uMax
        self.uMin = uMin
        self.dMax = dMax
        self.dMin = dMin
        assert uMode in ["min", "max"]
        self.uMode = uMode
        if uMode == "min":
            assert dMode == "max"
        else:
            assert dMode == "min"
        self.dMode = dMode

    def opt_ctrl(self, state, spat_deriv):
        """
        :param t: time t
        :param state: tuple of coordinates
        :param spat_deriv: tuple of spatial derivative in all dimensions
        :return:
        """
        # System dynamics
        # x_dot     = v * cos(theta) + d_1
        # y_dot     = v * sin(theta) + d_2
        # v_dot = a
        # theta_dot = v * tan(delta) / L

        # Graph takes in 4 possible inputs, by default, for now
        opt_a = self.uMax[0]
        opt_w = self.uMax[1]
        if self.uMode == "min":
            if spat_deriv[2] > 0:
                opt_a = self.uMin[0]
            if spat_deriv[3] > 0:
                opt_w = self.uMin[1]
        else:
            if spat_deriv[2] < 0:
                opt_a = self.uMin[0]
            if spat_deriv[3] < 0:
                opt_w = self.uMin[1]
        return opt_a, opt_w

    def dynamics(self, t, state, uOpt):
        L = 0.3
        x_dot = state[2] * math.cos(state[3])
        y_dot = state[2] * math.sin(state[3])
        v_dot = uOpt[0]
        theta_dot = state[2] * math.tan(uOpt[1]) / L

        return (x_dot, y_dot, v_dot, theta_dot)


class Grid:
    def __init__(self, min, max, dims, pts_each_dim, pDim=[]):
        self.max = max
        self.min = min
        self.dims = len(pts_each_dim)
        self.pts_each_dim = pts_each_dim
        self.pDim = pDim

        # Make some modifications to the initialized
        for dim in pDim:
            self.max[dim] = self.min[dim] + (self.max[dim] - self.min[dim]) * (
                1 - 1 / self.pts_each_dim[dim]
            )
        self.dx = (self.max - self.min) / (self.pts_each_dim - 1.0)

        """
        Below is re-shaping the self.vs so that we can make use of broadcasting
        self.vs[i] is reshape into (1,1, ... , pts_each_dim[i], ..., 1) such that pts_each_dim[i] is used in ith position
        """
        self.vs = []
        self.__grid_points = []
        for i in range(0, dims):
            tmp = np.linspace(self.min[i], self.max[i], num=self.pts_each_dim[i])
            broadcast_map = np.ones(self.dims, dtype=int)
            broadcast_map[i] = self.pts_each_dim[i]
            self.__grid_points.append(tmp)
            tmp = np.reshape(tmp, tuple(broadcast_map))
            self.vs.append(tmp)

    def get_index(self, state):
        index = []
        for i, s in enumerate(state):
            idx = np.searchsorted(self.__grid_points[i], s)
            if idx > 0 and (
                idx == len(self.__grid_points[i])
                or math.fabs(s - self.__grid_points[i][idx - 1])
                < math.fabs(s - self.__grid_points[i][idx])
            ):
                '''
                if i == 0:
                    rospy.loginfo("x-inbetween: {} - {}".format(self.__grid_points[i][idx-1], self.__grid_points[i][idx]))
                elif i==1:
                    rospy.loginfo("y-inbetween: {} - {}".format(self.__grid_points[i][idx-1], self.__grid_points[i][idx]))
                '''
                index.append(idx - 1)
            else:
                index.append(idx)

        return index

    def get_value(self, V, state):
        index = self.get_index(state)
        return V[tuple(index)]


class Controller:
    def __init__(self):
        self.grid = Grid(
            np.array([-3.0, -1.0, 0.0, -math.pi]),
            np.array([3.0, 4.0, 4.0, math.pi]),
            4,
            np.array([60, 60, 20, 36]),
            [3],
        )
        self.V_file_apart = "/home/michael/catkin_ws/src/jetracer_controller/scripts/apart.npy"
        self.V_file_line = "/home/michael/catkin_ws/src/jetracer_controller/scripts/line_01.npy"
        self.V_file_center = "/home/michael/catkin_ws/src/jetracer_controller/scripts/center.npy"
        self.V_apart = np.load(self.V_file_apart)
        self.V_line = np.load(self.V_file_line)
        self.V_center = np.load(self.V_file_center)
        self.V = self.V_center
        self.car = DubinsCar4D()
        self.prev_ts_vicon_msg_timestamp = None
        self.Position = namedtuple("Position", "x y")
        self.prev_position = self.Position(x=0.0, y=0.0)
        self.prev_velocity = 0.0

        rospy.init_node("jetracer_controller_node")
        rospy.loginfo("starting jetracer controller node...")
        rospy.loginfo("using value function: {}".format(self.V_file_center))

        rospy.loginfo("starting subscriber for {}".format(VICON_CAR_TOPIC))
        rospy.Subscriber(JOY_TOPIC, Joy, self.joy_callback, queue_size=1)
        rospy.Subscriber(KEYBOARD_CONTROL_TOPIC, JetRacerCarMsg, self.control_callback, queue_size=1)
        rospy.Subscriber(VICON_CAR_TOPIC, TransformStamped, self.callback, queue_size=1)

        self.publisher = rospy.Publisher(
            CAR_CONTROL_TOPIC, JetRacerCarMsg, queue_size=1
        )

        self.optimal_takeover = False
        self.optimal_timestamp = rospy.Time().now().to_sec()
        self.optimal_msg = JetRacerCarMsg()

        self.jetracer_msg_mutex = Lock()
        self.jetracer_msg = JetRacerCarMsg()

        self.V_mutex = Lock()

        self.boundary_epsilon = 0.15

        # play a sound when optimal control takes over
        self.play_sound = True

        self.velocity_index = np.linspace(2.0, 4.0, 10)

        while not rospy.is_shutdown():
            rospy.spin()

    def joy_callback(self, joy_msg):
        if joy_msg.buttons[2]:
            rospy.loginfo("switch to line value")
            time.sleep(0.15)
            self.V_mutex.acquire()
            self.V = self.V_line
            self.V_mutex.release()
        elif joy_msg.buttons[0]:
            rospy.loginfo("switch to center value ")
            time.sleep(0.15)
            self.V_mutex.acquire()
            self.V = self.V_center
            self.V_mutex.release()
        elif joy_msg.buttons[1]:
            rospy.loginfo("switch to apart value ")
            time.sleep(0.15)
            self.V_mutex.acquire()
            self.V = self.V_apart
            self.V_mutex.release()

    def velocity_test(self, ts_msg):
        if self.prev_ts_vicon_msg_timestamp == None:
            current_time = rospy.Time().now().to_sec()
            self.prev_ts_vicon_msg_timestamp = current_time

            self.prev_position = self.Position(
                x=pose.translation.x, y=pose.translation.y
            )

    def control_callback(self, jetracer_msg):
        self.jetracer_msg_mutex.acquire()

        rospy.logdebug("received new message")
        self.jetracer_msg.throttle = jetracer_msg.throttle
        self.jetracer_msg.steerAngle = jetracer_msg.steerAngle

        self.jetracer_msg_mutex.release()

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
        # rotation_quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        rotation_quaternion = tf.transformations.quaternion_from_euler(0, 0, math.pi)
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
        delta_time = current_time - self.optimal_timestamp

        # if we near the bonudary, allow optimal control to take over for 0.5 seconds
        # before handing control back to user
        # near the bonudary of BRT, apply optimal control
        if value <= self.boundary_epsilon and self.in_bound(state):
            self.V_mutex.aquire()
            dV_dx3_L, dV_dx3_R = self.spa_derivX3_4d(i, j, k, l, self.V, self.grid)
            dV_dx4_L, dV_dx4_R = self.spa_derivX4_4d(i, j, k, l, self.V, self.grid)
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

            if self.play_sound:
                jetracer_msg.throttle = 0.0
                subprocess.Popen(["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet",  "/home/michael/catkin_ws/src/jetracer_controller/scripts/safe.wav"])
                self.play_sound = False

                rospy.logwarn("optimal control taking over!")
                if jetracer_msg.steerAngle < 0:
                    rospy.loginfo("throttle: {} steerAngle: {} {}".format(jetracer_msg.throttle, jetracer_msg.steerAngle, "left"))
                else:
                    rospy.loginfo("throttle: {} steerAngle: {} {}".format(jetracer_msg.throttle, jetracer_msg.steerAngle, "right"))
                rospy.loginfo("value: {}".format(value))
                rospy.loginfo("in grid world\nx: {} y: {}, v: {}, theta: {}".format(
                    self.grid._Grid__grid_points[0][i],
                    self.grid._Grid__grid_points[1][j],
                    self.grid._Grid__grid_points[2][k],
                    self.grid._Grid__grid_points[3][l]))

                rospy.loginfo("irl\n x: {} y: {}, v: {}, theta: {}".format(
                    state[0], state[1], state[2], state[3]))

            self.optimal_takeover = True
            self.optimal_timestamp = rospy.Time().now().to_nsec()
            self.optimal_msg.throttle = jetracer_msg.throttle
            self.optimal_msg.steerAngle = jetracer_msg.steerAngle


            # rospy.loginfo("throttle: {} steerAngle: {}".format(jetracer_msg.throttle, jetracer_msg.steerAngle))
            self.publisher.publish(self.optimal_msg)
        else:
            if self.play_sound == False:
                self.play_sound = True



            self.jetracer_msg_mutex.acquire()

            self.publisher.publish(self.jetracer_msg)

            self.jetracer_msg_mutex.release()

    def spa_derivX4_4d(
        self, i, j, k, l, V, g
    ):  # Left -> right == Outer Most -> Inner Most
        left_deriv = 0.0
        right_deriv = 0.0
        if 3 not in g.pDim:
            if l == 0:
                left_boundary = 0.0
                left_boundary = V[i, j, k, l] + abs(
                    V[i, j, k, l + 1] - V[i, j, k, l]
                ) * np.sign(V[i, j, k, l])
                left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[3]
                right_deriv = (V[i, j, k, l + 1] - V[i, j, k, l]) / g.dx[3]
            elif l == V.shape[3] - 1:
                right_boundary = 0.0
                right_boundary = V[i, j, k, l] + abs(
                    V[i, j, k, l] - V[i, j, k, l - 1]
                ) * np.sign(V[i, j, k, l])
                left_deriv = (V[i, j, k, l] - V[i, j, k, l - 1]) / g.dx[3]
                right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[3]
            elif l != 0 and l != V.shape[3] - 1:
                left_deriv = (V[i, j, k, l] - V[i, j, k, l - 1]) / g.dx[3]
                right_deriv = (V[i, j, k, l + 1] - V[i, j, k, l]) / g.dx[3]
            return left_deriv, right_deriv
        else:
            if l == 0:
                left_boundary = 0.0
                left_boundary = V[i, j, k, V.shape[3] - 1]
                left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[3]
                right_deriv = (V[i, j, k, l + 1] - V[i, j, k, l]) / g.dx[3]
            elif l == V.shape[3] - 1:
                right_boundary = 0.0
                right_boundary = V[i, j, k, 0]
                left_deriv = (V[i, j, k, l] - V[i, j, k, l - 1]) / g.dx[3]
                right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[3]
            elif l != 0 and l != V.shape[3] - 1:
                left_deriv = (V[i, j, k, l] - V[i, j, k, l - 1]) / g.dx[3]
                right_deriv = (V[i, j, k, l + 1] - V[i, j, k, l]) / g.dx[3]
            return left_deriv, right_deriv

    def spa_derivX3_4d(
        self, i, j, k, l, V, g
    ):  # Left -> right == Outer Most -> Inner Most
        left_deriv = 0.0
        right_deriv = 0.0
        if 2 not in g.pDim:
            if k == 0:
                left_boundary = 0.0
                left_boundary = V[i, j, k, l] + abs(
                    V[i, j, k + 1, l] - V[i, j, k, l]
                ) * np.sign(V[i, j, k, l])
                left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[2]
                right_deriv = (V[i, j, k + 1, l] - V[i, j, k, l]) / g.dx[2]
            elif k == V.shape[2] - 1:
                right_boundary = 0.0
                right_boundary = V[i, j, k, l] + abs(
                    V[i, j, k, l] - V[i, j, k - 1, l]
                ) * np.sign(V[i, j, k, l])
                left_deriv = (V[i, j, k, l] - V[i, j, k - 1, l]) / g.dx[2]
                right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[2]
            elif k != 0 and k != V.shape[2] - 1:
                left_deriv = (V[i, j, k, l] - V[i, j, k - 1, l]) / g.dx[2]
                right_deriv = (V[i, j, k + 1, l] - V[i, j, k, l]) / g.dx[2]
            return left_deriv, right_deriv
        else:
            if k == 0:
                left_boundary = 0.0
                left_boundary = V[i, j, V.shape[2] - 1, l]
                left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[2]
                right_deriv = (V[i, j, k + 1, l] - V[i, j, k, l]) / g.dx[2]
            elif k == V.shape[2] - 1:
                right_boundary = 0.0
                right_boundary = V[i, j, 0, l]
                left_deriv = (V[i, j, k, l] - V[i, j, k - 1, l]) / g.dx[2]
                right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[2]
            elif k != 0 and k != V.shape[2] - 1:
                left_deriv = (V[i, j, k, l] - V[i, j, k - 1, l]) / g.dx[2]
                right_deriv = (V[i, j, k + 1, l] - V[i, j, k, l]) / g.dx[2]
            return left_deriv, right_deriv

    def spa_derivX2_4d(self, i, j, k, l, V, g):  #
        left_deriv = 0.0
        right_deriv = 0.0
        if 1 not in g.pDim:
            if j == 0:
                left_boundary = 0.0
                left_boundary = V[i, j, k, l] + abs(
                    V[i, j + 1, k, l] - V[i, j, k, l]
                ) * np.sign(V[i, j, k, l])
                left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[1]
                right_deriv = (V[i, j + 1, k, l] - V[i, j, k, l]) / g.dx[1]
            elif j == V.shape[1] - 1:
                right_boundary = 0.0
                right_boundary = V[i, j, k, l] + abs(
                    V[i, j, k, l] - V[i, j - 1, k, l]
                ) * np.sign(V[i, j, k, l])
                left_deriv = (V[i, j, k, l] - V[i, j - 1, k, l]) / g.dx[1]
                right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[1]
            elif j != 0 and j != V.shape[1] - 1:
                left_deriv = (V[i, j, k, l] - V[i, j - 1, k, l]) / g.dx[1]
                right_deriv = (V[i, j + 1, k, l] - V[i, j, k, l]) / g.dx[1]
            return left_deriv, right_deriv
        else:
            if j == 0:
                left_boundary = 0.0
                left_boundary = V[i, V.shape[1] - 1, k, l]
                left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[1]
                right_deriv = (V[i, j + 1, k, l] - V[i, j, k, l]) / g.dx[1]
            elif j == V.shape[1] - 1:
                right_boundary = 0.0
                right_boundary = V[i, 0, k, l]
                left_deriv = (V[i, j, k, l] - V[i, j - 1, k, l]) / g.dx[1]
                right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[1]
            elif j != 0 and j != V.shape[1] - 1:
                left_deriv = (V[i, j, k, l] - V[i, j - 1, k, l]) / g.dx[1]
                right_deriv = (V[i, j + 1, k, l] - V[i, j, k, l]) / g.dx[1]
            return left_deriv, right_deriv

    def spa_derivX1_4d(
        self, i, j, k, l, V, g
    ):  # Left -> right == Outer Most -> Inner Most
        left_deriv = 0.0
        right_deriv = 0.0
        if 0 not in g.pDim:
            if i == 0:
                left_boundary = 0.0
                left_boundary = V[i, j, k, l] + abs(
                    V[i + 1, j, k, l] - V[i, j, k, l]
                ) * np.sign(V[i, j, k, l])
                left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[0]
                right_deriv = (V[i + 1, j, k, l] - V[i, j, k, l]) / g.dx[0]
            elif i == V.shape[0] - 1:
                right_boundary = 0.0
                right_boundary = V[i, j, k, l] + abs(
                    V[i, j, k, l] - V[i - 1, j, k, l]
                ) * np.sign(V[i, j, k, l])
                left_deriv = (V[i, j, k, l] - V[i - 1, j, k, l]) / g.dx[0]
                right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[0]
            elif i != 0 and i != V.shape[0] - 1:
                left_deriv = (V[i, j, k, l] - V[i - 1, j, k, l]) / g.dx[0]
                right_deriv = (V[i + 1, j, k, l] - V[i, j, k, l]) / g.dx[0]
            return left_deriv, right_deriv
        else:
            if i == 0:
                left_boundary = 0.0
                left_boundary = V[V.shape[0] - 1, j, k, l]
                left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[0]
                right_deriv = (V[i + 1, j, k, l] - V[i, j, k, l]) / g.dx[0]
            elif i == V.shape[0] - 1:
                right_boundary = 0.0
                right_boundary = V[0, j, k, l]
                left_deriv = (V[i, j, k, l] - V[i - 1, j, k, l]) / g.dx[0]
                right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[0]
            elif i != 0 and i != V.shape[0] - 1:
                left_deriv = (V[i, j, k, l] - V[i - 1, j, k, l]) / g.dx[0]
                right_deriv = (V[i + 1, j, k, l] - V[i, j, k, l]) / g.dx[0]
            return left_deriv, right_deriv


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
