#!/usr/bin/env python
import rospy
import tf
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from jetracer.msg import jetRacerCar as JetRacerCarMsg
from collections import namedtuple
import math
import time

VICON_CAR_TOPIC = "vicon/jetracer_1/jetracer_1"
VICON_OBJECT_TOPIC = "vicon/jetracer_helmet/jetracer_helmet"
KEYBOARD_CONTROL_TOPIC = "/jetRacer/keyboard"
CAR_CONTROL_TOPIC = "/jetRacer_Controller"

DEBUG = False

class DubinsCar4D:
    def __init__(self, x=[0,0,0,0], uMin = [-1.0, -math.pi / 12], uMax = [1.0, math.pi / 12],
                 dMin = [0.0, 0.0], dMax=[0.0, 0.0], uMode="min", dMode="max"):
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
        assert(uMode in ["min", "max"])
        self.uMode = uMode
        if uMode == "min":
            assert(dMode == "max")
        else:
            assert(dMode == "min")
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
            if (spat_deriv[2] > 0):
                opt_a = self.uMin[0]
            if (spat_deriv[3] > 0):
                opt_w = self.uMin[1]
        else:
            if (spat_deriv[2] < 0):
                opt_a = self.uMin[0]
            if (spat_deriv[3] < 0):
                opt_w = self.uMin[1]
        return opt_a ,opt_w

    def dynamics(self, t, state, uOpt):
        L = 0.26
        x_dot = state[2] * math.cos(state[3])
        y_dot = state[2] * math.sin(state[3])
        v_dot = uOpt[0]
        theta_dot = state[2] * math.tan(uOpt[1]) / L

        return (x_dot, y_dot, v_dot, theta_dot)

class Grid:
    def __init__(self, min, max, dims ,pts_each_dim, pDim=[]):
        self.max = max
        self.min = min
        self.dims = len(pts_each_dim)
        self.pts_each_dim = pts_each_dim
        self.pDim = pDim

        # Make some modifications to the initialized
        for dim in pDim:
            self.max[dim] = self.min[dim] + (self.max[dim] - self.min[dim]) * (1 - 1/self.pts_each_dim[dim])
        self.dx = (self.max - self.min) / (self.pts_each_dim - 1.0)

        """
        Below is re-shaping the self.vs so that we can make use of broadcasting
        self.vs[i] is reshape into (1,1, ... , pts_each_dim[i], ..., 1) such that pts_each_dim[i] is used in ith position
        """
        self.vs = []
        self.__grid_points = []
        for i in range(0,dims):
            tmp = np.linspace(self.min[i],self.max[i], num=self.pts_each_dim[i])
            broadcast_map = np.ones(self.dims, dtype=int)
            broadcast_map[i] = self.pts_each_dim[i]
            self.__grid_points.append(tmp)
            tmp = np.reshape(tmp, tuple(broadcast_map))
            self.vs.append(tmp)

    def get_index(self, state):
        index = []
        for i, s in enumerate(state):
          idx = np.searchsorted(self.__grid_points[i], s)
          if idx > 0 and (idx == len(self.__grid_points[i]) or
                          math.fabs(s - self.__grid_points[i][idx - 1]) < math.fabs(s - self.__grid_points[i][idx])):
            index.append(idx-1)
          else:
            index.append(idx)
        rospy.loginfo("x: {} y: {}, v: {}, theta: {}".format(
            self.__grid_points[0][index[0]],
            self.__grid_points[1][index[1]],
            self.__grid_points[2][index[2]],
            self.__grid_points[3][index[3]]))
        return index

    def get_value(self, V, state):
        index = self.get_index(state)
        return V[tuple(index)]


class Controller():
    def __init__(self):
        '''
        self.grid = Grid(np.array([-3.0, -1.0, -1.0, -math.pi / 2]),
                         np.array([4.0, 3.0, 4.0, math.pi / 2]),
                         4,
                         np.array([60, 60, 20, 36]), [3])
        '''
        self.grid = Grid(np.array([-1.0, -1.0, -1.0, -math.pi]),
                         np.array([2.0, 2.0, 1.0, math.pi]),
                         4,
                         np.array([40, 40, 20, 20]), [3])
        self.V = np.load("cone00.npy")
        self.car = DubinsCar4D() 
        self.prev_ts_vicon_msg_timestamp = None
        self.Position = namedtuple('Position', 'x y')
        self.prev_position = self.Position(x=0.0, y=0.0)
        self.prev_velocity = 0.0

        rospy.init_node('jetracer_controller_node')
        rospy.loginfo("starting jetracer controller node...")

        if DEBUG:
            rospy.Subscriber(VICON_CAR_TOPIC, TransformStamped, self.callback)
        else:
            rospy.Subscriber(KEYBOARD_CONTROL_TOPIC, JetRacerCarMsg, self.controller_callback, queue_size=1)
            self.publisher = rospy.Publisher(CAR_CONTROL_TOPIC, JetRacerCarMsg, queue_size=1)

        self.max_speed = 0.0
        self.max_t = 0.0
        self.min_dt = 100
        self.max_d = 0.0

        while not rospy.is_shutdown():
            rospy.spin()

    def controller_callback(self, jetracer_msg):
        start_time = time.time()
        ts_msg = rospy.wait_for_message(VICON_CAR_TOPIC, TransformStamped)
        # obj_msg = rospy.wait_for_message(VICON_OBJECT_TOPIC, TransformStamped)
        end_time = time.time()
        rospy.loginfo("eslapted time (s) {}".format(end_time - start_time))

        if self.prev_ts_vicon_msg_timestamp == None:
            current_time = rospy.Time().now().to_sec()
            self.prev_ts_vicon_msg_timestamp = current_time

        # get state of jetracer
        pose = ts_msg.transform
        theta = self.calculate_heading(pose)
        position = self.Position(x=pose.translation.x, y=pose.translation.y)
        velocity = self.calculate_velocity(position)

        state = (position.x, position.y, velocity, theta)
        i, j, k, l = self.grid.get_index(state)
        ''' 
        rospy.loginfo("x: {} y: {}, v: {}, theta: {}".format(
            self.grid.__grid_points[0][i],
            self.grid.__grid_points[1][j],
            self.grid.__grid_points[2][k],
            self.grid.__grid_points[3][l]))
        '''

        dV_dx3_L, dV_dx3_R = self.spa_derivX3_4d(i, j, k, l, self.V, self.grid)
        dV_dx4_L, dV_dx4_R = self.spa_derivX4_4d(i, j, k, l, self.V, self.grid)

        dV_dx3 = (dV_dx3_L + dV_dx3_R) / 2
        dV_dx4 = (dV_dx4_L + dV_dx4_R) / 2

        # rospy.loginfo("theta: " + str(theta))
        rospy.loginfo("speed: " + str(velocity))

        value = self.grid.get_value(self.V, state)
        rospy.loginfo("value: {}".format(value))
        if value < 0.1:
            rospy.loginfo("optimal control taking over!")
            opt_a, opt_w = self.car.opt_ctrl(state, (0, 0, dV_dx3, dV_dx4))
            rospy.loginfo("opt a: {} opt w: {}".format(opt_a, opt_w))
            jetracer_msg = JetRacerCarMsg()
            jetracer_msg.throttle = state[2] + 0.2 * opt_a
            jetracer_msg.throttle = min(jetracer_msg.throttle, 10.0)
            jetracer_msg.throttle = max(jetracer_msg.throttle, 15.0)
            jetracer_msg.steerAngle = opt_w

        self.publisher.publish(jetracer_msg)

    def calculate_heading(self, pose):
        x = pose.rotation.x
        y = pose.rotation.y
        z = pose.rotation.z
        w = pose.rotation.w
        quaternion = (x,y,z,w)

        # add offset to make yaw=0 face the wooden shelves
        # with drones
        # rotation_quaternion = tf.transformations.quaternion_from_euler(0, 0, math.pi/2)
        rotation_quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        # rotation_quaternion = tf.transformations.quaternion_from_euler(0, 0, -math.pi)

        quaternion = tf.transformations.quaternion_multiply(rotation_quaternion, quaternion)

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        return yaw

    def calculate_velocity(self, position):
        current_time = rospy.Time().now().to_sec()
        delta_time = current_time - self.prev_ts_vicon_msg_timestamp

        if delta_time < 0.1:
            return self.prev_velocity

        velocity = math.hypot(position.x - self.prev_position.x, position.y - self.prev_position.y) / delta_time
        # rospy.loginfo("dist: {}".format(velocity))
        # self.max_d = max(self.max_d, velocity)
        # rospy.loginfo("max dist: {}".format(self.max_d))
        # self.min_dt = min(self.min_dt, delta_time)
        # rospy.loginfo("min dt: {}".format(self.min_dt))
        # velocity /= delta_time
        self.prev_position = self.Position(x=position.x, y=position.y)
        self.prev_ts_vicon_msg_timestamp = rospy.Time().now().to_sec()
        return velocity


    def callback(self, ts_msg):
        ''' debug '''
        if self.prev_ts_vicon_msg_timestamp == None:
            self.prev_ts_vicon_msg_timestamp = rospy.Time().now().to_sec()

            pose = ts_msg.transform
            self.prev_position = self.Position(x=pose.translation.x, y=pose.translation.y)

        # get state of jetracer
        pose = ts_msg.transform
        theta = self.calculate_heading(pose)
        position = self.Position(x=pose.translation.x, y=pose.translation.y)
        velocity = self.calculate_velocity(position)

        # self.max_speed = max(velocity, self.max_speed)
        # rospy.loginfo("speed: " + str(velocity))
        # rospy.loginfo("max speed: " + str(self.max_speed))
        rospy.loginfo("theta: " + str(theta))

    def spa_derivX4_4d(self, i, j, k, l, V, g): # Left -> right == Outer Most -> Inner Most
        left_deriv = 0.0
        right_deriv = 0.0
        if 3 not in g.pDim:
            if l == 0:
                left_boundary = 0.0
                left_boundary = V[i, j, k, l] + abs(V[i, j, k, l + 1] - V[i, j, k, l]) * np.sign(V[i, j, k, l])
                left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[3]
                right_deriv = (V[i, j, k, l + 1] - V[i, j, k, l]) / g.dx[3]
            elif l == V.shape[3] - 1:
                right_boundary = 0.0
                right_boundary = V[i, j, k, l] + abs(V[i, j, k, l] - V[i, j, k, l - 1]) * np.sign(V[i, j, k, l])
                left_deriv = (V[i, j, k, l] - V[i, j, k, l - 1]) / g.dx[3]
                right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[3]
            elif (l != 0 and l != V.shape[3] - 1):
                left_deriv = (V[i, j, k, l] - V[i, j, k, l - 1]) / g.dx[3]
                right_deriv = (V[i, j, k, l + 1] - V[i, j, k, l]) / g.dx[3]
            return left_deriv, right_deriv
        else:
            if (l == 0):
                left_boundary = 0.0
                left_boundary = V[i, j, k, V.shape[3] - 1]
                left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[3]
                right_deriv = (V[i, j, k, l + 1] - V[i, j, k, l]) / g.dx[3]
            elif (l == V.shape[3] - 1):
                right_boundary = 0.0
                right_boundary = V[i, j, k, 0]
                left_deriv = (V[i, j, k , l] - V[i, j, k, l - 1]) / g.dx[3]
                right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[3]
            elif (l != 0 and l != V.shape[3] - 1):
                left_deriv = (V[i, j, k, l] - V[i, j, k, l - 1]) / g.dx[3]
                right_deriv = (V[i, j, k, l + 1] - V[i, j, k, l]) / g.dx[3]
            return left_deriv, right_deriv

    def spa_derivX3_4d(self, i, j, k, l, V, g): # Left -> right == Outer Most -> Inner Most
        left_deriv = 0.0
        right_deriv = 0.0
        if 2 not in g.pDim:
            if (k == 0):
                left_boundary = 0.0
                left_boundary = V[i, j, k, l] + abs(V[i, j, k + 1, l ] - V[i, j, k, l]) * np.sign(
                    V[i, j, k, l])
                left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[2]
                right_deriv = (V[i, j, k + 1, l] - V[i, j, k, l]) / g.dx[2]
            elif (k == V.shape[2] - 1):
                right_boundary = 0.0
                right_boundary = V[i, j, k, l] + abs(V[i, j, k, l] - V[i, j, k - 1, l]) * np.sign(
                    V[i, j, k, l])
                left_deriv = (V[i, j, k, l] - V[i, j, k - 1, l]) / g.dx[2]
                right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[2]
            elif (k != 0 and k != V.shape[2] - 1):
                left_deriv = (V[i, j, k, l] - V[i, j, k - 1, l]) / g.dx[2]
                right_deriv = (V[i, j, k + 1, l] - V[i, j, k, l]) / g.dx[2]
            return left_deriv, right_deriv
        else:
            if (k == 0):
                left_boundary = 0.0
                left_boundary = V[i, j, V.shape[2] - 1, l]
                left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[2]
                right_deriv = (V[i, j, k + 1, l] - V[i, j, k, l]) / g.dx[2]
            elif (k == V.shape[2] - 1):
                right_boundary = 0.0
                right_boundary = V[i, j, 0, l]
                left_deriv = (V[i, j, k , l] - V[i, j, k - 1, l]) / g.dx[2]
                right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[2]
            elif (k != 0 and k != V.shape[2] - 1):
                left_deriv = (V[i, j, k, l] - V[i, j, k - 1, l]) / g.dx[2]
                right_deriv = (V[i, j, k + 1, l] - V[i, j, k, l]) / g.dx[2]
            return left_deriv, right_deriv

    def spa_derivX2_4d(self, i, j, k, l, V, g): #
        left_deriv = 0.0
        right_deriv = 0.0
        if 1 not in g.pDim:
            if (j == 0):
                left_boundary = 0.0
                left_boundary = V[i, j, k, l] + abs(V[i, j + 1, k, l] - V[i, j, k, l]) * np.sign(
                    V[i, j, k, l])
                left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[1]
                right_deriv = (V[i, j + 1, k, l] - V[i, j, k, l]) / g.dx[1]
            elif (j == V.shape[1] - 1):
                right_boundary = 0.0
                right_boundary = V[i, j, k, l] + abs(V[i, j, k, l] - V[i, j - 1, k, l]) * np.sign(
                    V[i, j, k, l])
                left_deriv = (V[i, j, k, l] - V[i, j - 1, k, l]) / g.dx[1]
                right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[1]
            elif (j != 0 and j != V.shape[1] - 1):
                left_deriv = (V[i, j, k, l] - V[i, j - 1, k, l]) / g.dx[1]
                right_deriv = (V[i, j + 1, k, l] - V[i, j, k, l]) / g.dx[1]
            return left_deriv, right_deriv
        else:
            if (j == 0):
                left_boundary = 0.0
                left_boundary = V[i, V.shape[1] - 1 , k, l]
                left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[1]
                right_deriv = (V[i, j + 1, k, l] - V[i, j, k, l]) / g.dx[1]
            elif (j == V.shape[1] - 1):
                right_boundary = 0.0
                right_boundary = V[i, 0, k, l]
                left_deriv = (V[i, j, k, l] - V[i, j - 1, k, l]) / g.dx[1]
                right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[1]
            elif (j != 0 and j != V.shape[1] - 1):
                left_deriv = (V[i, j, k, l] - V[i, j - 1, k, l]) / g.dx[1]
                right_deriv = (V[i, j + 1, k, l] - V[i, j, k, l]) / g.dx[1]
            return left_deriv, right_deriv

    def spa_derivX1_4d(self, i, j, k, l, V, g): # Left -> right == Outer Most -> Inner Most
        left_deriv = 0.0
        right_deriv = 0.0
        if 0 not in g.pDim:
            if (i == 0):
                left_boundary = 0.0
                left_boundary = V[i, j, k, l] + abs(V[i + 1, j, k, l] - V[i, j, k, l]) * np.sign(
                    V[i, j, k, l])
                left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[0]
                right_deriv = (V[i + 1, j, k, l] - V[i, j, k, l]) / g.dx[0]
            elif (i == V.shape[0] - 1):
                right_boundary = 0.0
                right_boundary = V[i, j, k, l] + abs(V[i, j, k, l] - V[i - 1, j, k, l]) * np.sign(
                    V[i, j, k, l])
                left_deriv = (V[i, j, k, l] - V[i - 1, j, k, l]) / g.dx[0]
                right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[0]
            elif (i != 0 and i != V.shape[0] - 1):
                left_deriv = (V[i, j, k, l] - V[i -1, j, k, l]) / g.dx[0]
                right_deriv = (V[i + 1, j, k, l] - V[i, j, k, l]) / g.dx[0]
            return left_deriv, right_deriv
        else:
            if (i == 0):
                left_boundary = 0.0
                left_boundary = V[V.shape[0] - 1, j, k, l]
                left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[0]
                right_deriv = (V[i + 1, j, k, l] - V[i, j, k, l]) / g.dx[0]
            elif (i == V.shape[0] - 1):
                right_boundary = 0.0
                right_boundary = V[0, j, k, l]
                left_deriv = (V[i, j, k, l] - V[i - 1, j, k, l]) / g.dx[0]
                right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[0]
            elif (i != 0 and i != V.shape[0] - 1):
                left_deriv = (V[i, j, k, l] - V[i -1, j, k, l]) / g.dx[0]
                right_deriv = (V[i + 1, j, k, l] - V[i, j, k, l]) / g.dx[0]
            return left_deriv, right_deriv

def main():
    try:
        controller = Controller()
    except rospy.ROSInterruptException:
        rospy.loginfo("shutdown")
        pass

if __name__ == '__main__':
    main()
