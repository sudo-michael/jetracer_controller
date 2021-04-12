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

VICON_CAR_TOPIC = "vicon/jetracer/jetracer"
DEBUG = False


class Controller:
    def __init__(self):
        self.prev_ts_vicon_msg_timestamp = None
        self.Position = namedtuple("Position", "x y")
        self.prev_position = self.Position(x=0.0, y=0.0)
        self.prev_velocity = 0.0

        rospy.init_node("jetracer_controller_node_states")
        rospy.loginfo("starting jetracer controller node...")
        rospy.loginfo("starting subscriber for {}".format(VICON_CAR_TOPIC))
        rospy.Subscriber(VICON_CAR_TOPIC, TransformStamped, self.callback, queue_size=1)

        while not rospy.is_shutdown():
            rospy.spin()

    def velocity_test(self, ts_msg):
        if self.prev_ts_vicon_msg_timestamp == None:
            current_time = rospy.Time().now().to_sec()
            self.prev_ts_vicon_msg_timestamp = current_time

            self.prev_position = self.Position(
                x=pose.translation.x, y=pose.translation.y
            )

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
        print(state)
        print("x: {} y: {} v: {}, theta: {}".format(position.x, position.y, velocity, theta))


def main():
    try:
        controller = Controller()
    except rospy.ROSInterruptException:
        rospy.loginfo("shutdown")
        pass


if __name__ == "__main__":
    main()
