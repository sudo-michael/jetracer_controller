#!/usr/bin/env python
import rospy
import tf
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

class Controller():
    def __init__(self):
        self.prev_ts_vicon_msg_timestamp = None
        self.Position = namedtuple('Position', 'x y')
        self.prev_position = self.Position(x=0.0, y=0.0)
        rospy.init_node('jetracer_controller_node')
        rospy.loginfo("starting jetracer controller node...")
        rospy.Subscriber(KEYBOARD_CONTROL_TOPIC, JetRacerCarMsg, self.controller_callback, queue_size=1)
        self.publisher = rospy.Publisher(CAR_CONTROL_TOPIC, JetRacerCarMsg, queue_size=1)

        # DEBUG
        # rospy.Subscriber(VICON_CAR_TOPIC, TransformStamped, self.callback)

        while not rospy.is_shutdown():
            rospy.spin()

    # DEBUG
    '''
    def callback(self, ts_msg):
        if self.prev_ts_vicon_msg_timestamp == None:
            current_time = rospy.Time().now().to_sec()
            self.prev_ts_vicon_msg_timestamp = current_time

        # get state of jetracer
        pose = ts_msg.transform
        theta = self.calculate_heading(pose)
        position = self.Position(x=pose.translation.x, y=pose.translation.y)
        velocity = self.calculate_velocity(position)

        rospy.loginfo("theta: " + str(theta))
        rospy.loginfo("speed: " + str(velocity))
    '''

    def calculate_heading(self, pose):
        x = pose.rotation.x
        y = pose.rotation.y
        z = pose.rotation.z
        w = pose.rotation.w
        quaternion = (x,y,z,w)

        # add offset to make yaw=0 face the wooden shelves
        # with drones
        rotation_quaternion = tf.transformations.quaternion_from_euler(0, 0, -math.pi/2)

        quaternion = tf.transformations.quaternion_multiply(rotation_quaternion, quaternion)

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        return yaw

    def calculate_velocity(self, position):
        current_time = rospy.Time().now().to_sec()
        delta_time = current_time - self.prev_ts_vicon_msg_timestamp
        self.prev_ts_vicon_msg_timestamp = rospy.Time().now().to_sec()

        if delta_time < 0.0001:
            return  0.0
        else:
            velocity = math.hypot(position.x - self.prev_position.x, position.y - self.prev_position.y) / delta_time
            self.prev_position = self.Position(x=position.x, y=position.y)
            return velocity

    def controller_callback(self, jetracer_msg):
        start_time = time.time()
        ts_msg = rospy.wait_for_message(VICON_CAR_TOPIC, TransformStamped)
        obj_msg = rospy.wait_for_message(VICON_OBJECT_TOPIC, TransformStamped)
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

        # get position of obstacle
        pose =  obj_msg.transform.translation
        x = pose.x
        y = pose.y
        z = pose.z
        # rospy.loginfo("theta: " + str(theta))
        # rospy.loginfo("speed: " + str(velocity))
        rospy.loginfo("x: {}, y: {}, z: {}".format(x,y,z))

        self.publisher.publish(jetracer_msg)

def main():
    try:
        controller = Controller()
    except rospy.ROSInterruptException:
        rospy.loginfo("shutdown")
        pass

if __name__ == '__main__':
    main()
