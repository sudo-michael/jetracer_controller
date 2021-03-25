#!/usr/bin/env python2
import rospy

from jetracer_controller.msg import jetRacerCar as JetRacerCarMsg
from jetracer_controller.msg import optCtrl as optCtrl

from threading import Lock
import subprocess

# Subscription topic name
KEYBOARD_CONTROL_TOPIC = "/jetRacer/keyboard"
OPTIMAL_CONTROL_TOPIC  = "/jetRacer_optControl"

# Publish topic name
CAR_CONTROL_TOPIC = "/jetRacer_Controller"

DEBUG = False

class Car_controller:
    def __init__(self):
        rospy.init_node("jetracer_controller_node")
        rospy.loginfo("starting jetracer controller node...")

        # Listens to optimal control and manual control
        rospy.loginfo("starting subscriber for {}".format(OPTIMAL_CONTROL_TOPIC))
        rospy.Subscriber(KEYBOARD_CONTROL_TOPIC, JetRacerCarMsg, self.keyboard_callback, queue_size=1)
        rospy.Subscriber(OPTIMAL_CONTROL_TOPIC, optCtrl, self.opt_callback, queue_size=1)

        self.publisher = rospy.Publisher(
            CAR_CONTROL_TOPIC, JetRacerCarMsg, queue_size=1
        )
        self.manual_control = JetRacerCarMsg()
        self.manual_control_mutex = Lock()

        # play a sound when optimal control takes over
        self.play_sound = True
        while not rospy.is_shutdown():
            rospy.spin()

    def keyboard_callback(self, in_msg):
        self.manual_control_mutex.acquire()
        rospy.logdebug("received new message")
        self.manual_control.throttle = in_msg.throttle
        self.manual_control.steerAngle = in_msg.steerAngle
        self.manual_control_mutex.release()

    def opt_callback(self, opt_msg):
        if opt_msg.take_over: # In danger
            jetracer_msg = JetRacerCarMsg()
            jetracer_msg.throttle = opt_msg.throttle
            jetracer_msg.steerAngle = opt_msg.steerAngle
            if self.play_sound:
                jetracer_msg.throttle = 0.0
                subprocess.Popen(["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet",  "/home/michael/catkin_ws/src/jetracer_controller/scripts/safe.wav"])
                self.play_sound = False

                rospy.logwarn("optimal control taking over!")
                if jetracer_msg.steerAngle < 0:
                    rospy.loginfo("throttle: {} steerAngle: {} {}".format(jetracer_msg.throttle, jetracer_msg.steerAngle, "left"))
                else:
                    rospy.loginfo("throttle: {} steerAngle: {} {}".format(jetracer_msg.throttle, jetracer_msg.steerAngle, "right"))
            self.publisher.publish(jetracer_msg)
        else: # Not in danger
            if self.play_sound == False:
                self.play_sound = True

            self.manual_control_mutex.acquire()
            self.publisher.publish(self.manual_control)
            self.manual_control_mutex.release()

def main():
    try:
        rospy.logwarn(
            "Remember to check that grid and car function are the same as in user_definer.py"
        )
        controller = Car_controller()
    except rospy.ROSInterruptException:
        rospy.loginfo("shutdown")
        pass


if __name__ == "__main__":
    main()
