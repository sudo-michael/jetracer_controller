#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
import math

VICON_CAR_TOPIC = "vicon/jetracer_1/jetracer_1"
# prob need a class 1 for 
# https://answers.ros.org/question/232216/multiple-subscribers-and-single-publisher-in-one-python-script/



class Controller():
    def __init__(self):
        pass

    def vicon_callback(self):
        pass
''' if self.prev_call_vicon_ is None:
            self.prev_call_vicon_ = rospy.Time.now().to_sec()
            return
twist = Twist()
delta_time = rospy.Time.now().to_sec() - self.prev_call_vicon_
twist.linear.x = math.hypot(prev_state["position"][0] - self.state_["position"][0], prev_state["position"][1] - self.state_["position"][1]) / delta_time
twist.angular.z = ViconEnv.wrap_pi_to_pi(prev_state["orientation"]-self.state_["orientation"])/delta_time
self.prev_call_vicon_ = rospy.Time.now().to_sec()
self.velocity_history.add_element(np.asarray((twist.linear.x, twist.angular.z)), rospy.Time.now().to_sec())
'''

prev_vicon_msg_ts = None
prev_position = {"x": 0.0, "y": 0.0}

def calculate_heading(pose):
    x = pose.rotation.x
    y = pose.rotation.y
    z = pose.rotation.z
    w = pose.rotation.w
    quaternion = (x,y,z,w)
    (_, _, yaw) = tf.transformations.euler_from_quaternion(quaternion)
    return yaw


def callback(ts_data):
    global prev_vicon_msg_ts
    global prev_position
    if prev_vicon_msg_ts == None:
        current_time = rospy.Time().now().to_sec()
        # prev_vicon_msg_ts = current_time.secs + current_time.nsecs
        prev_vicon_msg_ts = current_time
    pose = ts_data.transform
    theta = calculate_heading(pose)
    position = {}
    position["x"] = pose.translation.y
    position["y"] = -pose.translation.x

    current_time = rospy.Time().now().to_sec()
    delta_time = current_time - prev_vicon_msg_ts
    prev_vicon_msg_ts = rospy.Time().now().to_sec()

    v = math.hypot(position["x"] - prev_position["x"], position["y"] - prev_position["y"]) / delta_time
    # rospy.loginfo("current x: {} current y: {}".format(position["x"], position["y"]))
    # rospy.loginfo("prev x: {} prev y: {}".format(prev_position["x"], prev_position["y"]))
    # rospy.loginfo("dt: {}".format(delta_time))
    rospy.loginfo("speed: " + str(v))
    prev_position = position



def main():
    rospy.init_node('jetson_controller')
    rospy.Subscriber(VICON_CAR_TOPIC, TransformStamped, callback)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
