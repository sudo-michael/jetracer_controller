#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped

VICON_CAR_TOPIC = "vicon/jetracer/jetracer"

def talker():
    pub = rospy.Publisher(VICON_CAR_TOPIC, TransformStamped, queue_size=10)
    rospy.init_node('fake_car_publisher', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        car_pose = TransformStamped()
        car_pose.transform.translation.x = 1.0
        car_pose.transform.translation.y = -1.0
        pub.publish(car_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass