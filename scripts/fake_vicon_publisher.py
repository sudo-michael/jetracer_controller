#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped

VICON_CONE1_TOPIC = "/vicon/cone_1/cone_1"
VICON_CONE2_TOPIC = "/vicon/cone_2/cone_2"

def talker():
    pub1 = rospy.Publisher(VICON_CONE1_TOPIC, TransformStamped, queue_size=10)
    pub2 = rospy.Publisher(VICON_CONE2_TOPIC, TransformStamped, queue_size=10)

    rospy.init_node('fake_cone_publisher', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        cone1_pose = TransformStamped()
        cone2_pose = TransformStamped()
        #[1.4, 2.15], [0.0, 1.0]
        cone1_pose.transform.translation.x = 1.4
        cone1_pose.transform.translation.y = 2.15
        cone2_pose.transform.translation.x = 0.0
        cone2_pose.transform.translation.y = 1.0
        pub1.publish(cone1_pose)
        pub2.publish(cone2_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
