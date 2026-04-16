#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
import tf
import math
import random

rospy.init_node("fake_rssi_sim")
listener = tf.TransformListener()

# Publish RSSI for both directions
pub_j2i = rospy.Publisher("/ddb/tplink/rssi/none_iris", Int32, queue_size=10)
pub_i2j = rospy.Publisher("/ddb/tplink/rssi/jackal", Int32, queue_size=10)
pub_i2j = rospy.Publisher("/ddb/tplink/rssi/basestation", Int32, queue_size=10)

rate = rospy.Rate(2.0)  # 1s update đúng yêu cầu

# RSSI Simulation parameters
MIN_RSSI = 35
MAX_RSSI = 100

STEP = 1  # mỗi giây tăng/giảm 1 dBm

class RSSISimulator:
    def __init__(self):
        self.current = MIN_RSSI
        self.direction = 1  # 1 = tăng, -1 = giảm

    def next_value(self):
        # cập nhật giá trị
        self.current += STEP * self.direction

        # chạm max → đảo chiều
        if self.current >= MAX_RSSI:
            self.current = MAX_RSSI
            self.direction = -1

        # chạm min → đảo chiều
        elif self.current <= MIN_RSSI:
            self.current = MIN_RSSI
            self.direction = 1

        return int(self.current)

simulator = RSSISimulator()

rospy.loginfo("📡 Fake RSSI Triangle Wave Started")

while not rospy.is_shutdown():
    try:
        rssi_value = simulator.next_value()

        rospy.loginfo(f"📡 RSSI: {rssi_value} dBm")

        pub_j2i.publish(Int32(rssi_value))
        pub_i2j.publish(Int32(rssi_value))

    except Exception as e:
        rospy.logerr(f"Error: {e}")

    rate.sleep()