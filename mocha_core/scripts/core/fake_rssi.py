#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

rospy.init_node("fake_rssi_sim")

rssi_topics = rospy.get_param(
    "~rssi_topics",
    [
        "/ddb/tplink/rssi/none_iris",
        "/ddb/tplink/rssi/jackal",
        "/ddb/tplink/rssi/basestation",
    ],
)
publishers = [rospy.Publisher(topic, Int32, queue_size=10) for topic in rssi_topics]

rate_hz = rospy.get_param("~rate", 2.0)
rate = rospy.Rate(rate_hz)

MIN_RSSI = rospy.get_param("~min_rssi", 35)
MAX_RSSI = rospy.get_param("~max_rssi", 100)
STEP = rospy.get_param("~step", 1)

class RSSISimulator:
    def __init__(self):
        self.current = MIN_RSSI
        self.direction = 1

    def next_value(self):
        self.current += STEP * self.direction

        if self.current >= MAX_RSSI:
            self.current = MAX_RSSI
            self.direction = -1

        elif self.current <= MIN_RSSI:
            self.current = MIN_RSSI
            self.direction = 1

        return int(self.current)

simulator = RSSISimulator()

rospy.loginfo("Fake RSSI triangle wave started for topics: %s", rssi_topics)

while not rospy.is_shutdown():
    try:
        rssi_value = simulator.next_value()

        rospy.loginfo(f"RSSI: {rssi_value} dBm")

        for publisher in publishers:
            publisher.publish(Int32(rssi_value))

    except Exception as e:
        rospy.logerr(f"Error: {e}")

    rate.sleep()
