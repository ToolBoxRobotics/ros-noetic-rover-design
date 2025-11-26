#!/usr/bin/env python3

# rover_base/src/cmd_vel_watchdog.py

import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class CmdVelWatchdog:
    def __init__(self):
        rospy.init_node("cmd_vel_watchdog")

        self.timeout = rospy.get_param("~timeout", 0.5)
        self.last_cmd_time = time.time()

        # Rover control publishers
        self.pub_speed = rospy.Publisher("wheel_speed_cmd", Float32MultiArray, queue_size=1)
        self.pub_steer  = rospy.Publisher("steering_cmd", Float32MultiArray, queue_size=1)

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb)

        rospy.loginfo("[WATCHDOG] Initialized with timeout %.2f sec", self.timeout)

    def cmd_cb(self, msg):
        self.last_cmd_time = time.time()

    def publish_stop(self):
        # 6 wheel speeds = 0
        self.pub_speed.publish(Float32MultiArray(data=[0.0]*6))
        # 4 steering servos = straight forward
        self.pub_steer.publish(Float32MultiArray(data=[0.0]*4))

    def spin(self):
        rate = rospy.Rate(20)
        warned = False
        while not rospy.is_shutdown():
            if time.time() - self.last_cmd_time > self.timeout:
                if not warned:
                    rospy.logwarn("[WATCHDOG] /cmd_vel timeout → stopping rover.")
                    warned = True
                self.publish_stop()
            else:
                warned = False
            rate.sleep()

if __name__ == "__main__":
    CmdVelWatchdog().spin()
