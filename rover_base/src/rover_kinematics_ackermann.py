#!/usr/bin/env python3

##############################################################################
# Fully Corrected Ackermann-Accurate Six-Wheel Kinematics Node
#  This version computes:
#  . Front & rear Ackermann steering angles
#  . Differential wheel velocities for:
#  . front left (FL), middle left (FM), rear left (RL)
#  . front right (FR), middle right (FM), rear right (RR)
#  . Correct direction + normalization
##############################################################################


import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math

class SixWheelAckermann:
    def __init__(self):
        rospy.init_node("ackermann_kinematics")

        # Robot geometry
        self.L = rospy.get_param("~wheelbase", 0.50)       # F-R distance
        self.W = rospy.get_param("~track_width", 0.40)     # L-R distance
        self.mid_offset = rospy.get_param("~mid_offset", 0.0)  # Optional middle offset

        self.max_speed = rospy.get_param("~max_speed", 1.0)
        self.max_steer_deg = rospy.get_param("~max_steer_deg", 35.0)

        self.pub_speed = rospy.Publisher("wheel_speed_cmd", Float32MultiArray, queue_size=1)
        self.pub_steer = rospy.Publisher("steering_cmd", Float32MultiArray, queue_size=1)

        rospy.Subscriber("cmd_vel", Twist, self.cmd_cb)

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def compute(self, vx, wz):
        """ Compute steering angles + wheel velocities for 6WD + 4WS """
        if abs(wz) < 1e-6:   # Straight driving
            angles = [0, 0, 0, 0]
            speed_norm = vx / self.max_speed
            return angles, [speed_norm] * 6

        R = vx / wz
        L = self.L
        W = self.W / 2.0

        # Ackermann equations
        delta_FL = math.atan(L / (R - W))
        delta_FR = math.atan(L / (R + W))
        delta_RL = -delta_FL
        delta_RR = -delta_FR

        # Clamp to mechanical limits
        clamp_rad = math.radians(self.max_steer_deg)
        angles = [
            self.clamp(delta_FL, -clamp_rad, clamp_rad),
            self.clamp(delta_FR, -clamp_rad, clamp_rad),
            self.clamp(delta_RL, -clamp_rad, clamp_rad),
            self.clamp(delta_RR, -clamp_rad, clamp_rad)
        ]

        # Turning arcs for each wheel
        # Front left/right, mid left/right, rear left/right
        R_FL = math.sqrt((R - W)**2 + (L)**2)
        R_FR = math.sqrt((R + W)**2 + (L)**2)
        R_ML = abs(R - W)
        R_MR = abs(R + W)
        R_RL = R_FL
        R_RR = R_FR

        radii = [R_FL, R_ML, R_RL, R_FR, R_MR, R_RR]  # FL,FM,RL, FR,FM,RR (ordered left then right)

        wheel_speeds = [wz * r for r in radii]

        # Normalize to [-1,1]
        maxv = max(abs(v) for v in wheel_speeds)
        if maxv > self.max_speed:
            wheel_speeds = [v / maxv * self.max_speed for v in wheel_speeds]

        speeds_norm = [v / self.max_speed for v in wheel_speeds]

        return angles, speeds_norm

    def cmd_cb(self, msg):
        vx = msg.linear.x
        wz = msg.angular.z

        angles, speeds = self.compute(vx, wz)

        # Convert angles to degrees
        steer_deg = [math.degrees(a) for a in angles]

        self.pub_steer.publish(Float32MultiArray(data=steer_deg))
        self.pub_speed.publish(Float32MultiArray(data=speeds))

if __name__ == "__main__":
    SixWheelAckermann()
    rospy.spin()
