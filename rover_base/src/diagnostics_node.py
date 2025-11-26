
#!/usr/bin/env python3

# File: rover_base/src/diagnostics_node.py

import rospy, time, os
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu, NavSatFix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class RoverDiagnostics:
    def __init__(self):
        rospy.init_node("rover_diagnostics")

        # State holders
        self.last_imu = 0
        self.last_gps = 0
        self.last_power = 0
        self.last_cmd_vel = 0
        self.power_data = None

        # Subs
        rospy.Subscriber("/imu/data", Imu, self.imu_cb)
        rospy.Subscriber("/fix", NavSatFix, self.gps_cb)
        rospy.Subscriber("/power", Float32MultiArray, self.power_cb)
        rospy.Subscriber("/cmd_vel", rospy.AnyMsg, self.cmd_vel_cb)

        # Pub
        self.pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)

        rospy.loginfo("[DIAGNOSTICS] Online")

    def imu_cb(self, msg):
        self.last_imu = time.time()

    def gps_cb(self, msg):
        self.last_gps = time.time()

    def power_cb(self, msg):
        self.last_power = time.time()
        self.power_data = msg.data

    def cmd_vel_cb(self, msg):
        self.last_cmd_vel = time.time()

    def cpu_temp(self):
        try:
            with open("/sys/class/thermal/thermal_zone0/temp") as f:
                return float(f.read())/1000.0
        except:
            return -1

    def make_status(self, name, level, message, values):
        st = DiagnosticStatus()
        st.name = name
        st.level = level
        st.message = message
        for k,v in values.items():
            st.values.append(KeyValue(k=str(k), v=str(v)))
        return st

    def spin(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            now = time.time()

            arr = DiagnosticArray()
            arr.header.stamp = rospy.Time.now()

            # IMU
            imu_ok = (now - self.last_imu < 1.0)
            arr.status.append(self.make_status(
                "IMU",
                DiagnosticStatus.OK if imu_ok else DiagnosticStatus.ERROR,
                "OK" if imu_ok else "No IMU data",
                {"age": now - self.last_imu}
            ))

            # GPS
            gps_ok = (now - self.last_gps < 2.0)
            arr.status.append(self.make_status(
                "GPS",
                DiagnosticStatus.OK if gps_ok else DiagnosticStatus.WARN,
                "OK" if gps_ok else "No GPS fix",
                {"age": now - self.last_gps}
            ))

            # Power
            if self.power_data:
                voltage, current = self.power_data
                level = DiagnosticStatus.OK
                msg = "OK"
                if voltage < 10.5:
                    level = DiagnosticStatus.ERROR
                    msg = "LOW BATTERY"
                arr.status.append(self.make_status(
                    "Battery",
                    level,
                    msg,
                    {"voltage": voltage, "current": current}
                ))

            # CPU Temp
            temp = self.cpu_temp()
            level = DiagnosticStatus.OK
            msg = "Normal"
            if temp > 70: msg, level = "HOT", DiagnosticStatus.WARN
            if temp > 80: msg, level = "OVERHEAT", DiagnosticStatus.ERROR
            arr.status.append(self.make_status(
                "CPU Temperature",
                level,
                msg,
                {"cpu_temp_c": temp}
            ))

            # CMD_VEL
            cmd_ok = (now - self.last_cmd_vel < 0.5)
            arr.status.append(self.make_status(
                "Command Stream",
                DiagnosticStatus.OK if cmd_ok else DiagnosticStatus.WARN,
                "OK" if cmd_ok else "cmd_vel timeout",
                {"age": now - self.last_cmd_vel}
            ))

            self.pub.publish(arr)
            rate.sleep()

if __name__ == "__main__":
    RoverDiagnostics().spin()
