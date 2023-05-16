#!/usr/bin/env python3
import rospy
from rumba_msg.msg import BatteryStatus

if __name__ == "__main__":
    rospy.init_node("Battery_status")
    pub = rospy.Publisher("/rumba/battery_status", BatteryStatus, queue_size=10)
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        msg = BatteryStatus()
        msg.percentage_battery = 80
        msg.location_x = 2.5
        msg.location_y = 5.5
        msg.orientation_theta = 1.5
        msg.brushes_up = True
        msg.debug_message = "Battery ok"

        pub.publish(msg)
        rate.sleep()