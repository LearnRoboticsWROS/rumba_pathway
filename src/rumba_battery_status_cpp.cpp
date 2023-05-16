#include "ros/ros.h"
#include "rumba_msg/BatteryStatus.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "battery_status");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<rumba_msg::BatteryStatus>("/rumba/battery_status", 1000);
    ros::Rate loop_rate(5);
    while (ros::ok())
    {
        rumba_msg::BatteryStatus msg;

        msg.percentage_battery = 80;
        msg.location_x = 2.5;
        msg.location_y = 5.5;
        msg.orientation_theta = 1.5;
        msg.brushes_up = true;
        msg.debug_message = "Battery ok, from cpp file";

        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}