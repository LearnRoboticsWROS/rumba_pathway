#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "turtlesim/Pose.h"
#include <iostream>

using namespace std;
geometry_msgs::Pose2D current_pose;

void callback_function(const turtlesim::Pose::ConstPtr &pos)
{
    current_pose.x = pos->x;
    current_pose.y = pos->y;
    current_pose.theta = pos->theta;
    cout << "Current x = " << current_pose.x;
    cout << "Current y = " << current_pose.y;
    cout << "Current theta = " << current_pose.theta << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rumba_pose");
    ros::NodeHandle n;
    turtlesim::Pose pos;
    ros::Subscriber sub = n.subscribe("turtle1/pose", 1000, callback_function);
    ros::spin();
    return 0;

}