#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "turtlesim/Pose.h"
#include "turtlesim/TeleportAbsolute.h"
#include "std_srvs/Empty.h"
#include "rumba_msg/TargetPosition.h"
#include <iostream>

class RumbaController
{
public:
    RumbaController()
    {
        // Class constructor
        pub_cmd_vel_ = n_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
        sub_pose_ = n_.subscribe("/turtle1/pose", 1000, &RumbaController::callback, this);
        client_pose_ = n_.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
        client_clear_ = n_.serviceClient<std_srvs::Empty>("/clear");
        client_target_pose = n_.serviceClient<rumba_msg::TargetPosition>("/go_to_position");
    }

    double getDistance()
    {
        return sqrt(pow(target_position.request.target_x - current_pose.x,2) + pow(target_position.request.target_y - current_pose.y,2));
    }
    double distance = getDistance();

    void callback(const turtlesim::Pose::ConstPtr &pos)
    {
        current_pose.x = pos->x;
        current_pose.y = pos->y;
        current_pose.theta = pos->theta;

        teleport_pose.request.x = 1.0;
        teleport_pose.request.y = 1.0;
        teleport_pose.request.theta = 0.0;

        target_position.request.target_x = 3.0;
        target_position.request.target_y = 3.0;
        target_position.request.target_theta = 0.0;


        if (client_target_pose.call(target_position))
        {
            
            ROS_INFO("Serivce called");
            std::cout<< target_position.response.message << std::endl;

            // float threshold = 0.05;

            float threshold;
            n_.getParam("/threshold_parameter", threshold);

            float diff = current_pose.theta - target_position.response.angular_z;
            float diff_theta = current_pose.theta - target_position.request.target_theta;
            std::cout << " diff theta = " << diff << std::endl;
            double distance = getDistance();

            // Rotate untill the turtle reached the target theta
            if (std::abs(diff) > threshold)
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = 1.0;

                // need a condition for stopping the turtle when reaches the target theta
                if (std::abs(diff_theta) < threshold and distance < threshold)
                {
                    cmd.linear.x = 0.0;
                    cmd.angular.z = 0.0;
                    ROS_INFO("Target position reached!");
                    client_clear_.call(clear_pathway);
                }
            }
            else if (std::abs(diff) < threshold)
            {
                cmd.linear.x = target_position.response.linear_x;
                cmd.angular.z = 0.0;
            }

        }

        else if (current_pose.x > 10.0)
        {
            cmd.linear.x = 0.8;
            cmd.angular.z = 1.2;
        }
        else if (current_pose.x < 1.0)
        {
            cmd.linear.x = 0.8;
            cmd.angular.z = -1.2;
        }
        else if (current_pose.y > 10)
        {
            client_pose_.call(teleport_pose);
            client_clear_.call(clear_pathway);
        }
        else
        {
            cmd.linear.x = 3.5;
            cmd.angular.z = 0.0;
        } 
        pub_cmd_vel_.publish(cmd);    

    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_cmd_vel_;
    ros::Subscriber sub_pose_;
    geometry_msgs::Pose2D current_pose;
    turtlesim::Pose pose;
    geometry_msgs::Twist cmd;
    ros::ServiceClient client_pose_;
    ros::ServiceClient client_clear_;
    turtlesim::TeleportAbsolute teleport_pose;
    std_srvs::Empty clear_pathway;

    // Create a client that request the go_to_position service 
    ros::ServiceClient client_target_pose;

    // Initialize the service
    rumba_msg::TargetPosition target_position;
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "pathway");
    RumbaController Controller;
    ros::spin();
    return 0;
}