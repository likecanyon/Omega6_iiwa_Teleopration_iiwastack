
#include <cmath>
#include <iostream>
#include <iiwa_ros/command/cartesian_pose.hpp>
#include <iiwa_ros/command/joint_position.hpp>
#include <iiwa_ros/service/time_to_destination.hpp>
#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/state/joint_position.hpp>
#include <iiwa_ros/state/cartesian_wrench.hpp>

#include "dhdc.h"
#include "HapticDevice.h"
// getTimeToDestination() can also return negative values and the info from the cabinet take some milliseconds to update
// once the motion is started. That means that if you call getTimeToDestination() right after you set a target pose, you
// might get the wrong info (e.g. a negative number). This function tried to call getTimeToDestination() until something
// meaningful is obtained or until a maximum amount of time passed.
void sleepForMotion(iiwa_ros::service::TimeToDestinationService &iiwa, const double maxSleepTime)
{
    double ttd = iiwa.getTimeToDestination();
    ros::Time start_wait = ros::Time::now();
    while (ttd < 0.0 && (ros::Time::now() - start_wait) < ros::Duration(maxSleepTime))
    {
        ros::Duration(0.5).sleep();
        ttd = iiwa.getTimeToDestination();
    }
    if (ttd > 0.0)
    {
        ROS_INFO_STREAM("Sleeping for " << ttd << " seconds.");
        ros::Duration(ttd).sleep();
    }
    else
    {
        ROS_ERROR_STREAM("cannot get the TimeToDestination");
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "CommandRobot");
    ros::NodeHandle nh("~");

    iiwa_ros::state::CartesianPose iiwa_pose_state;
    iiwa_ros::state::JointPosition iiwa_joint_state;
    iiwa_ros::command::CartesianPose iiwa_pose_command;
    iiwa_ros::command::JointPosition iiwa_joint_command;
    iiwa_ros::service::TimeToDestinationService iiwa_time_destination;
    iiwa_msgs::CartesianPose command_cartesian_position;
    iiwa_msgs::CartesianPose iiwa_init_pose;
    iiwa_msgs::JointPosition command_joint_position;
    iiwa_ros::state::CartesianWrench TCPforce_state;
    iiwa_msgs::CartesianWrench command_cartesian_force;

    iiwa_pose_state.init("iiwa");
    iiwa_pose_command.init("iiwa");
    iiwa_joint_state.init("iiwa");
    iiwa_joint_command.init("iiwa");
    iiwa_time_destination.init("iiwa");
    TCPforce_state.init("iiwa");
    std::cout << "ROS Init" << std::endl;
    // Haptic Init
    HapticDevice haptic_dev(nh, false);
    haptic_dev.SetForceLimit(20, 20, 20);
    int button0_state_ = dhdGetButton(0);
     
    haptic_dev.Start();
    haptic_dev.keep_alive_ = true;
    double current_position[3] = {0.0, 0.0, 0.0};
    double current_position_Init[3] = {0.0, 0.0, 0.0};
    std::cout << "Haptic Init" << std::endl;
    button0_state_ = dhdGetButton(0);
    std::cout<<button0_state_<<std::endl;
    // ROS spinner.
    ros::AsyncSpinner spinner(3);
    spinner.start();

    // Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
    bool use_cartesian_command;
    nh.param("use_cartesian_command", use_cartesian_command, true);

    // Dynamic parameter to choose the rate at wich this node should run
    double ros_rate;
    nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
    ros::Rate *loop_rate_ = new ros::Rate(ros_rate);

    // while (ros::ok() && (haptic_dev.keep_alive_ == true))
    // {
    //     if (iiwa_pose_state.isConnected())
    //     {
    //         command_joint_position.position.a1 = -60 * M_PI / 180;
    //         command_joint_position.position.a2 = 45 * M_PI / 180;
    //         command_joint_position.position.a3 = 0 * M_PI / 180;
    //         command_joint_position.position.a4 = -90 * M_PI / 180;
    //         command_joint_position.position.a5 = 0 * M_PI / 180;
    //         command_joint_position.position.a6 = -45 * M_PI / 180;
    //         command_joint_position.position.a7 = 0 * M_PI / 180;
    //         iiwa_joint_command.setPosition(command_joint_position);
    //         sleepForMotion(iiwa_time_destination, 2.0);
    //         break;
    //     }
    //     else

    //     {
    //         ROS_WARN_STREAM("Robot is not connected...");
    //         ros::Duration(5.0).sleep(); // 5 seconds
    //     }
    // }

    while (ros::ok() && (haptic_dev.keep_alive_ == true))
    {

        button0_state_ = dhdGetButton(0);
        std::cout<<button0_state_<<std::endl;
        // if (iiwa_pose_state.isConnected())
        {

            if (button0_state_)
            {
                std::cout << "button on" << std::endl;
                dhdGetPosition(&current_position[0], &current_position[1], &current_position[2]);
                command_cartesian_position.poseStamped.pose.position.x = iiwa_init_pose.poseStamped.pose.position.x + current_position[0] - current_position_Init[0];
                command_cartesian_position.poseStamped.pose.position.y = iiwa_init_pose.poseStamped.pose.position.y + current_position[1] - current_position_Init[1];
                command_cartesian_position.poseStamped.pose.position.z = iiwa_init_pose.poseStamped.pose.position.z + current_position[2] - current_position_Init[2];

                // command_cartesian_force = TCPforce_state.getWrench();
                // std::cout << command_cartesian_force.wrench.force.x << "   ";
                // std::cout << command_cartesian_force.wrench.force.y << "   ";
                // std::cout << command_cartesian_force.wrench.force.z << "   ";
                // std::cout << command_cartesian_force.wrench.torque.x << "   ";
                // std::cout << command_cartesian_force.wrench.torque.y << "   ";
                // std::cout << command_cartesian_force.wrench.torque.z << std::endl;

                iiwa_pose_command.setPose(command_cartesian_position.poseStamped);
            }
            else
            {
                //主手位姿读取
                dhdGetPosition(&current_position_Init[0], &current_position_Init[1], &current_position_Init[2]);
                iiwa_init_pose = iiwa_pose_state.getPose();
                command_cartesian_position = iiwa_pose_state.getPose();
                command_joint_position = iiwa_joint_state.getPosition();
                // command_joint_position.position.a4 -=
                //     5 * M_PI / 180; // 0.0872665 // Adding/Subtracting 5 degrees (in radians) to the 4th joint
                // iiwa_joint_command.setPosition(command_joint_position);
            }
            //loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.
        }
        // else
        // {
        //     ROS_WARN_STREAM("Robot is not connected...");
        //     ros::Duration(5.0).sleep(); // 5 seconds
        // }
    }
    // process finished
    haptic_dev.keep_alive_ = false;
    spinner.stop();
};
