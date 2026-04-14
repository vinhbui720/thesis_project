/*
 * NODE's PACKAGE: robot_planning
 * NODE EXE NAME:  stream_joint_command_node
 * NODE INIT NAME: stream_joint_command_node
 * Subcribe topic:
 * NODE function: Streams joint commands to /joint_command
 */
#define NODE_RATE 50

// --- INCLUDE BEGIN ---
#include <math.h>
#include <iostream>
#include <string>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#define NUMBER_OF_JOINT 6

sensor_msgs::JointState g_current_joints;

void cb_joint_state(const sensor_msgs::JointState::ConstPtr &msg)
{
    g_current_joints = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stream_joint_command_node");
    ros::NodeHandle nh;
    ros::Publisher streaming_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_command", 10);
    ros::Subscriber sub_joint = nh.subscribe("joint_states", 1, cb_joint_state);
    ros::Rate loop_rate(NODE_RATE);

    trajectory_msgs::JointTrajectory traj_msg;
    traj_msg.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(NUMBER_OF_JOINT, 0.0);
    point.velocities.resize(NUMBER_OF_JOINT, 0.0);
    point.time_from_start = ros::Duration(0.0);
    traj_msg.points.push_back(point);

    while (ros::ok())
    {
        // Update positions from latest joint state
        if (g_current_joints.position.size() == NUMBER_OF_JOINT)
        {
            for (size_t i = 0; i < NUMBER_OF_JOINT; ++i)
            {
                traj_msg.points[0].positions[i] = g_current_joints.position[i];
                traj_msg.points[0].velocities[i] = 0.0; // zero velocity for streaming
            }
            traj_msg.header.stamp = ros::Time::now();
            streaming_pub.publish(traj_msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
