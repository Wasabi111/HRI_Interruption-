#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"

#include <cmath>
#include <iostream>
#include <algorithm>

int language_state = 1;
int screen_state = 1;
int gripper_state = 1;
int prev_language_state = 1;
int prev_screen_state = 1;
int prev_gripper_state = 1;
const double easement_k = 0.5;
const double t_max = 8.0;
const double t_trigger = 0.2;

ros::Time staring_time;
ros::Duration staring_period;

bool isStaring()
{
    if (screen_state == 1 || gripper_state == 1)
    {
        return true;
    }
    return false;
}

bool isPrevStaring()
{
    if (prev_screen_state == 1 || prev_gripper_state == 1)
    {
        return true;
    }
    return false;
}

void languageCb(const std_msgs::Int64::ConstPtr& msg)
{
    prev_language_state = language_state;
    language_state = msg->data;
}

void updateGazeEvent()
{
    if (isStaring() && !isPrevStaring())
    {
        staring_time = ros::Time::now();
    }
    else if (isStaring() && isPrevStaring())
    {
        double time_diff = (ros::Time::now() - staring_time).toSec();
        if (staring_period.toSec() + time_diff <= t_max)
        {
            staring_period += ros::Duration(time_diff);
        }
        else
        {
            staring_period = ros::Duration(t_max);
        }
        staring_time = ros::Time::now();
    }
    else if (!isStaring() && isPrevStaring())
    {
        staring_time = ros::Time::now();
    }
    else if (!isStaring() && !isPrevStaring())
    {
        double time_diff = (ros::Time::now() - staring_time).toSec();
        if (staring_period.toSec() - time_diff >= 0)
        {
            staring_period -= ros::Duration(time_diff);
        }
        else
        {
            staring_period = ros::Duration(0);
        }
        staring_time = ros::Time::now();
    }
}

void screenCb(const std_msgs::Int64::ConstPtr& msg)
{
    prev_screen_state = screen_state;
    screen_state = msg->data;
}

void gripperCb(const std_msgs::Int64::ConstPtr& msg)
{
    prev_gripper_state = gripper_state;
    gripper_state = msg->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "easement_factor");
    ros::NodeHandle n;

    ros::Subscriber sub_language_easement = n.subscribe("/hiro/language/easement", 1, languageCb);
    ros::Subscriber sub_screen_status = n.subscribe("/hiro/lookat_screen", 1, screenCb);
    ros::Subscriber sub_gripper_status = n.subscribe("/hiro/lookat_gripper", 1, gripperCb);

    staring_time = ros::Time::now();
    staring_period = ros::Duration(0);

    ros::Publisher pub_easement = n.advertise<std_msgs::Float64>("/hiro/final_easement", 1);
    ros::Rate loop_rate(500);
    while (ros::ok())
    {
        updateGazeEvent();
        std_msgs::Float64 msg;
        double easement_time = std::max((staring_period.toSec() - t_trigger), 0.0);
        double gaze_exp = std::exp(-easement_k*easement_time) * 0.5 + 0.5;
        msg.data = std::min(gaze_exp, static_cast<double>(language_state));
        pub_easement.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}