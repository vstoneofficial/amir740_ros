/*
* /joint_statesのデータを/amir/jointCmdに入れ、
* joint_state_publisher_guiでAMIR 740をRviz上でスライダで動かすためのノード
* 速度を常に一定速度を送信するように設定します。
* 
*/

#include <ros/ros.h>
#include <amir_control/jointCmd.h>
#include <sensor_msgs/JointState.h>

const float DEG_TO_RAD = 0.017453292519943295769236907684886;
const int SMALL_MOTOR_SPEED = 90; // deg/s
const int LARGE_MOTOR_SPEED = 30; // deg/s

ros::Publisher relay_pub;

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    static amir_control::jointCmd output;

    for(int i = 0; i < 6; i++)
    {
        output.angle[i] = msg->position[i] * 1000; // rad to mrad
    }

    for(int i = 0; i < 4; i++)
    {
        output.vel[i] = LARGE_MOTOR_SPEED * DEG_TO_RAD * 1000; // mrad/s
    }
    for(int i = 4; i < 6; i++)
    {
        output.vel[i] = SMALL_MOTOR_SPEED * DEG_TO_RAD * 1000; // mrad/s
    }

    relay_pub.publish(output);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "joint_state_relay");

    // prep for ROS communcation
    ros::NodeHandle n; 
    ros::Subscriber cmd_sub = n.subscribe("/joint_states", 10, jointStateCallback); // robot feedback
    relay_pub = n.advertise<amir_control::jointCmd>("/amir/jointCmd", 10);
    ros::spin();
}

