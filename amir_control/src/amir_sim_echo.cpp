/*
* /amir/jointCmdのデータを/amir/jointSensorに入れ、
* 疑似的にROS controlでAMIR 740をRviz上で動かすためのノード
* 
*/

#include <ros/ros.h>
#include <amir_control/jointCmd.h>
#include <amir_control/jointSensor.h>

ros::Publisher sensor_pub;

void cmdCallback(const amir_control::jointCmd::ConstPtr& msg)
{
    static amir_control::jointSensor sensor;

    for(int i = 0; i < msg->angle.size(); i++)
    {
        sensor.angle[i] = msg->angle[i];
        // sensor.vel[i] = msg->vel[i] / 1000;
    }
    sensor_pub.publish(sensor);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "amir_sim_echo");

    ros::NodeHandle n; 

    ros::Subscriber cmd_sub = n.subscribe("/amir/jointCmd", 10, cmdCallback);
    sensor_pub = n.advertise<amir_control::jointSensor>("/amir/jointSensor", 10);

    ros::spin();
}

