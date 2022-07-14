#include <amir_control/amir_hw_interface.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <std_msgs/Float32MultiArray.h>

namespace amir_ns
{
    amirHWInterface::amirHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
        : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
    {
        sensor_sub = nh.subscribe("/amir/jointSensor", 1, &amirHWInterface::sensorCallback, this);
        goal_pos_sub_arm = nh.subscribe("/arm_velocity_trajectory_controller/follow_joint_trajectory/goal", 1, &amirHWInterface::armGoalCallback, this);
        goal_pos_sub_gripper = nh.subscribe("/gripper_velocity_trajectory_controller/follow_joint_trajectory/goal", 1, &amirHWInterface::gripperGoalCallback, this);

        cmd_pub = nh.advertise<amir_control::jointCmd>("/amir/jointCmd", 1);

        ROS_INFO("amirHWInterface declared.");
    }

    void amirHWInterface::armGoalCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr &msg)
    {
        int waypoints = msg->goal.trajectory.points.size();
        // Assume the final point of trajectory as our goal, and set it to joint_position_command_
        for (int i = 0; i < msg->goal.trajectory.points[waypoints - 1].positions.size(); i++)
        {
            joint_position_command_[i] = msg->goal.trajectory.points[waypoints - 1].positions[i];
        }
    }

    void amirHWInterface::gripperGoalCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr &msg)
    {
        int waypoints = msg->goal.trajectory.points.size();
        // Assume the final point of trajectory as our goal, and set it to joint_position_command_(gripper)
        joint_position_command_[5] = msg->goal.trajectory.points[waypoints - 1].positions[0];
    }

    void amirHWInterface::sensorCallback(const amir_control::jointSensor::ConstPtr &msg)
    {
        for (int i = 0; i < num_joints_; i++)
        {
            joint_position_[i] = msg->angle[i] / RAD_TO_MRAD; // mrad -> rad
        }
    }

    void amirHWInterface::init()
    {
        // Call parent class version of this function
        ros_control_boilerplate::GenericHWInterface::init();

        joint_velocity_prev_.resize(joint_velocity_.size());
        joint_position_command_.resize(joint_position_.size());
        joint_position_command_[0] = 2.970;  // 初期化の時、ロボットの初期姿勢を設定
        joint_position_command_[1] = 2.360;
        joint_position_command_[2] = -2.790;
        joint_position_command_[3] = 1.310;
        joint_position_command_[4] = -2.760;
        joint_position_command_[5] = 0.260;

        ROS_INFO("amirHWInterface initiated.");
    }

    void amirHWInterface::read(ros::Duration &elapsed_time)
    {
        // No need to read since our write() command populates our state for us
        // ros::spinOnce();
    }

    void amirHWInterface::write(ros::Duration &elapsed_time)
    {
        static amir_control::jointCmd joint_cmd;

        bool change_detected = false;
        for (int i = 0; i < num_joints_; i++)
        {
            if (joint_velocity_prev_[i] != joint_velocity_command_[i])
            {
                change_detected = true;
                i = num_joints_;
            }
        }

        if (change_detected)
        {
            for (int i = 0; i < num_joints_; i++)
            {
                joint_cmd.angle[i] = joint_position_command_[i] * RAD_TO_MRAD;
                joint_cmd.vel[i] = joint_velocity_command_[i] * RAD_TO_MRAD;
                joint_velocity_prev_[i] = joint_velocity_command_[i];
            }
            joint_cmd.msg_ctr = joint_cmd.msg_ctr + 1;
            cmd_pub.publish(joint_cmd);
        }
    }

    void amirHWInterface::enforceLimits(ros::Duration &period)
    {
        // Enforces position and velocity
        // pos_jnt_sat_interface_.enforceLimits(period);
        // vel_jnt_sat_interface_.enforceLimits(period);
    }

} // namespace amir_ns
