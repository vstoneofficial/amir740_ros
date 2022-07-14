#ifndef AMIR_HW_INTERFACE_H
#define AMIR_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <amir_control/jointCmd.h>
#include <amir_control/jointSensor.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float32MultiArray.h>


const double DEG_TO_RAD = 0.017453292519943295769236907684886;
const double RAD_TO_DEG = 57.295779513082320876798154814105;
const double RAD_TO_MRAD = 1000.0;

namespace amir_ns
{
/** \brief Hardware interface for a robot */
class amirHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  amirHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Initialize the robot hardware interface */
  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration& elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration& elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration& period);

protected:
  std::vector<double> p_error_;
  std::vector<double> p_error_prev_;
  std::vector<double> cum_p_error_;
  std::vector<double> rate_p_error_;
  double v_error_;
  
  ros::Subscriber sensor_sub;
  ros::Subscriber goal_pos_sub_arm;
  ros::Subscriber goal_pos_sub_gripper;
  void sensorCallback(const amir_control::jointSensor::ConstPtr& msg);
  void armGoalCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& goal);
  void gripperGoalCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& goal);
  // void goalCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);


  ros::Publisher cmd_pub;
  std::vector<double> joint_position_prev_;
  std::vector<double> joint_velocity_prev_;
  const float maxAngle[6] = {170.0,     0.0, 160.0, 120.0,  158.0, 60.0};
  const float minAngle[6] = {-170.0, -135.0,   0.0, -75.0, -158.0, -15.0};


};  // class

}  // namespace ros_control_boilerplate

#endif
