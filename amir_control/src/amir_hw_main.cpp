#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <amir_control/amir_hw_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "amir_hw_interface_vel");
    ros::NodeHandle nh;

    // NOTE: We run the ROS loop in a separate thread as external calls such
    // as service callbacks to load controllers can block the (main) control loop
    ros::AsyncSpinner spinner(3);
    spinner.start();

    // Create the hardware interface specific to your robot
    std::shared_ptr<amir_ns::amirHWInterface> amir_hw_interface_vel(
        new amir_ns::amirHWInterface(nh));
    amir_hw_interface_vel->init();

    // Start the control loop
    ros_control_boilerplate::GenericHWControlLoop control_loop(nh, amir_hw_interface_vel);
    control_loop.run(); // Blocks until shutdown signal recieved

    return 0;
}
