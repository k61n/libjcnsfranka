#include <jcnsfranka.h>
#include <iostream>
#include <thread>

using namespace JcnsFranka;

Robot::Robot(char* ip)
{
    std::string ipstring(ip);
    try {
        robot = new orl::Robot(ipstring);
    }
    catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        robot->get_franka_robot().automaticErrorRecovery();
    }
}


Robot::~Robot()
{
    robot->get_franka_robot().stop();
    delete robot;
}


Coordinates Robot::readState()
{
    Coordinates result;
    orl::Pose pose;

    try {
        result.joints = robot->get_current_Joints();
        pose = robot->get_current_pose();
    }
    catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        robot->get_franka_robot().automaticErrorRecovery();
    }
    result.xyz[0] = pose.getPosition()[0];
    result.xyz[1] = pose.getPosition()[1];
    result.xyz[2] = pose.getPosition()[2];
    return result;
}


bool Robot::goHome()
{
    try {
        robot->get_franka_robot().stop();
        robot->get_franka_robot().automaticErrorRecovery();
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        double speed_factor = 0.5;
        robot->joint_motion(q_goal, speed_factor);
        robot->get_franka_gripper().homing();
    }
    catch (franka::Exception const& e) {
            std::cout << e.what() << std::endl;
            robot->get_franka_robot().automaticErrorRecovery();
            return false;
    }
    return true;
}


void Robot::moveJoints(std::array<double, 7> joints)
{
    try {
        robot->get_franka_robot().automaticErrorRecovery();
        double speed_factor = 0.5;
        robot->joint_motion(joints, speed_factor);
    }
    catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        robot->get_franka_robot().automaticErrorRecovery();
    }
}


void Robot::moveRelative(double dx, double dy, double dz)
{
    double s = sqrt(dx*dx + dy*dy + dz*dz);
    double t = s/vmax + vmax/amax;
    try {
        // t is the fastest time for franka to perform a movement
        // 4 * t is empirical value to allow franja move smooth yet fast
        robot->relative_cart_motion(dx, dy, dz, 4 * t);
    }
    catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        robot->get_franka_robot().automaticErrorRecovery();
    }
}


void Robot::moveAbsolute(double x, double y, double z)
{
    Coordinates state = this->readState();
    double x0 = state.xyz[0];
    double y0 = state.xyz[1];
    double z0 = state.xyz[2];
    double s = sqrt(pow((x - x0), 2) + pow((y - y0), 2) + pow((z - z0), 2));
    double t = s/vmax + vmax/amax;
    try {
        // t is the fastest time for franka to perform a movement
        // 4 * t is empirical value to allow franja move smooth yet fast
        robot->absolute_cart_motion(x, y, z, 4 * t);
    }
    catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        robot->get_franka_robot().automaticErrorRecovery();
    }
}


bool Robot::isGripping()
{
    franka::GripperState state;
    try {
        state = robot->get_franka_gripper().readOnce();
    }
    catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        robot->get_franka_robot().automaticErrorRecovery();
    }
    return state.is_grasped;
}


void Robot::grasp()
{
    try {
        robot->close_gripper(0, 0.05, 1, 0.1);
    }
    catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        robot->get_franka_robot().automaticErrorRecovery();
    }
}


void Robot::release()
{
    try {
        robot->open_gripper(0.05, 0.1);
    }
    catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        robot->get_franka_robot().automaticErrorRecovery();
    }
}


uint64_t Robot::communicationTest()
{
    uint64_t counter = 0;
    double avg_success_rate = 0.0;
    double min_success_rate = 1.0;
    double max_success_rate = 0.0;
    uint64_t time = 0;

    try {
        franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        robot->get_franka_robot().control([&time, &counter, &avg_success_rate, &min_success_rate, &max_success_rate, zero_torques](
                                          const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
            time += period.toMSec();
            if (time == 0.0) {
            return zero_torques;
            }
            counter++;

            if (counter % 100 == 0) {
            std::cout << "#" << counter
                      << " Current success rate: " << robot_state.control_command_success_rate
                      << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));

            avg_success_rate += robot_state.control_command_success_rate;
            if (robot_state.control_command_success_rate > max_success_rate) {
            max_success_rate = robot_state.control_command_success_rate;
            }
            if (robot_state.control_command_success_rate < min_success_rate) {
            min_success_rate = robot_state.control_command_success_rate;
            }

            if (time >= 10000) {
            std::cout << std::endl << "Finished test, shutting down example" << std::endl;
            return franka::MotionFinished(zero_torques);
            }

            // Sending zero torques - if EE is configured correctly, robot should not move
            return zero_torques;
            },
            false, 1000);

    }
    catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
    }
    std::cout << std::endl << std::endl << "#######################################################" << std::endl;
    uint64_t lost_robot_states = time - counter;
    if (lost_robot_states > 0) {
        std::cout << "The control loop did not get executed " << lost_robot_states << " times in the" << std::endl
                  << "last " << time << " milliseconds! (lost " << lost_robot_states << " robot states)" << std::endl << std::endl;
    }
    return lost_robot_states;
}
