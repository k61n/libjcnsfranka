#include "jcnsfranka.h"
#include <thread>
#include <chrono>

JcnsFranka::JcnsFranka(std::string ip)
{
    try {
        robot = new franka::Robot(ip, franka::RealtimeConfig::kIgnore);
        robot->automaticErrorRecovery();
        this->setDefault();
        std::cout << "Connected to " << ip << std::endl;
    }
    catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;

    }
}


JcnsFranka::~JcnsFranka()
{
    robot->stop();
    delete robot;
}


std::string JcnsFranka::readState()
{
    std::stringstream state;

    try {
        state << robot->readOnce();
    }
    catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
    }
    return state.str();
}


bool JcnsFranka::goHome()
{
    try {
        robot->stop();
        robot->automaticErrorRecovery();
        std::cout << "going home" << std::endl;;
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        double speed_factor = 0.5;
        MotionGenerator motion_generator(speed_factor, q_goal);
        robot->control(motion_generator, franka::ControllerMode::kJointImpedance, true,
                       franka::kDefaultCutoffFrequency);
    }
    catch (franka::Exception const& e) {
            std::cout << e.what() << std::endl;
            return false;
    }
    return true;
}


void JcnsFranka::moveJoints(double j1, double j2, double j3, double j4, double j5, double j6, double j7)
{
    try {
        robot->automaticErrorRecovery();
        std::array<double, 7> q_goal = {{j1, j2, j3, j4, j5, j6, j7}};
        double speed_factor = 0.5;
        MotionGenerator motion_generator(speed_factor, q_goal);
        robot->control(motion_generator, franka::ControllerMode::kJointImpedance, true,
                       franka::kDefaultCutoffFrequency);
    }
    catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
    }
}


void JcnsFranka::communicationTest()
{
    uint64_t counter = 0;
    double avg_success_rate = 0.0;
    double min_success_rate = 1.0;
    double max_success_rate = 0.0;
    uint64_t time = 0;

    try {
        franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        robot->control(
            [&time, &counter, &avg_success_rate, &min_success_rate, &max_success_rate, zero_torques](
                const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
              time += period.toMSec();
              if (time == 0.0) {
                return zero_torques;
              }
              counter++;
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
            true, 1000);

    }
    catch (franka::Exception const& e) {
    }
    std::cout << "Avg_success_rate " << avg_success_rate / counter << std::endl;
    std::cout << "Min_success_rate " << min_success_rate << std::endl;
    std::cout << "Max_success_rate " << max_success_rate << std::endl;
}


void JcnsFranka::setDefault()
{
    try {
        robot->setCollisionBehavior(
                    {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                    {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                    {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                    {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                    {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                    {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                    {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                    {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
        robot->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        robot->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    }
    catch (franka::Exception const& e) {
            std::cout << e.what() << std::endl;
    }
}
