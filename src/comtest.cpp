#include <iostream>
#include <thread>

#include <liborl/liborl.h>

#include "comtest.h"

uint64_t JcnsFranka::communication_test(char *ip, franka::RealtimeConfig realtime_config, bool limit_rate,
                                        double cutoff_frequency)
{
    uint64_t counter = 0;
    double avg_success_rate = 0.0;
    double min_success_rate = 1.0;
    double max_success_rate = 0.0;
    uint64_t time = 0;

    std::string ipstring(ip);
    franka::Robot *robot;
    try {
        robot = new franka::Robot(ipstring, realtime_config);
        robot->automaticErrorRecovery();
        franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        robot->control(
            [&time, &counter, &avg_success_rate, &min_success_rate, &max_success_rate, zero_torques]
            (const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques
            {
                time += period.toMSec();
                if (time == 0)
                    return zero_torques;
                counter++;

                if (counter % 100 == 0)
                    std::cout << "#" << counter << " Current success rate: "
                              << robot_state.control_command_success_rate << std::endl;
                std::this_thread::sleep_for(std::chrono::microseconds(100));

                avg_success_rate += robot_state.control_command_success_rate;
                if (robot_state.control_command_success_rate > max_success_rate)
                    max_success_rate = robot_state.control_command_success_rate;
                if (robot_state.control_command_success_rate < min_success_rate)
                    min_success_rate = robot_state.control_command_success_rate;

                if (time >= 10000)
                {
                    std::cout << "\nFinished test, shutting down example\n";
                    return franka::MotionFinished(zero_torques);
                }
                // Sending zero torques - if EE is configured correctly,
                // robot should not move
                return zero_torques;
            },
            limit_rate, cutoff_frequency);
    }
    catch (franka::Exception const& e)
    {
        std::cout << std::string(e.what());
    }
    delete robot;
    std::cout << "\n\n#######################################################\n";
    uint64_t lost_robot_states = time - counter;
    if (lost_robot_states > 0)
        std::cout << "The control loop did not get executed " << lost_robot_states << " times in the\n"
                  << "last " << time << " milliseconds! (lost " << lost_robot_states << " robot states)\n\n";
    return lost_robot_states;
}

uint64_t JcnsFranka::communication_test(char *ip, int realtime_config, bool limit_rate, double cutoff_frequency)
{
    return communication_test(ip, static_cast<franka::RealtimeConfig>(realtime_config), limit_rate, cutoff_frequency);
}
