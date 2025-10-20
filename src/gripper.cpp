
#include "gripper.h"

using namespace JcnsFranka;

Gripper::Gripper(const std::string& ip)
{
    gripper = new franka::Gripper(ip);
}

Gripper::~Gripper()
{
    stop();
    delete gripper;
}

void Gripper::reference()
{
    if (auto res = gripper->homing(); !res)
        throw std::runtime_error("jcnsfranka::Gripper::reference failed");
    franka::GripperState state = gripper->readOnce();
    maxWidth = state.max_width;
}

double Gripper::max_width() const
{
    return maxWidth;
}

double Gripper::read_width()
{
    franka::GripperState state = gripper->readOnce();
    return state.width;
}

void Gripper::set_width(double width)
{
    if (auto res = gripper->move(width, maxSpeed); !res)
        throw std::runtime_error("jcnsfranka::Gripper::move failed");
}

void Gripper::grasp(double width, double force)
{
    // Gripper maintains the force until the target is reached,
    // therefore the deviation is kept of the size of maxWidth in order
    // to gripper to maintain the force indefinitely
    double deviation = maxWidth;
    if (auto res = gripper->grasp(width, maxSpeed, force, deviation, deviation); !res)
        throw std::runtime_error("jcnsfranka::Gripper::grasp failed");
}

bool Gripper::is_gripping()
{
    franka::GripperState state = gripper->readOnce();
    return state.is_grasped;
}

void Gripper::stop()
{
    if (auto res = gripper->stop(); !res)
        throw std::runtime_error("jcnsfranka::Gripper::grasp failed");
}
