
#include "gripper.h"

using namespace JcnsFranka;

Gripper::Gripper(const std::string& ip)
{
    gripper = new franka::Gripper(ip);
}

Gripper::~Gripper()
{
    gripper->stop();
    delete gripper;
}

void Gripper::reference()
{
    gripper->homing();
    franka::GripperState state;
    state = gripper->readOnce();
    maxWidth = state.max_width;
}

double Gripper::max_width() const
{
    return maxWidth;
}

double Gripper::read_width()
{
    franka::GripperState state;
    state = gripper->readOnce();
    return state.width;
}

void Gripper::set_width(double width)
{
    gripper->move(width, 0.05);
}

void Gripper::grasp(double width, double force)
{
    // Gripper maintains the force until the target is reached,
    // therefore the deviation is kept of the size of maxWidth in order
    // to gripper to maintain the force indefinitely
    double deviation = maxWidth;
    gripper->grasp(width, 0.05, force, deviation, deviation);
}

bool Gripper::is_gripping()
{
    franka::GripperState state;
    state = gripper->readOnce();
    return state.is_grasped;
}

void Gripper::stop()
{
    gripper->stop();
}
