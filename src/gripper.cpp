#include "gripper.h"


using namespace JcnsFranka;


Gripper::Gripper(std::string ip)
{
    gripper = new franka::Gripper(ip);
}


Gripper::~Gripper()
{
    gripper->stop();
    delete gripper;
}


void Gripper::go_home()
{
    gripper->homing();
    franka::GripperState state;
    state = gripper->readOnce();
    maxWidth = state.max_width;
}


double Gripper::read_width()
{
    franka::GripperState state;
    state = gripper->readOnce();
    return state.width;
}


double Gripper::max_width()
{
    return maxWidth;
}


void Gripper::close_gripper(double width, double force)
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


void Gripper::move_gripper(double width)
{
    gripper->move(width, 0.05);
}


void Gripper::stop()
{
    gripper->stop();
}
