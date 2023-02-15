#include "robot.h"
#include <iostream>
#include <thread>


using namespace JcnsFranka;


Robot::Robot(char* ip)
{
    std::string ipstring(ip);
    try {
        robot = new orl::Robot(ipstring);
        gripper = new Gripper(ipstring);
        frankaerror = "";
    }
    catch (franka::Exception const& e) {
        frankaerror = std::string(e.what());
//        std::strcpy(frankaerror, e.what());
//        strlcpy(frankaerror, e.what(), sizeof(e.what()));
    }
}


Robot::~Robot()
{
    gripper->stop();
    delete gripper;
    robot->get_franka_robot().stop();
    delete robot;
}


Coordinates Robot::read_state()
{
    Coordinates result;
    orl::Pose pose;

    try {
        result.joints = robot->get_current_Joints();
        pose = robot->get_current_pose();
        frankaerror = "";
    }
    catch (franka::Exception const& e) {
        frankaerror = std::string(e.what());
    }
    result.xyz[0] = pose.getPosition()[0];
    result.xyz[1] = pose.getPosition()[1];
    result.xyz[2] = pose.getPosition()[2];
    return result;
}


void Robot::set_load(double load_mass,
                     const std::array<double, 3>& F_x_Cload,
                     const std::array<double, 9>& load_inertia)
{
    orl::Payload payload;
    payload.mass = load_mass;
    payload.pos_wrt_flange = F_x_Cload;
    payload.inertia_matrix = load_inertia;
    try {
        robot->setLoad(payload);
        frankaerror = "";
    }
    catch (franka::Exception const& e) {
        frankaerror = std::string(e.what());
    }
}


void Robot::go_home()
{
    try {
        robot->get_franka_robot().stop();
        reset_error();
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        double speed_factor = 0.5;
        robot->joint_motion(q_goal, speed_factor);
        gripper->go_home();
        frankaerror = "";
    }
    catch (franka::Exception const& e) {
        frankaerror = std::string(e.what());
    }
}


void Robot::move_joints(std::array<double, 7> joints)
{
    try {
        double speed_factor = 0.5;
        robot->joint_motion(joints, speed_factor);
        frankaerror = "";
    }
    catch (franka::Exception const& e) {
        frankaerror = std::string(e.what());
    }
}


void Robot::move_relative(double dx, double dy, double dz)
{
    double s = sqrt(dx*dx + dy*dy + dz*dz);
    double t = s/vmax + vmax/amax;

    try {
        // t is the fastest time for franka to perform a movement
        // 4 * t is empirical value to allow franja move smooth yet fast
        robot->relative_cart_motion(dx, dy, dz, 4 * t);
        frankaerror = "";
    }
    catch (franka::Exception const& e) {
        frankaerror = std::string(e.what());
    }
}


void Robot::move_absolute(double x, double y, double z)
{
    Coordinates state = this->read_state();
    double x0 = state.xyz[0];
    double y0 = state.xyz[1];
    double z0 = state.xyz[2];
    double s = sqrt(pow((x - x0), 2) + pow((y - y0), 2) + pow((z - z0), 2));
    double t = s/vmax + vmax/amax;

    try {
        // t is the fastest time for franka to perform a movement
        // 4 * t is empirical value to allow franja move smooth yet fast
        robot->absolute_cart_motion(x, y, z, 4 * t);
        frankaerror = "";
    }
    catch (franka::Exception const& e) {
        frankaerror = std::string(e.what());
    }
}


bool Robot::is_gripping()
{
    bool state;
    try {
        state = gripper->is_gripping();
        frankaerror = "";
    }
    catch (franka::Exception const& e) {
        frankaerror = std::string(e.what());
    }
    return state;
}


void Robot::close_gripper(double width, double force)
{
    try {
        gripper->close_gripper(width, force);
        frankaerror = "";
    }
    catch (franka::Exception const& e) {
        frankaerror = std::string(e.what());
    }
}


void Robot::move_gripper(double width)
{
    try {
        gripper->move_gripper(width);
        frankaerror = "";
    }
    catch (franka::Exception const& e) {
        frankaerror = std::string(e.what());
    }
}


bool Robot::is_in_error_mode()
{
    if (frankaerror == "")
        return false;
    else
        return true;
}


char *Robot::read_error()
{
    return const_cast<char*>(frankaerror.c_str());
}


void Robot::reset_error()
{
    robot->get_franka_robot().automaticErrorRecovery();
    frankaerror = "";
}
