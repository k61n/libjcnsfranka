
#include "robot.h"
#include <cmath>

using namespace JcnsFranka;

Robot::Robot(char* ip)
{
    std::string ipstring(ip);
    try
    {
        robot = new orl::Robot(ipstring);
        gripper = new Gripper(ipstring);
        frankaerror = "";
    }
    catch (franka::Exception const& e) {
        frankaerror = std::string(e.what());
    }
}

Robot::~Robot()
{
    gripper->stop();
    delete gripper;
    robot->get_franka_robot().stop();
    delete robot;
}

const Coordinates& Robot::read_state()
{
    if (!is_moving())
    {
        try
        {
            franka::RobotState s = robot->get_franka_robot().readOnce();
            state.joints = s.q;
            auto pose = orl::Pose(s.O_T_EE);
            state.xyz[0] = pose.getPosition()[0];
            state.xyz[1] = pose.getPosition()[1];
            state.xyz[2] = pose.getPosition()[2];
            frankaerror = "";
        }
        catch (franka::Exception const& e)
        {
            frankaerror = std::string(e.what());
        }
    }
    return state;
}

bool Robot::is_moving() const
{
    return moving;
}

void Robot::set_load(double load_mass, const std::array<double, 3>& F_x_Cload,
                     const std::array<double, 9>& load_inertia)
{
    orl::Payload payload{};
    payload.mass = load_mass;
    payload.pos_wrt_flange = F_x_Cload;
    payload.inertia_matrix = load_inertia;
    try
    {
        robot->setLoad(payload);
        frankaerror = "";
    }
    catch (franka::Exception const& e)
    {
        frankaerror = std::string(e.what());
    }
}

void Robot::go_home()
{
    moving = true;
    try
    {
        robot->get_franka_robot().stop();
        reset_error();
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        double speed_factor = 0.1;
        robot->joint_motion(
            [&](const franka::RobotState& frankastate)
            {
                copy_state(frankastate);
            },
            q_goal, speed_factor);
        gripper->go_home();
        gripperforce = 0;
        frankaerror = "";
    }
    catch (franka::Exception const& e)
    {
        frankaerror = std::string(e.what());
    }
    moving = false;
}

void Robot::move_joints(std::array<double, 7> joints, double speed_factor)
{
    moving = true;
    try
    {
        if ((speed_factor > 0) && (speed_factor <= 1))
        {
            robot->joint_motion(
                [&](const franka::RobotState& frankastate)
                {
                    copy_state(frankastate);
                },
                joints, speed_factor);
            frankaerror = "";
        } else
        {
            frankaerror = "jcnsfranka: speed_factor must be in range 0..1";
        }
    }
    catch (franka::Exception const& e)
    {
        frankaerror = std::string(e.what());
    }
    moving = false;
}

void Robot::move_relative(double dx, double dy, double dz, double dt)
{
    moving = true;
    if (dt == 0)
    {
        double s = sqrt(dx * dx + dy * dy + dz * dz);
        // fastest time for franka to perform a movement
        dt = s/vmax + vmax/amax;
        // 10 * t is empirical value to allow franka move smooth yet fast
        dt *= 10;
    }
    try
    {
        robot->relative_cart_motion(
            [&](const franka::RobotState& frankastate)
            {
                copy_state(frankastate);
            },
            dx, dy, dz, dt);
        frankaerror = "";
    }
    catch (franka::Exception const& e)
    {
        frankaerror = std::string(e.what());
    }
    moving = false;
}

void Robot::move_linear(double dx, double dy, double dz)
{
    double distance = sqrt(dx * dx + dy * dy + dz * dz);
    double step = 0.001; // steps are 1 mm
    int n = std::ceil(distance / step);
    double ddx = dx / n;
    double ddy = dy / n;
    double ddz = dz / n;

    for (int i = 0; i < n; i++)
        this->move_relative(ddx, ddy, ddz);
}

void Robot::move_absolute(double x, double y, double z)
{
    auto current_state = read_state();
    double x0 = current_state.xyz[0];
    double y0 = current_state.xyz[1];
    double z0 = current_state.xyz[2];
    double s = sqrt(pow((x - x0), 2) + pow((y - y0), 2) + pow((z - z0), 2));
    double t = s / vmax + vmax / amax;

    moving = true;
    try
    {
        // t is the fastest time for franka to perform a movement
        // 10 * t is empirical value to allow franka move smooth yet fast
        robot->absolute_cart_motion(
            [&](const franka::RobotState& frankastate)
            {
                copy_state(frankastate);
            },
            x, y, z, 10 * t);
        frankaerror = "";
    }
    catch (franka::Exception const& e)
    {
        frankaerror = std::string(e.what());
    }
    moving = false;
}

bool Robot::is_gripping()
{
    bool gripper_state;
    try
    {
        gripper_state = gripper->is_gripping();
        frankaerror = "";
    }
    catch (franka::Exception const& e)
    {
        frankaerror = std::string(e.what());
    }
    return gripper_state;
}

double Robot::read_gripper_force() const
{
    return gripperforce;
}

double Robot::read_gripper_width()
{
    if (!is_moving())
    {
        try
        {
            gripperwidth = gripper->read_width();
            frankaerror = "";
        }
        catch (franka::Exception const &e)
        {
            frankaerror = std::string(e.what());
        }
    }
    return gripperwidth;
}

void Robot::close_gripper(double width, double force)
{
    moving = true;
    try
    {
        gripperforce = force;
        gripper->close_gripper(width, force);
        frankaerror = "";
    }
    catch (franka::Exception const& e)
    {
        gripperforce = 0;
        frankaerror = std::string(e.what());
    }
    moving = false;
}

void Robot::move_gripper(double width)
{
    gripperforce = 0;
    moving = true;
    try
    {
        gripper->move_gripper(width);
        frankaerror = "";
    }
    catch (franka::Exception const& e)
    {
        frankaerror = std::string(e.what());
    }
    moving = false;
}

bool Robot::is_in_error_mode()
{
    return !frankaerror.empty();
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

void Robot::copy_state(const franka::RobotState &frankastate)
{
    Coordinates temp{};
    temp.joints = frankastate.q;
    auto pose = orl::Pose(frankastate.O_T_EE);
    temp.xyz[0] = pose.getPosition()[0];
    temp.xyz[1] = pose.getPosition()[1];
    temp.xyz[2] = pose.getPosition()[2];
    state = temp;
}
