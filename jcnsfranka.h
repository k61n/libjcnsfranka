#ifndef JCNSFRANKA_H
#define JCNSFRANKA_H

#include "jcnsfranka_global.h"
#include "examples_common.h"

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <iostream>

class JCNSFRANKA_EXPORT JcnsFranka
{
public:
    JcnsFranka(std::string ip);
    ~JcnsFranka();

    franka::RobotState readState();
    bool isGripping();
    void communicationTest();

    bool goHome();
    void moveJoints(std::array<double, 7> joints);
    void grasp();
    void release();

    double maxWidth;

private:
    void setDefault();

    franka::Robot *robot;
    franka::Gripper *gripper;
};

#endif // JCNSFRANKA_H
