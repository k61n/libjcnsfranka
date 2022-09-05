#ifndef JCNSFRANKA_H
#define JCNSFRANKA_H

#include "jcnsfranka_global.h"
#include "examples_common.h"

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <iostream>
#include "liborl/liborl.h"


struct Coordinates {
    std::array<double, 7> joints;
    std::array<double, 3> xyz;
};


class JCNSFRANKA_EXPORT JcnsFranka
{
public:
    JcnsFranka(std::string ip);
    ~JcnsFranka();

    Coordinates readState();
    bool goHome();
    void moveJoints(std::array<double, 7> joints);
    bool isGripping();
    void grasp();
    void release();
    void communicationTest();

private:
    orl::Robot *robot;
    double maxWidth = 0.7;
};

#endif // JCNSFRANKA_H
