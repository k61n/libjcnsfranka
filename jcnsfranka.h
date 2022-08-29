#ifndef JCNSFRANKA_H
#define JCNSFRANKA_H

#include "jcnsfranka_global.h"
#include "examples_common.h"

#include <franka/exception.h>
#include <iostream>

class JCNSFRANKA_EXPORT JcnsFranka
{
public:
    JcnsFranka(std::string ip);
    ~JcnsFranka();

    std::string readState();
    void communicationTest();

    bool goHome();
    void moveJoints(std::array<double, 7> joints);

private:
    void setDefault();

    franka::Robot *robot;
};

#endif // JCNSFRANKA_H
