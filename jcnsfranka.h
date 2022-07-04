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
    void moveJoints(double j1, double j2, double j3, double j4, double j5, double j6, double j7);

private:
    void setDefault();

    franka::Robot *robot;
};

#endif // JCNSFRANKA_H
