#ifndef JCNSFRANKA_H
#define JCNSFRANKA_H

#include "liborl/liborl.h"

namespace JcnsFranka {
    struct Coordinates {
        std::array<double, 7> joints;
        std::array<double, 3> xyz;
    };

    class Robot
    {
    public:
        Robot(char *ip);
        ~Robot();

        Coordinates readState();
        bool goHome();
        void moveJoints(std::array<double, 7> joints);
        void moveRelative(double dx, double dy, double dz);
        void moveAbsolute(double x, double y, double z);
        bool isGripping();
        void grasp();
        void release();
        uint64_t communicationTest();

    private:
        orl::Robot *robot;
        double amax = 13; // [m s^-2]
        double vmax = 1.7;  // [m s^-1]
    };
}

#endif // JCNSFRANKA_H
