#ifndef JCNSFRANKA_H
#define JCNSFRANKA_H

#include "liborl/liborl.h"


struct Coordinates {
    std::array<double, 7> joints;
    std::array<double, 3> xyz;
};


class JcnsFranka
{
public:
    JcnsFranka(char *ip);
    ~JcnsFranka();

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


extern "C"
{
    JcnsFranka* init(char *ip) {return new JcnsFranka(ip);}
    void deinit(JcnsFranka* self) {delete self;}
    double* readState(JcnsFranka* self)
    {
        Coordinates state = self->readState();
        double *result = new double[10];
        for (int i=0; i < 7; i++)
            result[i] = state.joints[i];
        for (int i=0; i < 3; i++)
            result[i + 7] = state.xyz[i];
        return result;
    }
    bool goHome(JcnsFranka* self) {return self->goHome();}
    void moveJoints(JcnsFranka* self, double* joints)
    {
        std::array<double, 7> joints_arr;
        for (int i = 0; i < 7; i++)
            joints_arr[i] = joints[i];
        return self->moveJoints(joints_arr);
    }
    void moveRelative(JcnsFranka* self, double dx, double dy, double dz) {return self->moveRelative(dx, dy, dz);}
    void moveAbsolute(JcnsFranka* self, double x, double y, double z) {return self->moveAbsolute(x, y, z);}
    bool isGripping(JcnsFranka* self) {return self->isGripping();}
    void grasp(JcnsFranka* self) {return self->grasp();}
    void release(JcnsFranka* self) {return self->release();}
    uint64_t communicationTest(JcnsFranka* self) {return self->communicationTest();}
}


#endif // JCNSFRANKA_H
