#include <jcnsfranka.h>

extern "C"
{
    JcnsFranka::Robot* init(char *ip) {return new JcnsFranka::Robot(ip);}
    void deinit(JcnsFranka::Robot* self) {delete self;}
    double* readState(JcnsFranka::Robot* self)
    {
        JcnsFranka::Coordinates state = self->readState();
        double *result = new double[10];
        for (int i=0; i < 7; i++)
            result[i] = state.joints[i];
        for (int i=0; i < 3; i++)
            result[i + 7] = state.xyz[i];
        return result;
    }
    void goHome(JcnsFranka::Robot* self) {self->goHome();}
    void moveJoints(JcnsFranka::Robot* self, double* joints)
    {
        std::array<double, 7> joints_arr;
        for (int i = 0; i < 7; i++)
            joints_arr[i] = joints[i];
        self->moveJoints(joints_arr);
    }
    void moveRelative(JcnsFranka::Robot* self, double dx, double dy, double dz) {self->moveRelative(dx, dy, dz);}
    void moveAbsolute(JcnsFranka::Robot* self, double x, double y, double z) {self->moveAbsolute(x, y, z);}
    bool isGripping(JcnsFranka::Robot* self) {return self->isGripping();}
    void close_gripper(JcnsFranka::Robot* self, double force, double width) {self->close_gripper(force, width);}
    void open_gripper(JcnsFranka::Robot* self, double width) {self->open_gripper(width);}
    uint64_t communicationTest(JcnsFranka::Robot* self) {return self->communicationTest();}
    bool is_in_error_mode(JcnsFranka::Robot* self) {return self->is_in_error_mode();}
    char* read_error(JcnsFranka::Robot* self) {return self->read_error();}
    void reset_error(JcnsFranka::Robot* self) {self->reset_error();}
}
