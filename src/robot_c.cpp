#include "robot.h"
#include "comtest.h"

extern "C"
{
    JcnsFranka::Robot* init(char *ip) {
        return new JcnsFranka::Robot(ip);
    }

    void deinit(JcnsFranka::Robot* self) {
        delete self;
    }

    double* read_state(JcnsFranka::Robot* self) {
        JcnsFranka::Coordinates state = self->read_state();
        double *result = new double[10];
        for (int i = 0; i < 7; i++)
            result[i] = state.joints[i];
        for (int i = 0; i < 3; i++)
            result[i + 7] = state.xyz[i];
        return result;
    }

    void set_load(JcnsFranka::Robot* self, double mass, double* F_x_Cload,
                  double* load_inertia) {
        std::array<double, 3> arg2;
        std::array<double, 9> arg3;
        for (int i = 0; i < 3; i++)
            arg2[i] = F_x_Cload[i];
        for (int i = 0; i < 9; i++)
            arg3[i] = load_inertia[i];
        self->set_load(mass, arg2, arg3);
    }

    void go_home(JcnsFranka::Robot* self) {
        self->go_home();
    }

    void move_joints(JcnsFranka::Robot* self, double* joints) {
        std::array<double, 7> joints_arr;
        for (int i = 0; i < 7; i++)
            joints_arr[i] = joints[i];
        self->move_joints(joints_arr);
    }

    void move_relative(JcnsFranka::Robot* self, double dx, double dy,
                       double dz, double dt) {
        self->move_relative(dx, dy, dz, dt);
    }

    void move_linear(JcnsFranka::Robot* self, double dx, double dy, double dz) {
        self->move_linear(dx, dy, dz);
    }

    void move_absolute(JcnsFranka::Robot* self, double x, double y, double z) {
        self->move_absolute(x, y, z);
    }

    bool is_gripping(JcnsFranka::Robot* self) {
        return self->is_gripping();
    }

    void close_gripper(JcnsFranka::Robot* self, double force, double width) {
        self->close_gripper(force, width);
    }

    void move_gripper(JcnsFranka::Robot* self, double width) {
        self->move_gripper(width);
    }

    bool is_in_error_mode(JcnsFranka::Robot* self) {
        return self->is_in_error_mode();
    }

    char* read_error(JcnsFranka::Robot* self) {
        return self->read_error();
    }

    void reset_error(JcnsFranka::Robot* self) {
        self->reset_error();
    }

    uint64_t communication_test(char *ip) {
        return JcnsFranka::communication_test(ip);
    }
}
