
#include "robot.h"
#include "comtest.h"

extern "C"
{
    JcnsFranka::Robot* init(char *ip, int realtime_config)
    {
        return new JcnsFranka::Robot(ip, realtime_config);
    }

    void deinit(JcnsFranka::Robot* self)
    {
        delete self;
    }

    struct JcnsFrankaPose
    {
        double joints[7];
        double xyz[3];
        double rpy[3];
    };

    JcnsFrankaPose read_state(JcnsFranka::Robot* self)
    {
        JcnsFranka::Pose state = self->read_state();
        JcnsFrankaPose result{};
        for (int i = 0; i < 7; i++)
            result.joints[i] = state.joints[i];
        for (int i = 0; i < 3; i++)
            result.xyz[i] = state.xyz[i];
        for (int i = 0; i < 3; i++)
            result.rpy[i] = state.rpy[i];
        return result;
    }

    int read_mode(JcnsFranka::Robot* self)
    {
        return static_cast<int>(self->read_mode());
    }

    struct JcnsFrankaLoad
    {
        double mass;
        double F_x_Cload[3];
        double load_inertia[9];
    };

    JcnsFrankaLoad read_load(JcnsFranka::Robot* self)
    {
        JcnsFranka::Load load = self->read_load();
        JcnsFrankaLoad result{};
        result.mass = load.mass;
        for (int i = 0; i < 3; i++)
            result.F_x_Cload[i] = load.F_x_Cload[i];
        for (int i = 0; i < 9; i++)
            result.load_inertia[i] = load.load_inertia[i];
        return result;
    };

    double read_csr(JcnsFranka::Robot* self)
    {
        return self->read_csr();
    }

    bool is_moving(JcnsFranka::Robot* self)
    {
        return self->is_moving();
    }

    void set_load(JcnsFranka::Robot* self, double mass, const double* F_x_Cload, const double* load_inertia)
    {
        std::array<double, 3> arg2{};
        std::array<double, 9> arg3{};
        for (int i = 0; i < 3; i++)
            arg2[i] = F_x_Cload[i];
        for (int i = 0; i < 9; i++)
            arg3[i] = load_inertia[i];
        self->set_load(mass, arg2, arg3);
    }

    void go_home(JcnsFranka::Robot* self)
    {
        self->go_home();
    }

    void move_joints(JcnsFranka::Robot* self, const double* joints, double speed_factor)
    {
        std::array<double, 7> joints_arr{};
        for (int i = 0; i < 7; i++)
            joints_arr[i] = joints[i];
        self->move_joints(joints_arr, speed_factor);
    }

    void move_relative(JcnsFranka::Robot* self, double dx, double dy, double dz, double dt)
    {
        self->move_relative(dx, dy, dz, dt);
    }

    void move_linear(JcnsFranka::Robot* self, double dx, double dy, double dz)
    {
        self->move_linear(dx, dy, dz);
    }

    void move_absolute(JcnsFranka::Robot* self, double x, double y, double z, double dt)
    {
        self->move_absolute(x, y, z, dt);
    }

    bool is_gripping(JcnsFranka::Robot* self)
    {
        return self->is_gripping();
    }

    double read_gripper_force(JcnsFranka::Robot* self)
    {
        return self->read_gripper_force();
    }

    double read_gripper_width(JcnsFranka::Robot* self)
    {
        return self->read_gripper_width();
    }

    void close_gripper(JcnsFranka::Robot* self, double width, double force)
    {
        self->close_gripper(width, force);
    }

    void move_gripper(JcnsFranka::Robot* self, double width)
    {
        self->move_gripper(width);
    }

    bool is_in_error_mode(JcnsFranka::Robot* self)
    {
        return self->is_in_error_mode();
    }

    char* read_error(JcnsFranka::Robot* self)
    {
        return self->read_error();
    }

    void reset_error(JcnsFranka::Robot* self)
    {
        self->reset_error();
    }

    uint64_t communication_test(char *ip, int realtime_config, bool limit_rate, double cutoff_frequency)
    {
        return JcnsFranka::communication_test(ip, realtime_config, limit_rate, cutoff_frequency);
    }
}
