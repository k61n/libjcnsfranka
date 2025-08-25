#ifndef ROBOT_H
#define ROBOT_H


#include "gripper.h"


namespace JcnsFranka {
    /**
     * @brief The Coordinates class
     * Used to output the state functions.
     */
    struct Coordinates {
        std::array<double, 7> joints;
        std::array<double, 3> xyz;
    };


    /**
     * @brief The Robot class
     * Simple wrapper around orl::Robot class
     */
    class Robot
    {
    public:
        /**
         * @brief Robot
         * Establishes the connection to the Franka robot
         * @param ip name or ip of the Franka robot
         */
        explicit Robot(char *ip);
        ~Robot();

        /**
         * @brief read_state
         * Reads current joints and end-effector positions
         * @return current joints and end-effector positions
         */
        const Coordinates& read_state();

        /**
         * @brief set_load
         * Sets dynamic parameters of a payload
         * @param load_mass mass of the load in [kg]
         * @param F_x_Cload translation from flange to center of mass of load [m]
         * @param load_inertia inertia matrix in [kg*m2], column-major
         */
        void set_load(double load_mass,
                      const std::array<double, 3>& F_x_Cload,
                      const std::array<double, 9>& load_inertia);

        /**
         * @brief go_home
         * Moves the Franka robot to homing position and resets the end-effector
         * Home position: { 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4 } [rad]
         * @return True when homing is finished
         */
        void go_home();

        /**
         * @brief move_joints
         * Sets the robot in a position that corresponds to passed joint angles
         * @param joints an array with angles for the joints [rad]
         * @param speed_factor fraction of max joint speed [a.u.]
         */
        void move_joints(std::array<double, 7> joints, double speed_factor);

        /**
         * @brief move_relative
         * Performs relative motion of the Franka robot in cartesian space
         * @param dx relative displacement in X axis [m]
         * @param dy relative displacement in Y axis [m]
         * @param dz relative displacement in Z axis [m]
         * @param dt time to complete the movement [s]
         */
        void move_relative(double dx, double dy, double dz, double dt=0);

        /**
         * @brief move_linear
         * Performs relative motion in cartesian space but ensures its
         * linear tajectory. As a consequence this movement might be
         * particularly slow
         * @param dx relative displacement in X axis [m]
         * @param dy relative displacement in Y axis [m]
         * @param dz relative displacement in Z axis [m]
         */
        void move_linear(double dx, double dy, double dz);

        /**
         * @brief move_absolute
         * Performs motion of robot to a given coordinate in cartesian space
         * @param x target X coordinate in cartesian space [m]
         * @param y target Y coordinate in cartesian space [m]
         * @param z target Z coordinate in cartesian space [m]
         */
        void move_absolute(double x, double y, double z);

        /**
         * @brief is_gripping
         * Checks gripping status of the end-effecor
         * @return True if end-effector is closed
         */
        bool is_gripping();

        /**
         * @brief close_gripper
         * Method to grasp an object with force
         * @param width
         * @param force
         */
        void close_gripper(double width, double force);

        /**
         * @brief move_gripper
         * Opens gripper to a desired width
         * @param width
         */
        void move_gripper(double width);

        /**
         * @brief is_in_error_mode
         * Class method to check if the robot is in error state
         * @return True if is in error state
         */
        bool is_in_error_mode();

        /**
         * @brief error
         * Class method to return recent error if any
         * @return error message obtained from libfranka exception
         */
        char* read_error();

        /**
         * @brief reset_error
         * Resets current error
         */
        void reset_error();

    private:
        /**
         * @brief robot
         * orl::Robot instance
         */
        orl::Robot *robot;

        /**
         * @brief gripper
         * JcnsFranka::Gripper instance
         */
        Gripper *gripper;

        /**
         * @brief amax
         * Maximum acceleration of the Franka robot when moved in cartesian space
         */
        double amax = 13; // [m s^-2]

        /**
         * @brief vmax
         * Maximum velocity of the Franka robot when moved in cartesian space
         */
        double vmax = 1.7;  // [m s^-1]

        /**
         * @brief frankaerror
         * Description of error from libfranka
         */
        std::string frankaerror;

        /**
         * @brief is_moving
         * Stores if robot is in motion
         */
        bool is_moving = false;

        /**
         * @brief state
         * Stores current robot state
         */
        Coordinates state{};

        /**
         * @brief copy_state
         * Convenience function to copy franka::RobotState to
         * JcnsFranka::Robot::state
         */
        void copy_state(const franka::RobotState& frankastate);
    };
}


#endif // ROBOT_H
