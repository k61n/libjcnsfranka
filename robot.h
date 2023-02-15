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
        Robot(char *ip);
        ~Robot();

        /**
         * @brief read_state
         * Reads current joints and end-effector positions
         * @return Current joints and end-effector positions
         */
        Coordinates read_state();

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
         * Moves the Franka robot to a homing position and resets the end-effector
         * Home position is { 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4 } [rad]
         * @return True when homing is finished
         */
        void go_home();

        /**
         * @brief move_joints
         * Sets the Franka robot in a position that corresponds to passed joint angles
         * @param joints an array with angles for the joints [rad]
         */
        void move_joints(std::array<double, 7> joints);

        /**
         * @brief move_relative
         * Perform relative motion of the Franka robot in cartesian space
         * @param dx relative displacement in X axis [m]
         * @param dy relative displacement in Y axis [m]
         * @param dz relative displacement in Z axis [m]
         */
        void move_relative(double dx, double dy, double dz);

        /**
         * @brief move_absolute
         * Perform motion of the Franka robot to a given coordinate in cartesian space
         * @param x target X coordinate in cartesian space [m]
         * @param y target Y coordinate in cartesian space [m]
         * @param z target Z coordinate in cartesian space [m]
         */
        void move_absolute(double x, double y, double z);

        /**
         * @brief is_gripping
         * Check gripping status of the end-effecor
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
         * Controls the distance between gripper's fingers
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
         * Resets current error.
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
         * Maximum velocuty of the Franka robot when moved in cartesian space
         */
        double vmax = 1.7;  // [m s^-1]

        /**
         * @brief frankaerror
         * Description of error from libfranka
         */
        std::string frankaerror = "";
    };
}


#endif // ROBOT_H
