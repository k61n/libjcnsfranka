/**
 * @file jcnsfranka.h
 * @brief C++ class to wrap high-level functions from LibORL to conrol Franka Emika robot.
 * @authors Konstantin Kholostov
 */


#ifndef JCNSFRANKA_H
#define JCNSFRANKA_H


#include "liborl/liborl.h"


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
     * @brief The Gripper class
     * Simple wrapper around franka::Gripper class
     */
    class Gripper
    {
    public:
        /**
         * @brief Gripper
         * Connects to the gripper by name or ip address of the Franka robot
         * @param ip name or ip of the Franka robot
         */
        Gripper(std::string ip);
        ~Gripper();

        /**
         * @brief go_home
         * References the gripper
         */
        void go_home();

        /**
         * @brief read_width
         * @return width of the actual gripper state
         */
        double read_width();

        /**
         * @brief max_width
         * @return max width of the gripper
         */
        double max_width();

        /**
         * @brief is_gripping
         * @return Check gripping status
         * @return True is gripping
         */
        bool is_gripping();

        /**
         * @brief close_gripper
         * Closes the gripper applying the force
         * @param width
         * @param force
         */
        void close_gripper(double width, double force);

        /**
         * @brief open_gripper
         * Opens gripper to a desired width
         * @param width
         */
        void open_gripper(double width);
        void stop();

    private:
        /**
         * @brief state
         * Variable to store gripper state from libfranka
         */
        franka::GripperState state;

        /**
         * @brief gripper
         * Gripper instance from libfranka
         */
        franka::Gripper *gripper;

        /**
         * @brief maxWidth
         * Once gripper is initialized its max width is stored
         */
        double maxWidth;
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
         * @brief readState
         * Reads current joints and end-effector positions
         * @return Current joints and end-effector positions
         */
        Coordinates readState();

        /**
         * @brief goHome
         * Moves the Franka robot to a homing position and resets the end-effector
         * Home position is { 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4 } [rad]
         * @return True when homing is finished
         */
        void goHome();

        /**
         * @brief moveJoints
         * Sets the Franka robot in a position that corresponds to passed joint angles
         * @param joints an array with angles for the joints [rad]
         */
        void moveJoints(std::array<double, 7> joints);

        /**
         * @brief moveRelative
         * Perform relative motion of the Franka robot in cartesian space
         * @param dx relative displacement in X axis [m]
         * @param dy relative displacement in Y axis [m]
         * @param dz relative displacement in Z axis [m]
         */
        void moveRelative(double dx, double dy, double dz);

        /**
         * @brief moveAbsolute
         * Perform motion of the Franka robot to a given coordinate in cartesian space
         * @param x target X coordinate in cartesian space [m]
         * @param y target Y coordinate in cartesian space [m]
         * @param z target Z coordinate in cartesian space [m]
         */
        void moveAbsolute(double x, double y, double z);

        /**
         * @brief isGripping
         * Check gripping status of the end-effecor
         * @return True if end-effector is closed
         */
        bool isGripping();

        /**
         * @brief grasp
         * Method to grasp an object
         */
        void close_gripper(double width, double force);

        /**
         * @brief release
         * Method to release an object
         */
        void open_gripper(double width);

        /**
         * @brief communicationTest
         * Method sends 10k empty commands to the Franka robot and checks the response
         * @return number of lost states
         */
        uint64_t communicationTest();

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

#endif // JCNSFRANKA_H
