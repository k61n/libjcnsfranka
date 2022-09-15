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
     * @brief The Robot class
     * Simple wrapper around orl::Robot class
     */
    class Robot
    {
    public:
        /**
         * @brief Robot
         * Establishes the connection to the Franka robot.
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
        bool goHome();

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
        void grasp();

        /**
         * @brief release
         * Method to release an object
         */
        void release();

        /**
         * @brief communicationTest
         * Method sends 10k empty commands to the Franka robot and checks the response
         * @return number of lost states
         */
        uint64_t communicationTest();

    private:
        /**
         * @brief robot
         * orl::Robot instance
         */
        orl::Robot *robot;

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
    };
}

#endif // JCNSFRANKA_H
