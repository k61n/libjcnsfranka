#ifndef GRIPPER_H
#define GRIPPER_H

#include "liborl/liborl.h"

namespace JcnsFranka 
{
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
        explicit Gripper(const std::string& ip);
        ~Gripper();

        /**
         * @brief reference
         * References the gripper
         */
        void reference();

        /**
         * @brief max_width
         * @return max width of the gripper
         */
        double max_width() const;

        /**
         * @brief read_width
         * @return width of the actual gripper state
         */
        double read_width();

        /**
         * @brief set_gripper_width
         * Opens gripper to a desired width
         * @param width
         */
        void set_width(double width);

        /**
         * @brief grasp
         * Closes the gripper applying the force
         * @param width
         * @param force
         */
        void grasp(double width, double force);

        /**
         * @brief is_gripping
         * @return Checks gripping status
         * @return True is gripping
         */
        bool is_gripping();

        /**
         * @brief stop
         * Stops the movement
         */
        void stop();

    private:
        /**
         * @brief gripper
         * Gripper instance from libfranka
         */
        franka::Gripper *gripper;

        /**
         * @brief maxWidth [m]
         * Once gripper is initialized its max width is stored
         */
        double maxWidth = 0.081;

        /**
         * @brief maxSpeed [m/s]
         * Once gripper is initialized its max speed is stored
         */
        double maxSpeed = 0.05;
    };
}

#endif // GRIPPER_H
