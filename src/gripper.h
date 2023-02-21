#ifndef GRIPPER_H
#define GRIPPER_H


#include "liborl/liborl.h"


namespace JcnsFranka {
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
         * @return Checks gripping status
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
         * @brief move_gripper
         * Opens gripper to a desired width
         * @param width
         */
        void move_gripper(double width);

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
         * @brief maxWidth
         * Once gripper is initialized its max width is stored
         */
        double maxWidth;
    };
}


#endif // GRIPPER_H
