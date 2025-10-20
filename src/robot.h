#ifndef ROBOT_H
#define ROBOT_H


#include "gripper.h"


namespace JcnsFranka {

    /**
     * @brief The Pose class
     * describes pose as joint angles, as cartesian coordinates,
     * and as roll, pitch and yaw angles of the end-effector.
     * @param joints: joint angles
     * @param xyz: cartesian coordinates in X, Y and Z axes
     * @param rpy: roll, pitch and yaw angles of the end-effector
     */
    struct Pose {
        std::array<double, 7> joints;
        std::array<double, 3> xyz;
        std::array<double, 3> rpy;
    };

    /**
     * @brief The Load class
     * describes mass, translation from flange to center of mass of load, and
     * inertia matrix.
     * @param mass: mass of the load in [kg]
     * @param F_x_Cload: translation from flange to center of mass of load in [m]
     * @param load_inertia: inertia matrix in [kg*m2], column-major
     */
    struct Load {
        double mass;
        std::array<double, 3> F_x_Cload;
        std::array<double, 9> load_inertia;
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
         * @param realtime_config if set to Enforce, an exception will be thrown
         * if realtime priority cannot be set when required.
         * Setting realtime_config to Ignore disables this behavior.
         */
        explicit Robot(char *ip, franka::RealtimeConfig realtime_config = franka::RealtimeConfig::kIgnore);
        explicit Robot(char *ip, int realtime_config);
        ~Robot();

        /**
         * @brief reference
         * Moves the Franka robot to homing position and resets the end-effector
         * Home position: { 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4 } [rad]
         * @return True when homing is finished
         */
        void reference();

        /**
         * @brief read_mode
         * @return Current robot mode
         */
        franka::RobotMode read_mode() const;

        /**
         * @brief read_pose
         * Reads current joints and end-effector positions
         * @return current joints and end-effector positions
         */
        Pose read_pose();

        /**
         * @brief set_pose
         * Set new pose to the end-effector
         * @param x cartesian coordinate in X-axis [m]
         * @param y cartesian coordinate in Y-axis [m]
         * @param z cartesian coordinate in Z-axis [m]
         * @param roll roll angle [rad]
         * @param pitch pitch angle [rad]
         * @param yaw yaw angle [rad]
         * @param t time to finish the motion [s]
         */
        void set_pose(double x, double y, double z, double roll, double pitch, double yaw, double t=0);

        /**
         * @brief read_load
         * Reads current load configuration
         * @return current load configuration
         */
        Load read_load() const;

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
         * @brief read_csr
         * Reads current control command success rate
         * @return Percentage of the last 100 control commands that were
         * successfully received by the robot
         */
        double read_csr() const;

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
         * @param t time to complete the movement [s]
         */
        void move_relative(double dx, double dy, double dz, double t=0);

        /**
         * @brief move_linear
         * Performs relative motion in cartesian space but ensures its
         * linear trajectory. As a consequence this movement might be
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
         * @param t time to complete the movement [s]
         */
        void move_absolute(double x, double y, double z, double t=0);

        /**
         * @brief is_moving
         * Reads if robot is currently moving
         * @return true if robot is moving
         */
        bool is_moving() const;

        /**
         * @brief read_gripper_width
         * Reads current gripper opening width [m]
         * @return current gripper opening width [m]
         */
        double read_gripper_width();

        /**
         * @brief set_gripper_width
         * Opens gripper to a desired width
         * @param width
         */
        void set_gripper_width(double width);

        /**
         * @brief read_gripper_force
         * Returns last applied gripper force [N]
         * @return last gripper opening width [N]
         */
        double read_gripper_force() const;

        /**
         * @brief grasp
         * Method to grasp an object with force
         * @param width
         * @param force
         */
        void grasp(double width, double force);

        /**
         * @brief is_gripping
         * Checks gripping status of the end-effector
         * @return True if end-effector is closed
         */
        bool is_gripping();

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

        /**
         * @brief is_in_error_mode
         * Class method to check if the robot is in error state
         * @return True if is in error state
         */
        bool is_in_error_mode();

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
         * @brief state
         * Stores current robot state
         */
        franka::RobotState state{};

        /**
         * @brief moving
         * Stores if robot is in motion
         */
        bool moving = false;

        /**
         * @brief gripperwidth
         * Last set gripper force.
         */
        double gripperforce = 0;

        /**
         * @brief gripperwidth
         * Last set gripper width.
         */
        double gripperwidth = 0;

        /**
         * @brief frankaerror
         * Description of error from libfranka
         */
        std::string frankaerror;

        /**
         * @brief calculate_t
         * Calculates time to finish the movement
         * @param x relative displacement in X axis [m]
         * @param y relative displacement in Y axis [m]
         * @param z relative displacement in Z axis [m]
         * @return time to finish the motion [s]
         */
        double calculate_t(double x, double y, double z) const;
    };
}


#endif // ROBOT_H
