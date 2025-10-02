
from ctypes import *


class FrankaPose(Structure):
    _fields_ = [("joints", c_double * 7),
                ("xyz", c_double * 3),
                ("rpy", c_double * 3)]

    def to_dict(self):
        return {'joints': list(self.joints),
                'xyz': list(self.xyz),
                'rpy': list(self.rpy)}


class FrankaLoad(Structure):
    _fields_ = [('mass', c_double),
                ('F_x_Cload', c_double * 3),
                ('load_inertia', c_double * 9)]

    def to_dict(self):
        return {'mass': float(self.mass),
                'F_x_Cload': list(self.F_x_Cload),
                'load_inertia': list(self.load_inertia)}


class FrankaRobot:
    """
    Python wrapper of JcnsFranka::Robot class in JcnsFranka C++ library.
    """
    def __init__(self, ip, realtime_config=False):

        self.lib = CDLL('libjcnsfranka.so.0.4.6', winmode=0)
        self.lib.init.argtypes = [c_char_p, c_int8]
        self.lib.init.restype = c_void_p
        buffer = create_string_buffer(ip.encode('utf-8'))
        self.obj = self.lib.init(buffer, int(not realtime_config))

        self.lib.deinit.argtypes = [c_void_p]
        self.lib.deinit.restype = None

        self.lib.reference.argtypes = [c_void_p]
        self.lib.reference.restype = None

        self.lib.read_mode.argtypes = [c_void_p]
        self.lib.read_mode.restype = c_int8

        self.lib.read_pose.argtypes = [c_void_p]
        self.lib.read_pose.restype = FrankaPose

        self.lib.set_pose.argtypes = [c_void_p, c_double, c_double, c_double,
                                      c_double, c_double, c_double, c_double]
        self.lib.set_pose.restype = None

        self.lib.read_load.argtypes = [c_void_p]
        self.lib.read_load.restype = FrankaLoad

        self.lib.set_load.argtypes = [c_void_p, c_double, POINTER(c_double), POINTER(c_double)]
        self.lib.set_load.restype = None

        self.lib.read_csr.argtypes = [c_void_p]
        self.lib.read_csr.restype = c_double

        self.lib.move_joints.argtypes = [c_void_p, POINTER(c_double), c_double]
        self.lib.move_joints.restype = None

        self.lib.move_relative.argtypes = [c_void_p, c_double, c_double,
                                           c_double, c_double]
        self.lib.move_relative.restype = None

        self.lib.move_linear.argtypes = [c_void_p, c_double, c_double, c_double]
        self.lib.move_linear.restype = None

        self.lib.move_absolute.argtypes = [c_void_p, c_double, c_double,
                                           c_double, c_double]
        self.lib.move_absolute.restype = None

        self.lib.is_moving.argtypes = [c_void_p]
        self.lib.is_moving.restype = c_bool

        self.lib.read_gripper_width.argtypes = [c_void_p]
        self.lib.read_gripper_width.restype = c_double

        self.lib.set_gripper_width.argtypes = [c_void_p, c_double]
        self.lib.set_gripper_width.restype = None

        self.lib.read_gripper_force.argtypes = [c_void_p]
        self.lib.read_gripper_force.restype = c_double

        self.lib.grasp.argtypes = [c_void_p, c_double, c_double]
        self.lib.grasp.restype = None

        self.lib.is_gripping.argtypes = [c_void_p]
        self.lib.is_gripping.restype = c_bool

        self.lib.read_error.argtypes = [c_void_p]
        self.lib.read_error.restype = c_char_p

        self.lib.reset_error.argtypes = [c_void_p]
        self.lib.reset_error.restype = None

        self.lib.is_in_error_mode.argtypes = [c_void_p]
        self.lib.is_in_error_mode.restype = c_bool

        if self.is_in_error_mode():
            raise Exception(self.read_error())

    def __del__(self):
        self.lib.deinit(self.obj)

    def reference(self):
        """
        Moves the Franka robot to homing position and resets the end-effector.
        Home position: { 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4 } [rad].
        :return: True when homing is finished.
        """
        self.lib.reference(self.obj)
        if self.is_in_error_mode():
            raise Exception(self.read_error())

    def read_mode(self):
        """
        Reads current robot mode.
        :return: robot mode.
        """
        result = self.lib.read_mode(self.obj)
        if self.is_in_error_mode():
            raise Exception(self.read_error())
        return result

    def read_pose(self):
        """
        Reads current joints and end-effector positions.
        :return: current joints and end-effector positions.
        """
        result = self.lib.read_pose(self.obj)
        if self.is_in_error_mode():
            raise Exception(self.read_error())
        return result.to_dict()

    def set_pose(self, x, y, z, roll, pitch, yaw, t=0):
        """
        Set new pose to the end-effector.
        :param x: cartesian coordinate of end-effector in X-axis [m]
        :param y: cartesian coordinate of end-effector in Y-axis [m]
        :param z: cartesian coordinate of end-effector in Z-axis [m]
        :param roll: roll angle of end-effector [rad]
        :param pitch: pitch angle of end-effector [rad]
        :param yaw: yaw angle of end-effector [rad]
        :param t: time to finish the motion [s]
        """
        result = self.lib.set_pose(self.obj, x, y, z, roll, pitch, yaw, t)
        if self.is_in_error_mode():
            raise Exception(self.read_error())

    def read_load(self):
        """
        Reads current end-effector load configuration as a dict with keys:
            * load_mass - mass of the load in [kg]
            * F_x_Cload - translation from flange to center of mass of load [m]
            * load_inertia - inertia matrix in [kg*m2], column-major
        :return: robot load.
        """
        result = self.lib.read_load(self.obj)
        if self.is_in_error_mode():
            raise Exception(self.read_error())
        return result.to_dict()

    def set_load(self, mass, F_x_Cload, load_inertia):
        """
        Sets dynamic parameters of a payload.
        :param mass: mass of the load in [kg].
        :param F_x_Cload: translation from flange to center of mass of load in [m].
        :param load_inertia: Inertia matrix in [kg*m2], column-major.
        """
        arg2 = (c_double * len(F_x_Cload))(*F_x_Cload)
        arg3 = (c_double * len(load_inertia))(*load_inertia)
        self.lib.set_load(self.obj, mass, arg2, arg3)
        if self.is_in_error_mode():
            raise Exception(self.read_error())

    def read_csr(self):
        """
        Reads current control command success rate.
        :return: Percentage of the last 100 control commands that were
            successfully received by the robot.
        """
        result = self.lib.read_csr(self.obj)
        if self.is_in_error_mode():
            raise Exception(self.read_error())
        return result

    def move_joints(self, joints, speed_factor):
        """
        Sets the robot in a position that corresponds to passed joint angles.
        :param joints: an array with angles for the joints [rad].
        :param speed_factor: fraction of max joint speed [a.u.].
        """
        j = (c_double * len(joints))(*joints)
        self.lib.move_joints(self.obj, j, speed_factor)
        if self.is_in_error_mode():
            raise Exception(self.read_error())

    def move_relative(self, dx, dy, dz, t=0):
        """
        Performs relative motion of the Franka robot in cartesian space.
        :param dx: relative displacement in X axis [m].
        :param dy: relative displacement in Y axis [m].
        :param dz: relative displacement in Z axis [m].
        :param t: time to complete the movement [s].
        """
        self.lib.move_relative(self.obj, dx, dy, dz, t)
        if self.is_in_error_mode():
            raise Exception(self.read_error())

    def move_linear(self, dx, dy, dz):
        """
        Performs relative motion in cartesian space but ensures its linear
        trajectory. As a consequence this movement might be particularly slow.
        :param dx: relative displacement in X axis [m].
        :param dy: relative displacement in Y axis [m].
        :param dz: relative displacement in Z axis [m].
        """
        self.lib.move_linear(self.obj, dx, dy, dz)
        if self.is_in_error_mode():
            raise Exception(self.read_error())

    def move_absolute(self, x, y, z, t=0):
        """
        Perform motion of the robot to a given coordinate in cartesian space.
        :param x: target X coordinate in cartesian space [m].
        :param y: target Y coordinate in cartesian space [m].
        :param z: target Z coordinate in cartesian space [m].
        :param t: time to complete the movement [s].
        """
        self.lib.move_absolute(self.obj, x, y, z, t)
        if self.is_in_error_mode():
            raise Exception(self.read_error())

    def is_moving(self):
        """
        Reads if robot is currently moving.
        :return: true if robot is moving.
        """
        result = self.lib.is_moving(self.obj)
        if self.is_in_error_mode():
            raise Exception(self.read_error())
        return result

    def read_gripper_width(self):
        """
        Reads current gripper opening width.
        :return: current gripper opening width [m].
        """
        result = self.lib.read_gripper_width(self.obj)
        if self.is_in_error_mode():
            raise Exception(self.read_error())
        return result

    def set_gripper_width(self, width):
        """
        Opens gripper to a desired width.
        :param width: width.
        """
        self.lib.set_gripper_width(self.obj, width)
        if self.is_in_error_mode():
            raise Exception(self.read_error())

    def read_gripper_force(self):
        """
        Returns last applied gripper force.
        :return: last applied gripper force [N].
        """
        result = self.lib.read_gripper_force(self.obj)
        if self.is_in_error_mode():
            raise Exception(self.read_error())
        return result

    def grasp(self, width, force):
        """
        Method to grasp an object with force.
        :param width: width.
        :param force: force.
        """
        self.lib.grasp(self.obj, width, force)
        if self.is_in_error_mode():
            raise Exception(self.read_error())

    def is_gripping(self):
        """
        Checks gripping status of the end-effector.
        :return: True if end-effector is closed.
        """
        result = self.lib.is_gripping(self.obj)
        if self.is_in_error_mode():
            raise Exception(self.read_error())
        return result

    def read_error(self):
        """
        Class method to return recent error if any.
        :return: error message obtained from libfranka exception.
        """
        return self.lib.read_error(self.obj).decode('utf-8')

    def reset_error(self):
        """
        Resets current error.
        """
        self.lib.reset_error(self.obj)

    def is_in_error_mode(self):
        """
        Class method to check if the robot is in error state.
        :return: True if is in error state.
        """
        return self.lib.is_in_error_mode(self.obj)


def comtest(ip, realtime_config, limit_rate, cutoff_frequency):
    """
    Sends 10k empty commands and controls the response.
    :param ip: name or ip address of the robot.
    :param realtime_config: enforce True or ignore False the realtime
        configuration.
    :param limit_rate: whether rate limiters work or not.
    :param cutoff_frequency: 1000 or low-pass filtered 100 [Hz].
    :return: number of lost states.
    """
    lib = CDLL('libjcnsfranka.so.0.4.6', winmode=0)
    buffer = create_string_buffer(ip.encode('utf-8'))
    lib.communication_test.argtypes = [c_char_p, c_int, c_bool, c_double]
    lib.communication_test.restype = c_uint64
    lib.communication_test(buffer, int(not realtime_config), limit_rate,
                           cutoff_frequency)
