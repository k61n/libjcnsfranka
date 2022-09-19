from ctypes import *
import platform


mode = dict(winmode=0) if platform.python_version() >= '3.8' else dict()


class FrankaRobot:
    """
    Python wrapper of JcnsFranka::Robot class in JcnsFranka C++ library.
    """
    def __init__(self, ip):
        self.lib = CDLL('libjcnsfranka.so', **mode)
        self.lib.init.argtypes = [c_char_p]
        self.lib.init.restype = c_void_p
        buffer = create_string_buffer(ip.encode('utf-8'))
        self.obj = self.lib.init(buffer)

    def __del__(self):
        self.lib.deinit.argtypes = [c_void_p]
        self.lib.deinit.restype = None
        self.lib.deinit(self.obj)

    def read_state(self):
        """
        Reads current joints and end-effector positions.
        :return: Current joints and end-effector positions.
        """
        self.lib.readState.argtypes = [c_void_p]
        self.lib.readState.restype = POINTER(c_double)
        result = self.lib.readState(self.obj)
        return [result[i] for i in range(10)]

    def go_home(self):
        """
        Moves the Franka robot to a homing position and resets the end-effector.
        Home position is { 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4 } [rad].
        :return: True when homing is finished.
        """
        self.lib.goHome.argtypes = [c_void_p]
        self.lib.goHome.restype = None
        self.lib.goHome(self.obj)

    def move_joints(self, joints):
        """
        Sets the Franka robot in a position that corresponds to passed joint angles.
        :param joints: joints an array with angles for the joints [rad].
        """
        self.lib.moveJoints.argtypes = [c_void_p, POINTER(c_double)]
        self.lib.moveJoints.restype = None
        input = (c_double * len(joints))(*joints)
        self.lib.moveJoints(self.obj, input)

    def move_relative(self, dx, dy, dz):
        """
        Perform relative motion of the Franka robot in cartesian space.
        :param dx: relative displacement in X axis [m].
        :param dy: relative displacement in Y axis [m].
        :param dz: relative displacement in Z axis [m].
        """
        self.lib.moveRelative.argtypes = [c_void_p, c_double, c_double, c_double]
        self.lib.moveRelative.restype = None
        self.lib.moveRelative(self.obj, dx, dy, dz)

    def move_absolute(self, x, y, z):
        """
        Perform motion of the Franka robot to a given coordinate in cartesian space.
        :param x: target X coordinate in cartesian space [m].
        :param y: coordinate in cartesian space [m].
        :param z: coordinate in cartesian space [m].
        """
        self.lib.moveAbsolute.argtypes = [c_void_p, c_double, c_double, c_double]
        self.lib.moveAbsolute.restype = None
        self.lib.moveAbsolute(self.obj, x, y, z)

    def is_gripping(self):
        """
        Check gripping status of the end-effecor.
        :return: True if end-effector is closed.
        """
        self.lib.isGripping.argtypes = [c_void_p]
        self.lib.isGripping.restype = c_bool
        self.lib.isGripping(self.obj)

    def grasp(self):
        """
        Method to grasp an object.
        """
        self.lib.grasp.argtypes = [c_void_p]
        self.lib.grasp.restype = None
        self.lib.grasp(self.obj)

    def release(self):
        """
        Method to release an object.
        """
        self.lib.release.argtypes = [c_void_p]
        self.lib.release.restype = None
        self.lib.release(self.obj)

    def communication_test(self):
        """
        Method sends 10k empty commands to the Franka robot and checks the response.
        :return: number of lost states.
        """
        self.lib.communicationTest.argtypes = [c_void_p]
        self.lib.communicationTest.restype = c_uint64
        return self.lib.communicationTest(self.obj)

    def read_error(self):
        self.lib.error.argtypes = [c_void_p]
        self.lib.error.restype = c_char_p
        return self.lib.error(self.obj).decode('utf-8')
