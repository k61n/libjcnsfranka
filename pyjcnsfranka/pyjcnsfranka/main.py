from ctypes import *
import platform


mode = dict(winmode=0) if platform.python_version() >= '3.8' else dict()


class Pyjcnsfranka:
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
        self.lib.readState.argtypes = [c_void_p]
        self.lib.readState.restype = POINTER(c_double)
        result = self.lib.readState(self.obj)
        return [result[i] for i in range(10)]

    def go_home(self):
        self.lib.goHome.argtypes = [c_void_p]
        self.lib.goHome.restype = c_bool
        return self.lib.goHome(self.obj)

    def move_joints(self, joints):
        self.lib.moveJoints.argtypes = [c_void_p, POINTER(c_double)]
        self.lib.moveJoints.restype = None
        input = (c_double * len(joints))(*joints)
        return self.lib.moveJoints(self.obj, input)

    def move_relative(self, dx, dy, dz):
        self.lib.moveRelative.argtypes = [c_void_p, c_double, c_double, c_double]
        self.lib.moveRelative.restype = None
        return self.lib.moveRelative(self.obj, dx, dy, dz)

    def move_absolute(self, x, y, z):
        self.lib.moveAbsolute.argtypes = [c_void_p, c_double, c_double, c_double]
        self.lib.moveAbsolute.restype = None
        return self.lib.moveAbsolute(self.obj, x, y, z)

    def is_gripping(self):
        self.lib.isGripping.argtypes = [c_void_p]
        self.lib.isGripping.restype = c_bool
        return self.lib.isGripping(self.obj)

    def grasp(self):
        self.lib.grasp.argtypes = [c_void_p]
        self.lib.grasp.restype = None
        return self.lib.grasp(self.obj)

    def release(self):
        self.lib.release.argtypes = [c_void_p]
        self.lib.release.restype = None
        return self.lib.release(self.obj)

    def communication_test(self):
        self.lib.communicationTest.argtypes = [c_void_p]
        self.lib.communicationTest.restype = c_uint64
        return self.lib.communicationTest(self.obj)


def main():
    robot = Pyjcnsfranka('192.168.1.5')
    print(robot.read_state())
    # robot.go_home()
    # robot.move_joints([0, 0, 0, -math.pi/2, 0, math.pi, math.pi/4])
    # robot.move_relative(0, 0, 0.01)
    # robot.move_absolute(0, 0, 0.01)
    # print(robot.is_gripping())
    # robot.grasp()
    # robot.release()
    # robot.communication_test()


if __name__ == '__main__':
    main()
