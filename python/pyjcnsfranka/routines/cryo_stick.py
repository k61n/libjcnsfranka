
import math
from pyjcnsfranka.robot import FrankaRobot


def load():
    robot = FrankaRobot('192.168.1.2')
    robot.set_load(0, [0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0])
    robot.go_home()
    robot.move_relative(-0.005, 0, 0)
    pos = robot.read_state()[:7]
    pos[0] = 76 / 180 * math.pi
    robot.move_joints(pos, 0.1)
    robot.move_relative(0, 0, -0.584)
    robot.set_load(1.461, [0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0])
    robot.close_gripper(0, 70)
    robot.move_relative(0, 0, 0.4)
    pos = robot.read_state()[:7]
    pos[0] = 60 / 180 * math.pi
    robot.move_joints(pos, 0.1)
    robot.move_relative(0, 0, 0.3)
    pos = robot.read_state()[:7]
    pos[0] = 0
    robot.move_joints(pos, 0.1)
    robot.move_relative(0.01, 0, 0)
    for i in range(8):
        robot.move_relative(0, 0, -0.1)
    robot.move_relative(0, 0, -0.03)
    robot.move_relative(-0.003, 0, 0)
    robot.move_relative(0, 0, -0.005)
    robot.move_gripper(0.1)
    robot.set_load(0, [0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0])
    robot.move_relative(0, 0, 0.1)
    robot.go_home()


def unload():
    robot = FrankaRobot('192.168.1.2')
    robot.set_load(0, [0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0])
    robot.go_home()
    robot.move_relative(0, 0, -0.7325)
    robot.set_load(1.461, [0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0])
    robot.close_gripper(0, 70)
    robot.move_relative(0, 0, 0.85, 2)
    pos = robot.read_state()[:7]
    pos[0] = 60 / 180 * math.pi
    robot.move_joints(pos, 0.1)
    robot.move_relative(0, 0, -0.2)
    pos = robot.read_state()[:7]
    pos[0] = 76 / 180 * math.pi
    robot.move_joints(pos, 0.1)
    for i in range(4):
        robot.move_relative(0, 0, -0.1)
    robot.move_relative(0, 0, -0.09)
    robot.move_gripper(0.1)
    robot.set_load(0, [0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0])
    robot.move_relative(0, 0, 0.1)
    robot.go_home()
