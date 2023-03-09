
import math
from time import sleep

from pyjcnsfranka.robot import FrankaRobot


sample1 = [-0.107702, -0.302816, 0.281]
#sample2
bottom = [0.3, 0, -0.2]

if __name__ == '__main__':
    robot = FrankaRobot('192.168.1.2')

    # move robot to the sample side
    robot.go_home()
    robot.move_gripper(0.03)
    joints = [-math.pi/2, -math.pi/4, 0, -3/4 * math.pi, 0, math.pi/2, math.pi/4]
    robot.move_joints(joints)

    # move to sample
    pos = robot.read_state()[7:]
    robot.move_absolute(sample1[0], sample1[1], pos[2])
    robot.move_absolute(sample1[0], sample1[1], sample1[2])
    robot.close_gripper(0, 20)
    for i in range(8):
        robot.move_relative(0, 0, 0.005)

    # turn robot pi/2 before going down for demo
    joints = robot.read_state()[:7]
    joints[0] = 0
    robot.move_joints(joints)
    robot.move_absolute(bottom[0], bottom[1], bottom[2])
    sleep(5)
    # go back up
    robot.move_joints(joints)

    # place sample back
    joints = [0, -math.pi/4, 0, -3/4 * math.pi, 0, math.pi/2, math.pi/4]
    robot.move_joints(joints)
    joints[0] = -math.pi/2
    robot.move_joints(joints)
    pos = robot.read_state()[7:]
    robot.move_absolute(sample1[0], sample1[1], pos[2])
    robot.move_absolute(sample1[0], sample1[1], sample1[2] + 0.04)
    for i in range(8):
        robot.move_relative(0, 0, -0.005)
    robot.move_gripper(0.03)
    robot.move_absolute(sample1[0], sample1[1], pos[2])
    joints[0] = 0
    robot.move_joints(joints)
