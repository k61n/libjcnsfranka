
import math
from time import sleep

from pyjcnsfranka.robot import FrankaRobot

sample1 = [-0.0489999, -0.301708, 0.279]
above_magnet = [0.387569, 0.244744, 0.636882]
in_magnet = [0.387569, 0.254744, 0.401882]

if __name__ == '__main__':
    robot = FrankaRobot('192.168.1.2')
    home_pose = [0, -math.pi/4, 0, -3/4*math.pi, 0, math.pi/2, math.pi/4]
    grab_pose = [-math.pi/2, -math.pi/4, 0, -3/4*math.pi, -1.5/180*math.pi, 91/180*math.pi, 3/4*math.pi]

    # move robot to the sample side
    robot.move_joints(grab_pose, 0.1)
    robot.set_gripper_width(0.03)

    # move to sample
    pos = robot.read_pose()[7:]
    robot.move_absolute(sample1[0], sample1[1], pos[2])
    robot.move_absolute(sample1[0], sample1[1], sample1[2])
    robot.grasp(0, 20)
    for i in range(8):
        # if i == 5:
        #     robot.move_relative(-0.005, 0, 0)
        robot.move_relative(0, 0, 0.005)

    # lift up the sample
    robot.move_joints(home_pose, 0.1)
    pos = robot.read_pose()[7:]
    robot.move_absolute(pos[0], pos[1], above_magnet[2])

    # align to magnet
    robot.move_absolute(above_magnet[0], above_magnet[1], above_magnet[2])

    # get in
    robot.move_absolute(in_magnet[0], in_magnet[1], in_magnet[2])
    sleep(5)

    # lift up
    robot.move_absolute(above_magnet[0], above_magnet[1], above_magnet[2] + 0.05)

    # put sample back
    robot.move_joints(grab_pose, 0.1)
    robot.move_absolute(sample1[0], sample1[1], sample1[2] + 0.04)
    # robot.move_relative(-0.005, 0, 0)
    for i in range(8):
        # if i == 5:
        #     robot.move_relative(0.005, 0, 0)
        robot.move_relative(0, 0, -0.005)
    robot.set_gripper_width(0.1)
    robot.move_absolute(sample1[0], sample1[1], pos[2])
