
import math

from pyjcnsfranka.robot import FrankaRobot


flange_cryo = [0.451938, 0.0160606, 0.435664]
flange_stor = [0.271937, -0.20894, 0.452]
sample_cryo = [0.454166, 0.0183907, 0.282]
sample_stor = [0.260127, 0.121152, 0.181918]


if __name__ == '__main__':
    robot = FrankaRobot('192.168.1.2')
    robot.go_home()
    pos = robot.read_state()[7:]
    robot.move_gripper(0.03)

    # go grab the flange
    pos = robot.read_state()[7:]
    robot.move_absolute(flange_cryo[0], flange_cryo[1], pos[2])
    robot.move_absolute(flange_cryo[0], flange_cryo[1], flange_cryo[2])
    robot.close_gripper(0, 70)

    # lift up flange to the appropriate height
    robot.move_absolute(flange_cryo[0], flange_cryo[1], flange_stor[2] + 0.12)

    # now the flange has to travel to the store position
    robot.move_absolute(flange_stor[0] - 0.13, flange_stor[1], flange_stor[2] + 0.12)
    robot.move_absolute(flange_stor[0] - 0.13, flange_stor[1], flange_stor[2] + 0.02)
    robot.move_absolute(flange_stor[0], flange_stor[1], flange_stor[2] + 0.02)
    robot.move_absolute(flange_stor[0], flange_stor[1], flange_stor[2])
    robot.move_gripper(0.03)
    robot.move_relative(0, 0, 0.02)

    # let's grab the sample
    robot.move_absolute(sample_stor[0], sample_stor[1], pos[2])
    robot.move_absolute(sample_stor[0], sample_stor[1], sample_stor[2])
    robot.close_gripper(0, 70)
    robot.move_linear(0, 0, 0.02)
    robot.move_absolute(sample_stor[0], sample_stor[1], pos[2])

    # place it above the cryo and put down, then lift up the hand
    robot.move_absolute(sample_cryo[0], sample_cryo[1], pos[2])
    robot.move_absolute(sample_cryo[0], sample_cryo[1], sample_cryo[2] + 0.02)
    robot.move_linear(0, 0, -0.02)
    robot.move_gripper(0.03)
    robot.move_absolute(sample_cryo[0], sample_cryo[1], pos[2])

    # cover the flange
    robot.move_absolute(flange_stor[0], flange_stor[1], pos[2])
    robot.move_absolute(flange_stor[0], flange_stor[1], flange_stor[2])
    robot.close_gripper(0, 70)
    robot.move_absolute(flange_stor[0], flange_stor[1], flange_stor[2] + 0.02)
    robot.move_absolute(flange_stor[0] - 0.13, flange_stor[1], flange_stor[2] + 0.02)
    robot.move_absolute(flange_stor[0] - 0.13, flange_stor[1], flange_stor[2] + 0.12)
    robot.move_absolute(flange_cryo[0], flange_cryo[1], flange_stor[2] + 0.12)
    robot.move_absolute(flange_cryo[0], flange_cryo[1], flange_cryo[2])
    robot.move_gripper(0.3)
    robot.move_relative(0, 0, 0.02)
    robot.go_home()

