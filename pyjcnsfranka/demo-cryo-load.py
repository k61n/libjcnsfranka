
from pyjcnsfranka.robot import FrankaRobot


flange_cryo = [0.443, -0.0025, 0.33]
flange_stor = [0.362, -0.206, 0.454]
sample_cryo = [0.446, -0.001, 0.182]
sample_stor = [0.336, 0.12, 0.182]


if __name__ == '__main__':
    robot = FrankaRobot('192.168.1.2')
    robot.go_home()

    # go grab the flange
    pos = robot.read_state()[7:]
    robot.move_absolute(flange_cryo[0], flange_cryo[1], pos[2])
    robot.move_absolute(flange_cryo[0], flange_cryo[1], flange_cryo[2])
    robot.move_gripper(0.01)
    robot.close_gripper(0, 70)

    # lift up flange to the appropriate height
    robot.move_absolute(flange_cryo[0], flange_cryo[1], flange_stor[2] + 0.02)

    # now the flange has to travel to the store position
    robot.move_absolute(flange_cryo[0] - 0.2, flange_cryo[1], flange_stor[2] + 0.02)
    robot.move_absolute(flange_cryo[0] - 0.2, flange_stor[1], flange_stor[2] + 0.02)
    robot.move_absolute(flange_stor[0], flange_stor[1], flange_stor[2] + 0.02)
    robot.move_absolute(flange_stor[0], flange_stor[1], flange_stor[2])
    robot.move_gripper(0.1)
    robot.move_relative(0, 0, 0.02)

    robot.go_home()
    # let's grab the sample
    pos = robot.read_state()[7:]
    robot.move_absolute(sample_stor[0], sample_stor[1], pos[2])
    robot.move_absolute(sample_stor[0], sample_stor[1], sample_stor[2])
    robot.move_gripper(0.01)
    robot.close_gripper(0, 70)
    robot.move_linear(0, 0, 0.02)
    robot.move_absolute(sample_stor[0], sample_stor[1], pos[2])

    # place it above the cryo and put down, then lift up the hand
    robot.move_absolute(sample_cryo[0], sample_cryo[1], pos[2])
    robot.move_absolute(sample_cryo[0], sample_cryo[1], sample_cryo[2] + 0.02)
    robot.move_linear(0, 0, -0.02)
    robot.move_gripper(0.1)
    robot.move_absolute(sample_cryo[0], sample_cryo[1], pos[2])

    # cover the flange
    robot.move_absolute(flange_stor[0], flange_stor[1], pos[2])
    robot.move_absolute(flange_stor[0], flange_stor[1], flange_stor[2])
    robot.move_gripper(0.01)
    robot.close_gripper(0, 70)
    robot.move_absolute(flange_stor[0], flange_stor[1], flange_stor[2] + 0.02)
    robot.move_absolute(flange_stor[0] - 0.1, flange_stor[1], flange_stor[2] + 0.02)
    robot.move_absolute(flange_stor[0] - 0.1, flange_cryo[1], flange_stor[2] + 0.02)
    robot.move_absolute(flange_cryo[0], flange_cryo[1], flange_stor[2] + 0.02)
    robot.move_absolute(flange_cryo[0], flange_cryo[1], flange_cryo[2])
    robot.move_gripper(0.1)
    robot.go_home()

