
from pyjcnsfranka.robot import FrankaRobot


flange_cryo = [0.4555, 0.013, 0.434]
flange_stor = [0.271, -0.2095, 0.453]
sample_cryo = [0.454, 0.0155, 0.283]
sample_stor = [0.2615, 0.120, 0.183]


def load():
    robot = FrankaRobot('192.168.1.2')
    robot.reference()
    pos = robot.read_pose()[7:]
    robot.set_gripper_width(0.04)

    # go grab the flange
    pos = robot.read_pose()[7:]
    robot.move_absolute(flange_cryo[0], flange_cryo[1], pos[2])
    robot.move_absolute(flange_cryo[0], flange_cryo[1], flange_cryo[2])
    robot.grasp(0, 70)

    # lift up flange to the appropriate height
    robot.move_absolute(flange_cryo[0], flange_cryo[1], flange_stor[2] + 0.12)

    # now the flange has to travel to the store position
    robot.move_absolute(flange_stor[0] - 0.13, flange_stor[1], flange_stor[2] + 0.12)
    robot.move_absolute(flange_stor[0] - 0.13, flange_stor[1], flange_stor[2] + 0.02)
    robot.move_absolute(flange_stor[0], flange_stor[1], flange_stor[2] + 0.02)
    robot.move_absolute(flange_stor[0], flange_stor[1], flange_stor[2] + 0.002)
    robot.set_gripper_width(0.04)
    robot.move_relative(0, 0, 0.02)

    # let's grab the sample
    robot.move_absolute(sample_stor[0], sample_stor[1], pos[2])
    robot.move_absolute(sample_stor[0], sample_stor[1], sample_stor[2])
    robot.grasp(0, 70)
    robot.move_linear(0, 0, 0.02)
    robot.move_absolute(sample_stor[0], sample_stor[1], pos[2])

    # place it above the cryo and put down, then lift up the hand
    robot.move_absolute(sample_cryo[0], sample_cryo[1], pos[2])
    robot.move_absolute(sample_cryo[0], sample_cryo[1], sample_cryo[2] + 0.02)
    robot.move_linear(0, 0, -0.02)
    robot.set_gripper_width(0.04)
    robot.move_absolute(sample_cryo[0], sample_cryo[1], pos[2])

    # cover the flange
    robot.move_absolute(flange_stor[0], flange_stor[1], pos[2])
    robot.move_absolute(flange_stor[0], flange_stor[1], flange_stor[2])
    robot.grasp(0, 70)
    robot.move_absolute(flange_stor[0], flange_stor[1], flange_stor[2] + 0.02)
    robot.move_absolute(flange_stor[0] - 0.13, flange_stor[1], flange_stor[2] + 0.02)
    robot.move_absolute(flange_stor[0] - 0.13, flange_stor[1], flange_stor[2] + 0.12)
    robot.move_absolute(flange_cryo[0], flange_cryo[1], flange_stor[2] + 0.12)
    robot.move_absolute(flange_cryo[0], flange_cryo[1], flange_cryo[2] + 0.002)
    robot.set_gripper_width(0.04)
    robot.move_relative(0, 0, 0.02)
    robot.reference()


def unload():
    robot = FrankaRobot('192.168.1.2')
    robot.reference()
    # fingers of the gripper should be in this position
    pos = robot.read_pose()[7:]
    robot.set_gripper_width(0.04)

    # go grab the flange
    robot.move_absolute(flange_cryo[0], flange_cryo[1], pos[2])
    robot.move_absolute(flange_cryo[0], flange_cryo[1], flange_cryo[2])
    robot.grasp(0, 70)

    # lift up flange to the appropriate height
    robot.move_absolute(flange_cryo[0], flange_cryo[1], flange_cryo[2] + 0.12)

    # now the flange has to travel to the store position
    robot.move_absolute(flange_stor[0] - 0.13, flange_stor[1], flange_cryo[2] + 0.12)
    robot.move_absolute(flange_stor[0] - 0.13, flange_stor[1], flange_stor[2] + 0.02)
    robot.move_absolute(flange_stor[0], flange_stor[1], flange_stor[2] + 0.02)
    robot.move_absolute(flange_stor[0], flange_stor[1], flange_stor[2] + 0.002)
    robot.set_gripper_width(0.04)
    robot.move_absolute(flange_stor[0], flange_stor[1], pos[2])

    # grab sample from cryostat
    robot.set_gripper_width(0.04)
    robot.move_absolute(sample_cryo[0], sample_cryo[1], pos[2])
    robot.move_absolute(sample_cryo[0], sample_cryo[1], sample_cryo[2])
    robot.grasp(0, 70)
    robot.move_linear(0, 0, 0.02)
    robot.move_absolute(sample_cryo[0], sample_cryo[1], pos[2])

    # move to storage position
    robot.move_absolute(sample_stor[0], sample_stor[1], pos[2])
    robot.move_absolute(sample_stor[0], sample_stor[1], sample_stor[2] + 0.02)
    robot.move_linear(0, 0, -0.02)
    robot.set_gripper_width(0.04)
    robot.move_absolute(sample_stor[0], sample_stor[1], pos[2])

    # cover the flange
    robot.move_absolute(flange_stor[0], flange_stor[1], pos[2])
    robot.move_absolute(flange_stor[0], flange_stor[1], flange_stor[2])
    robot.grasp(0, 70)
    robot.move_absolute(flange_stor[0], flange_stor[1], flange_stor[2] + 0.02)
    robot.move_absolute(flange_stor[0] - 0.13, flange_stor[1], flange_stor[2] + 0.02)
    robot.move_absolute(flange_stor[0] - 0.13, flange_stor[1], flange_cryo[2] + 0.12)
    robot.move_absolute(flange_cryo[0], flange_cryo[1], flange_cryo[2] + 0.12)
    robot.move_absolute(flange_cryo[0], flange_cryo[1], flange_cryo[2] + 0.002)
    robot.set_gripper_width(0.05)
    robot.reference()
