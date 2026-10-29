#
# physics.py - pretends to be the real world when running in the simulator.
#
# Students don't need to change this file. When you run the simulator, this
# code fakes what the motors and sensors would do on a real robot, so you can
# watch the robot drive around the field on your computer.
#
# Documentation: https://robotpy.readthedocs.io/projects/pyfrc/en/stable/physics.html
#

import math
import random

import wpimath
from pyfrc.physics.core import PhysicsInterface
from rev import SparkFlexSim, SparkMaxSim
from wpilib import RobotController
from wpimath.controller import PIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState
from wpimath.system.plant import DCMotor

from robot import Robot
from swerve_drive import SwerveModule


class SwerveModuleSim:
    """Fakes the motors of one swerve module."""

    def __init__(self, module: SwerveModule):
        self._driveSim = SparkFlexSim(module.drive_motor, DCMotor.NEO(1))
        self._turnSim = SparkMaxSim(module.turn_motor, DCMotor.NEO(1))
        self._angleOffset = module.angle_offset

        # Limits how fast the wheel can speed up, to fake inertia (5 G)
        self._driveRateLimiter = SlewRateLimiter(9.8 * 5)

        # Fakes how fast the steering motor can turn the wheel
        self._turnController = PIDController(31.0, 0, 0)
        self._turnController.enableContinuousInput(-math.pi, math.pi)

        # Start each wheel pointing in a random direction, like a real robot
        self._turnSim.setPosition(random.uniform(-math.pi, math.pi))

    def update(self, vbus: float, dt: float) -> SwerveModuleState:
        driveVelocity = self._driveRateLimiter.calculate(self._driveSim.getSetpoint())
        self._driveSim.iterate(driveVelocity, vbus, dt)

        self._turnController.setSetpoint(self._turnSim.getSetpoint())
        turnVelocity = self._turnController.calculate(wpimath.angleModulus(self._turnSim.getPosition()))
        self._turnSim.iterate(turnVelocity, vbus, dt)

        return SwerveModuleState(
            self._driveSim.getVelocity(),
            Rotation2d(self._turnSim.getPosition() - self._angleOffset))


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: Robot):
        self._robot = robot
        self._physics = physics_controller

        drive = robot.drive
        self._kinematics = drive.kinematics
        self._modules = (
            SwerveModuleSim(drive.front_left),
            SwerveModuleSim(drive.front_right),
            SwerveModuleSim(drive.rear_left),
            SwerveModuleSim(drive.rear_right),
        )
        self._gyroSim = drive.gyro.sim_state

        # The launcher flywheel speeds up toward whatever speed was asked for,
        # taking a little over a second to get from stopped to full speed
        self._launcherSim = SparkMaxSim(robot.launcher_motor, DCMotor.NEO(2))
        self._launcherRateLimiter = SlewRateLimiter(3.0)

    def update_sim(self, now: float, tm_diff: float) -> None:
        vbus = RobotController.getBatteryVoltage()

        # Launcher
        launcherVelocity = self._launcherRateLimiter.calculate(self._launcherSim.getSetpoint())
        self._launcherSim.iterate(launcherVelocity, vbus, tm_diff)

        # Drive
        if self._robot.isEnabled():
            states = tuple(module.update(vbus, tm_diff) for module in self._modules)
            speeds = self._kinematics.toChassisSpeeds(states)
            self._physics.drive(speeds, tm_diff)
            self._gyroSim.set_raw_yaw(self._physics.get_pose().rotation().degrees())
