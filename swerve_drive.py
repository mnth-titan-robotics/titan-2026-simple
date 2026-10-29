#
# swerve_drive.py - everything needed to drive a REV MAXSwerve drivetrain.
#
# You don't need to change anything in here to drive the robot. robot.py only
# uses three things from this file:
#
#     drive = SwerveDrive()               create the drivetrain
#     drive.drive(forward, left, turn)    move, using joystick values (-1 to 1)
#     drive.reset_heading()               make the current direction "forward"
#
# A swerve drive has four "modules" (one in each corner). Every module has
# two motors: one spins the wheel, and one steers it to point in any direction.
# The SwerveModule class controls a single corner, and the SwerveDrive class
# coordinates all four.
#

import math

import rev
from phoenix6.hardware import Pigeon2
from wpimath import units
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModuleState


# ==========================================================================
# SETTINGS
# ==========================================================================

# CAN IDs for each module's motors
FRONT_LEFT_DRIVE_ID = 1
FRONT_LEFT_TURN_ID = 2
FRONT_RIGHT_DRIVE_ID = 3
FRONT_RIGHT_TURN_ID = 4
REAR_RIGHT_DRIVE_ID = 5
REAR_RIGHT_TURN_ID = 6
REAR_LEFT_DRIVE_ID = 7
REAR_LEFT_TURN_ID = 8

# CAN ID of the Pigeon 2 gyro, which tells us which way the robot is facing
GYRO_ID = 1

# How each module's steering encoder is rotated compared to the robot frame, in radians
FRONT_LEFT_ANGLE_OFFSET = math.pi / 2
FRONT_RIGHT_ANGLE_OFFSET = math.pi
REAR_LEFT_ANGLE_OFFSET = 0.0
REAR_RIGHT_ANGLE_OFFSET = -math.pi / 2

# Distance between the left and right wheels, and between the front and back wheels
TRACK_WIDTH = units.inchesToMeters(26.5)
WHEEL_BASE = units.inchesToMeters(26.5)

# MAXSwerve gearing: 14 tooth pinion, 22 tooth spur gear, 45:15 bevel gears
WHEEL_DIAMETER = 0.0762  # meters
DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (14 * 15)
DRIVE_MOTOR_FREE_SPEED_RPS = 5676 / 60  # drive motor free speed, rotations per second
WHEEL_FREE_SPEED = DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_DIAMETER * math.pi / DRIVE_MOTOR_REDUCTION

# Top speeds when a joystick is pushed all the way
MAX_SPEED = WHEEL_FREE_SPEED     # meters per second (about 4.8)
MAX_TURN_SPEED = 2.0 * math.pi   # radians per second (one full spin each second)


# ==========================================================================
# ONE SWERVE MODULE
# ==========================================================================

class SwerveModule:
    """
    Controls one MAXSwerve module: a SPARK Flex driving the wheel and a
    SPARK MAX steering it, with a through-bore encoder measuring wheel direction.
    """

    def __init__(self, drive_motor_id, turn_motor_id, angle_offset):
        self.angle_offset = angle_offset

        self.drive_motor = rev.SparkFlex(drive_motor_id, rev.SparkFlex.MotorType.kBrushless)
        self.turn_motor = rev.SparkMax(turn_motor_id, rev.SparkMax.MotorType.kBrushless)

        # ----- Drive motor settings -----
        # Convert motor rotations into meters traveled by the wheel
        meters_per_rotation = WHEEL_DIAMETER * math.pi / DRIVE_MOTOR_REDUCTION
        drive_settings = rev.SparkFlexConfig()
        drive_settings.inverted(False)
        drive_settings.setIdleMode(rev.SparkFlexConfig.IdleMode.kBrake)
        drive_settings.smartCurrentLimit(50)
        drive_settings.encoder.positionConversionFactor(meters_per_rotation)
        drive_settings.encoder.velocityConversionFactor(meters_per_rotation / 60.0)
        drive_settings.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder)
        drive_settings.closedLoop.pid(0.04, 0, 0)
        # Feedforward: volts needed per meter/second of wheel speed (12 volts = free speed)
        drive_settings.closedLoop.feedForward.kV(12.0 / WHEEL_FREE_SPEED)
        drive_settings.closedLoop.outputRange(-1, 1)

        # ----- Turn motor settings -----
        # The steering encoder reports the wheel direction in radians (0 to 2*pi).
        # It's inverted because the module's output shaft spins opposite the motor.
        radians_per_rotation = 2 * math.pi
        turn_settings = rev.SparkMaxConfig()
        turn_settings.inverted(False)
        turn_settings.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        turn_settings.smartCurrentLimit(20)
        turn_settings.absoluteEncoder.inverted(True)
        turn_settings.absoluteEncoder.positionConversionFactor(radians_per_rotation)
        turn_settings.absoluteEncoder.velocityConversionFactor(radians_per_rotation / 60.0)
        turn_settings.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kAbsoluteEncoder)
        turn_settings.closedLoop.pid(1, 0, 0)
        turn_settings.closedLoop.outputRange(-1, 1)
        # Let the steering go "through zero", so turning from 350 degrees to
        # 10 degrees moves 20 degrees instead of 340.
        turn_settings.closedLoop.positionWrappingEnabled(True)
        turn_settings.closedLoop.positionWrappingInputRange(0, radians_per_rotation)

        # Reset old settings first, then save these so they survive a power cycle
        self.drive_motor.configure(
            drive_settings, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.turn_motor.configure(
            turn_settings, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        self.turn_encoder = self.turn_motor.getAbsoluteEncoder()
        self.drive_controller = self.drive_motor.getClosedLoopController()
        self.turn_controller = self.turn_motor.getClosedLoopController()

    def set_desired_state(self, desired_state):
        """Points the wheel in a direction and spins it at a speed (meters per second)."""
        # Shift the requested angle by this module's offset
        state = SwerveModuleState(
            desired_state.speed,
            desired_state.angle + Rotation2d(self.angle_offset))

        # If the wheel would need to turn more than 90 degrees, it's faster to
        # turn the other way and spin the wheel backwards instead.
        state.optimize(Rotation2d(self.turn_encoder.getPosition()))

        self.drive_controller.setSetpoint(state.speed, rev.SparkFlex.ControlType.kVelocity)
        self.turn_controller.setSetpoint(state.angle.radians(), rev.SparkMax.ControlType.kPosition)


# ==========================================================================
# THE WHOLE DRIVETRAIN
# ==========================================================================

class SwerveDrive:
    """Coordinates four swerve modules and a gyro to drive the robot."""

    def __init__(self):
        self.front_left = SwerveModule(FRONT_LEFT_DRIVE_ID, FRONT_LEFT_TURN_ID, FRONT_LEFT_ANGLE_OFFSET)
        self.front_right = SwerveModule(FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_TURN_ID, FRONT_RIGHT_ANGLE_OFFSET)
        self.rear_left = SwerveModule(REAR_LEFT_DRIVE_ID, REAR_LEFT_TURN_ID, REAR_LEFT_ANGLE_OFFSET)
        self.rear_right = SwerveModule(REAR_RIGHT_DRIVE_ID, REAR_RIGHT_TURN_ID, REAR_RIGHT_ANGLE_OFFSET)

        self.gyro = Pigeon2(GYRO_ID)

        # Kinematics does the math that turns "drive this way" into a speed and
        # angle for each wheel. It needs to know where each module sits
        # relative to the center of the robot (front-left, front-right,
        # rear-left, rear-right).
        self.kinematics = SwerveDrive4Kinematics(
            Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        )

    def drive(self, forward, left, turn):
        """
        Drives the robot using joystick-style values from -1.0 to 1.0.

        Driving is "field relative": pushing forward always moves the robot
        away from the driver, no matter which way the robot is facing.
        """
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            forward * MAX_SPEED,
            left * MAX_SPEED,
            turn * MAX_TURN_SPEED,
            self.gyro.getRotation2d(),
        )
        states = self.kinematics.toSwerveModuleStates(speeds)

        # If any wheel was asked to go faster than it can, slow all of them
        # down by the same amount so the robot still moves in the right direction.
        states = SwerveDrive4Kinematics.desaturateWheelSpeeds(states, MAX_SPEED)

        self.front_left.set_desired_state(states[0])
        self.front_right.set_desired_state(states[1])
        self.rear_left.set_desired_state(states[2])
        self.rear_right.set_desired_state(states[3])

    def reset_heading(self):
        """Makes the direction the robot is facing right now count as "forward"."""
        self.gyro.set_yaw(0)

    def get_heading(self):
        """Returns which way the robot is facing, in degrees."""
        return self.gyro.getRotation2d().degrees()
