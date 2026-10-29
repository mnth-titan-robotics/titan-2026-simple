#
# robot.py - the whole robot program lives in this file.
#
# Read it from top to bottom. It is split into four parts:
#
#   1. IMPORTS   - code written by other people that we want to use
#   2. SETTINGS  - numbers you can change to adjust how the robot behaves
#   3. HELPERS   - small functions that save us from repeating ourselves
#   4. ROBOT     - the robot itself: what happens at startup, and the main loop
#

# ==========================================================================
# 1. IMPORTS
# ==========================================================================
# "import" loads a module - a file full of code that someone else wrote.
# After importing, we use the module's name followed by a dot to reach the
# things inside it, like wpilib.XboxController or rev.SparkMax.

import wpilib
import wpimath
import rev

# swerve_drive.py is a file in THIS project. We import it the same way.
# Open it if you're curious, but you don't need to change it or understand
# it to use it.
from swerve_drive import SwerveDrive


# ==========================================================================
# 2. SETTINGS
# ==========================================================================
# Names in ALL_CAPS are "constants" - values that are set once and never
# change while the robot runs. Keeping them up here means you can tune the
# robot without digging through the rest of the code.

# Which USB slot each Xbox controller is plugged into on the Driver Station
DRIVER_CONTROLLER_PORT = 0
OPERATOR_CONTROLLER_PORT = 1

# Joystick values smaller than this are ignored, so a stick that doesn't
# rest perfectly at zero won't make the robot creep.
JOYSTICK_DEADBAND = 0.05

# How far a trigger must be pressed (0.0 to 1.0) before we count it as "pressed"
TRIGGER_THRESHOLD = 0.5

# CAN IDs - every motor controller on the robot has its own ID number
INTAKE_MOTOR_ID = 12
INTAKE_EXTENDER_LEFT_MOTOR_ID = 10
INTAKE_EXTENDER_RIGHT_MOTOR_ID = 11
INDEXER_MOTOR_ID = 16
LAUNCHER_LEFT_MOTOR_ID = 20
LAUNCHER_RIGHT_MOTOR_ID = 21

# Motor speeds go from -1.0 (full reverse) to 1.0 (full forward)
INTAKE_SPEED = 0.6
INTAKE_REVERSE_SPEED = -0.5
INTAKE_EXTENDER_SPEED = 0.3
INDEXER_SPEED = 0.9
INDEXER_REVERSE_SPEED = -0.9

# The launcher is different: instead of a power level, we ask for an exact
# flywheel speed (in RPM) and the motor controller holds it there.
LAUNCHER_SPEED = 3300

# The launcher counts as "ready" once it reaches this fraction of LAUNCHER_SPEED
LAUNCHER_READY_PERCENT = 0.9


# ==========================================================================
# 3. HELPERS
# ==========================================================================
# A function is a named, reusable block of code. "def" creates one.
# The names in parentheses are parameters - values handed to the function
# each time it is called.

def apply_settings(motor, settings):
    """Sends a group of settings to a motor controller."""
    # kResetSafeParameters clears out any old settings first, so the motor starts fresh.
    # kPersistParameters saves the settings so they survive a power cycle.
    motor.configure(
        settings,
        rev.ResetMode.kResetSafeParameters,
        rev.PersistMode.kPersistParameters,
    )


def read_joystick(value):
    """Ignores tiny joystick movements so the robot doesn't drift."""
    # "return" hands a value back to whoever called the function.
    return wpimath.applyDeadband(value, JOYSTICK_DEADBAND)


# ==========================================================================
# 4. ROBOT
# ==========================================================================
# A class bundles data (variables) and actions (methods) together.
# You've been using classes all along - every motor and controller is one.
#
# Our Robot class builds on wpilib.TimedRobot, which already knows how to talk
# to the Driver Station and run our code on a timer. We only fill in the parts
# that are specific to OUR robot.
#
# Inside a class, "self" means "this robot". Storing something as
# self.something lets every method in the class use it later.

class Robot(wpilib.TimedRobot):

    def robotInit(self):
        """
        ROBOT SETUP - runs ONE time, when the robot program first starts.
        This is where we create every controller, motor and sensor.
        """

        # ----- Controllers -----
        self.driver = wpilib.XboxController(DRIVER_CONTROLLER_PORT)
        self.operator = wpilib.XboxController(OPERATOR_CONTROLLER_PORT)

        # ----- Drive -----
        # Swerve drive is complicated, so all of its details live in swerve_drive.py
        self.drive = SwerveDrive()

        # ----- Intake: the spinning roller that grabs fuel off the floor -----
        self.intake_motor = rev.SparkMax(INTAKE_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
        intake_settings = rev.SparkMaxConfig()
        intake_settings.inverted(False)
        apply_settings(self.intake_motor, intake_settings)

        # ----- Intake extender: swings the intake out of and back into the robot -----
        # Two motors work together here. The right motor is set to "follow"
        # the left one, so we only ever have to control the left motor.
        self.intake_extender_motor = rev.SparkMax(INTAKE_EXTENDER_LEFT_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
        intake_extender_follower = rev.SparkMax(INTAKE_EXTENDER_RIGHT_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
        extender_settings = rev.SparkMaxConfig()
        extender_settings.inverted(False)
        extender_settings.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)  # Hold position when stopped
        extender_settings.smartCurrentLimit(25)
        extender_settings.voltageCompensation(12)
        apply_settings(self.intake_extender_motor, extender_settings)
        extender_settings.follow(INTAKE_EXTENDER_LEFT_MOTOR_ID, True)  # True = spin the opposite way
        apply_settings(intake_extender_follower, extender_settings)

        # ----- Indexer: carries fuel from the intake up to the launcher -----
        self.indexer_motor = rev.SparkMax(INDEXER_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
        indexer_settings = rev.SparkMaxConfig()
        indexer_settings.inverted(True)
        indexer_settings.smartCurrentLimit(25)
        indexer_settings.voltageCompensation(12)
        apply_settings(self.indexer_motor, indexer_settings)

        # ----- Launcher: two flywheels that shoot fuel -----
        self.launcher_motor = rev.SparkMax(LAUNCHER_LEFT_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
        launcher_follower = rev.SparkMax(LAUNCHER_RIGHT_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
        launcher_settings = rev.SparkMaxConfig()
        launcher_settings.inverted(True)
        launcher_settings.smartCurrentLimit(60)
        launcher_settings.voltageCompensation(12)
        # The motor controller holds a steady speed on its own:
        # - feedforward (kV) supplies the volts needed for each unit of speed
        # - PID adds extra power when a ball slows the flywheel down
        launcher_settings.closedLoop.pid(0.00033, 0.0, 0.0)
        launcher_settings.closedLoop.feedForward.kV(0.0036)  # volts per RPM
        # Convert motor RPM into flywheel RPM (1.5:1 gearing)
        launcher_settings.encoder.velocityConversionFactor(1.0 / 1.5)
        apply_settings(self.launcher_motor, launcher_settings)
        launcher_settings.follow(LAUNCHER_LEFT_MOTOR_ID, True)
        apply_settings(launcher_follower, launcher_settings)

        # The encoder measures how fast the launcher is spinning, and the
        # controller is what we use to ask for an exact speed.
        self.launcher_encoder = self.launcher_motor.getEncoder()
        self.launcher_controller = self.launcher_motor.getClosedLoopController()

    def robotPeriodic(self):
        """
        Runs about 50 times per second, no matter what mode the robot is in.
        We use it to show information on the dashboard.
        """
        wpilib.SmartDashboard.putNumber("Launcher Speed", self.launcher_encoder.getVelocity())
        wpilib.SmartDashboard.putBoolean("Launcher Ready", self.is_launcher_ready())
        wpilib.SmartDashboard.putNumber("Robot Heading", self.drive.get_heading())

    def teleopPeriodic(self):
        """
        THE MAIN LOOP - runs about 50 times per second while the drivers are
        in control (TeleOp mode).

        Each time through, we read the controllers and decide what every
        motor should be doing RIGHT NOW. Because this runs so often, letting
        go of a button stops its motor almost instantly.
        """

        # ==================================================================
        # DRIVER CONTROLS
        # ==================================================================

        # Pushing a stick forward gives a NEGATIVE number, so we flip the
        # signs with a minus to make "forward" positive.
        forward = -read_joystick(self.driver.getLeftY())
        left = -read_joystick(self.driver.getLeftX())
        turn = -read_joystick(self.driver.getRightX())
        self.drive.drive(forward, left, turn)

        # Back + Start together: tell the robot that the way it is facing
        # right now is "forward". Requiring both buttons prevents accidents.
        if self.driver.getBackButton() and self.driver.getStartButton():
            self.drive.reset_heading()

        # ==================================================================
        # OPERATOR CONTROLS
        # ==================================================================

        # ----- Intake -----
        # Python checks each condition in order and runs only the FIRST one
        # that is true. If none are true, the "else" part runs.
        if self.operator.getLeftTriggerAxis() > TRIGGER_THRESHOLD:
            self.intake_motor.set(INTAKE_SPEED)
        elif self.operator.getLeftBumperButton():
            self.intake_motor.set(INTAKE_REVERSE_SPEED)
        else:
            self.intake_motor.set(0)

        # ----- Intake extender -----
        # getPOV() reads the D-pad as an angle: 0 is up, 180 is down,
        # and -1 means nothing is pressed.
        dpad = self.operator.getPOV()
        if dpad == 180:
            self.intake_extender_motor.set(INTAKE_EXTENDER_SPEED)
        elif dpad == 0:
            self.intake_extender_motor.set(-INTAKE_EXTENDER_SPEED)
        else:
            self.intake_extender_motor.set(0)

        # ----- Launcher -----
        # Right trigger spins the launcher up so it's ready ahead of time.
        # Right bumper shoots, which also needs the launcher spinning.
        shoot_pressed = self.operator.getRightBumperButton()
        spin_up_pressed = self.operator.getRightTriggerAxis() > TRIGGER_THRESHOLD

        if spin_up_pressed or shoot_pressed:
            self.launcher_controller.setSetpoint(LAUNCHER_SPEED, rev.SparkMax.ControlType.kVelocity)
        else:
            self.launcher_motor.set(0)

        # ----- Indexer -----
        # Only feed fuel into the launcher once it is up to speed,
        # otherwise the shot falls short.
        if shoot_pressed and self.is_launcher_ready():
            self.indexer_motor.set(INDEXER_SPEED)
        elif self.operator.getBButton():
            self.indexer_motor.set(INDEXER_REVERSE_SPEED)
        else:
            self.indexer_motor.set(0)

    def is_launcher_ready(self):
        """Returns True when the launcher is spinning fast enough to shoot."""
        current_speed = abs(self.launcher_encoder.getVelocity())
        return current_speed >= LAUNCHER_SPEED * LAUNCHER_READY_PERCENT
