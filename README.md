# Simple Robot

A first step into robot programming. The whole robot (controllers, motors and the main loop) lives in one file: **`robot.py`**. Start there and read it from top to bottom.

It runs the same hardware and library versions as titan-2026, with everything except the basics taken out: no autonomous, auto-aim, automatic launcher speed, lights or vision.

## The files

| File | What it is | Do I need to read it? |
|---|---|---|
| `robot.py` | Settings, robot setup and the main loop | **Yes, start here** |
| `swerve_drive.py` | Controls the four swerve wheels and the gyro | Only if you're curious |
| `physics.py` | Fakes the real world in the simulator | No |
| `tests/pyfrc_test.py` | Checks that the code starts without crashing | No |
| `pyproject.toml` | Which library versions to install | No |

## Setup

1. Install [Python](https://www.python.org/downloads/) and [VS Code](https://code.visualstudio.com/).
2. Open this folder in VS Code. When asked, trust the folder and install the recommended Python extensions.
3. VS Code sets up the project on its own the first time: it creates a `.venv` folder and downloads the robot libraries. This takes a few minutes and needs internet. Watch the **Terminal** panel for "Robot environment is ready."

## Running

Open the **Run and Debug** panel (the play button with a bug, on the left side of VS Code), pick one of these from the dropdown, and press the green arrow (or **F5**):

| Choice | What it does |
|---|---|
| **Simulate** | Runs the robot on your computer |
| **Test** | Checks the code for crashes |
| **Deploy** | Sends the code to the real robot (connect to the robot's network first) |

The same actions are also in **Terminal → Run Task...**, and **Ctrl+Shift+B** starts the simulator.

Without VS Code, run `py -3 .vscode/scripts/robot_tasks.py sim` on Windows, or `python3 .vscode/scripts/robot_tasks.py sim` on macOS/Linux. Swap `sim` for `test`, `deploy` or `setup`.

**In the simulator:** drag a controller (or a keyboard) from *System Joysticks* into slot 0 (driver) or slot 1 (operator), click **Teleoperated** under *Robot State*, then open *NetworkTables → SmartDashboard → Field* to watch the robot move.

## Controls

**Driver (port 0)**

| Input | Action |
|---|---|
| Left stick | Drive (field relative: forward on the stick always moves away from you) |
| Right stick left/right | Turn |
| Back + Start | Reset heading (point the robot away from you first) |

**Operator (port 1)**

| Input | Action |
|---|---|
| Left trigger | Intake |
| Left bumper | Reverse intake |
| D-pad down / up | Extend / retract intake |
| Right trigger | Spin up launcher |
| Right bumper | Shoot (feeds fuel once the launcher is up to speed) |
| B | Reverse indexer |

## Things to try

1. Change `INTAKE_SPEED` and watch the dashboard value change in the simulator.
2. Make the X button on the operator controller reverse the intake too. (Hint: `or`)
3. Add a "slow mode": while the driver holds the left bumper, multiply `forward`, `left` and `turn` by `0.5`.
4. Show `self.intake_motor.get()` on the dashboard in `robotPeriodic`.
5. Move the launcher code out of `teleopPeriodic` into its own method, `def run_launcher(self):`, and call it from the main loop.
