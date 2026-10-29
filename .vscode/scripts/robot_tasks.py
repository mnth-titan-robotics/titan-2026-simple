"""
One entry point for every VS Code task: setup, sim, test and deploy.

Every command first makes sure the project's virtual environment (.venv)
exists and has the libraries listed in pyproject.toml installed, so the
tasks can be run in any order. Setup is skipped when pyproject.toml hasn't
changed since the last successful install.

Run it with any Python 3 on the computer:

    py -3 .vscode/scripts/robot_tasks.py sim        (Windows)
    python3 .vscode/scripts/robot_tasks.py sim      (macOS / Linux)
"""

import sys

if sys.version_info < (3, 11):
    sys.exit("Python 3.11 or newer is required. Install it from https://www.python.org/downloads/")

import hashlib
import os
import subprocess
import tomllib
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[2]
VENV_DIR = PROJECT_DIR / ".venv"
PYPROJECT = PROJECT_DIR / "pyproject.toml"
# Records which pyproject.toml the environment was last set up from
SETUP_STAMP = VENV_DIR / ".robot-setup-stamp"

if os.name == "nt":
    VENV_PYTHON = VENV_DIR / "Scripts" / "python.exe"
else:
    VENV_PYTHON = VENV_DIR / "bin" / "python"

# The robotpy command (and arguments) run for each task
COMMANDS = {
    "setup": None,
    "sim": ["sim"],
    "test": ["test", "--isolated", "--", "-v", "-s", "--exitfirst"],
    "deploy": ["deploy", "--skip-tests", "--ignore-image-version"],
}


def run(*args):
    """Runs a command from the project folder, stopping everything if it fails."""
    print(">", " ".join(str(arg) for arg in args), flush=True)
    result = subprocess.run(args, cwd=PROJECT_DIR)
    if result.returncode != 0:
        sys.exit(result.returncode)


def read_requirements(pyproject_text):
    """Turns the [tool.robotpy] section of pyproject.toml into a list of pip packages."""
    settings = tomllib.loads(pyproject_text)["tool"]["robotpy"]
    robotpy = f"robotpy=={settings['robotpy_version']}"
    components = settings.get("components", [])
    if components:
        robotpy = f"robotpy[{','.join(components)}]=={settings['robotpy_version']}"
    return [robotpy, *settings.get("requires", [])]


def ensure_environment():
    """Creates .venv and installs the robot libraries, if that hasn't been done yet."""
    pyproject_text = PYPROJECT.read_text(encoding="utf-8")
    wanted_stamp = hashlib.sha256(pyproject_text.encode("utf-8")).hexdigest()

    if VENV_PYTHON.exists() and SETUP_STAMP.exists():
        if SETUP_STAMP.read_text(encoding="utf-8").strip() == wanted_stamp:
            return

    if not VENV_PYTHON.exists():
        print(f"Creating virtual environment in {VENV_DIR} ...", flush=True)
        run(sys.executable, "-m", "venv", "--clear", VENV_DIR)

    print("Installing the robot libraries. The first time can take several minutes ...", flush=True)
    # "robotpy sync" can install these too, but on Windows it finishes the install in a
    # separate window without waiting, so the next command would run too early.
    run(VENV_PYTHON, "-m", "pip", "install", "--upgrade", "certifi", *read_requirements(pyproject_text))
    # Download the copies of everything that the roboRIO needs for deploy
    run(VENV_PYTHON, "-m", "robotpy", "sync", "--use-certifi", "--no-install", "--no-upgrade-project")

    SETUP_STAMP.write_text(wanted_stamp, encoding="utf-8")
    print("Robot environment is ready.", flush=True)


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in COMMANDS:
        sys.exit(f"Usage: robot_tasks.py [{' | '.join(COMMANDS)}]")

    ensure_environment()

    robotpy_args = COMMANDS[sys.argv[1]]
    if robotpy_args is not None:
        run(VENV_PYTHON, "-m", "robotpy", *robotpy_args)


if __name__ == "__main__":
    main()
