"""Check imports before starting the robot: python3 check_setup.py."""

import importlib
from pathlib import Path


def main():
    dependencies = (
        'numpy', 'matplotlib', 'rclpy', 'message_filters',
        'geometry_msgs.msg', 'nav_msgs.msg', 'sensor_msgs.msg',
    )
    failed = False
    for name in dependencies:
        try:
            importlib.import_module(name)
        except (ImportError, OSError) as error:
            print(f'FAIL: {name}: {error}')
            failed = True
        else:
            print(f'OK: {name}')

    local_modules = (
        'utilities', 'planner', 'pid', 'controller',
        'kalman_filter', 'localization', 'decisions', 'plot_ekf', 'plot_errors',
    )
    root = Path(__file__).resolve().parent
    for name in local_modules:
        if not (root / f'{name}.py').is_file():
            print(f'FAIL: missing starter/application module: {name}.py')
            failed = True

    if failed:
        print('\nUse the ROS 2 Python environment and install the dependencies in README.md.')
        return 1

    # Importing these modules creates no ROS nodes or publishers.
    for name in local_modules:
        try:
            imported = importlib.import_module(name)
            if Path(imported.__file__).resolve().parent != root:
                raise ImportError(f'{name} resolved outside this project')
        except (ImportError, OSError) as error:
            print(f'FAIL: {name}: {error}')
            failed = True

    if failed:
        return 1
    print('\nPASS: dependencies and bundled modules import successfully.')
    print('Next, start the simulator/robot and verify /odom, /imu, and /cmd_vel.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
