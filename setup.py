from setuptools import setup
from glob import glob
import os

package_name = 'two_robot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Lightweight ROS 2 multi-robot simulator and planner evaluation framework',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_node = two_robot_sim.sim_node:main',
            'live_replay = two_robot_sim.live_replay:main',
            'plot_run = two_robot_sim.plot_run:main',
            'run_experiments = two_robot_sim.run_experiments:main',
            'plot_metrics = two_robot_sim.plot_metrics:main',
        ],
    },
)

