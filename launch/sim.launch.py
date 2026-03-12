from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scenario = LaunchConfiguration('scenario')
    planner = LaunchConfiguration('planner')
    num_robots = LaunchConfiguration('num_robots')
    random_seed = LaunchConfiguration('random_seed')
    max_steps = LaunchConfiguration('max_steps')
    avoid_distance = LaunchConfiguration('avoid_distance')
    avoid_turn_gain = LaunchConfiguration('avoid_turn_gain')
    collision_distance = LaunchConfiguration('collision_distance')

    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='head_on'),
        DeclareLaunchArgument('planner', default_value='reciprocal'),
        DeclareLaunchArgument('num_robots', default_value='2'),
        DeclareLaunchArgument('random_seed', default_value='0'),
        DeclareLaunchArgument('max_steps', default_value='1500'),
        DeclareLaunchArgument('avoid_distance', default_value='2.0'),
        DeclareLaunchArgument('avoid_turn_gain', default_value='1.2'),
        DeclareLaunchArgument('collision_distance', default_value='0.45'),

        Node(
            package='two_robot_sim',
            executable='sim_node',
            name='multi_robot_sim',
            output='screen',
            parameters=[{
                'scenario': scenario,
                'planner': planner,
                'num_robots': num_robots,
                'random_seed': random_seed,
                'max_steps': max_steps,
                'avoid_distance': avoid_distance,
                'avoid_turn_gain': avoid_turn_gain,
                'collision_distance': collision_distance,
            }]
        )
    ])

