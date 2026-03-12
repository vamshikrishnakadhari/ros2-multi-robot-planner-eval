# ROS 2 Multi-Robot Trajectory Planner Evaluation in Conflict Scenarios

A lightweight ROS 2 Humble framework for evaluating decentralized trajectory planners for multiple mobile robots in conflict scenarios.

This project uses a custom 2D simulator instead of Gazebo to remain stable and efficient in a VirtualBox-based Ubuntu 22.04 environment. The focus is not high-fidelity physics, but systematic **planner evaluation** through repeatable scenarios, batch experiments, CSV-based metrics logging, and comparison plots.

## Project Goal

The project is designed around the idea of:

> evaluating trajectory planners in conflict scenarios of several mobile robots

The framework supports multiple robots, interchangeable planners, configurable scenarios, automatic run summaries, batch benchmark execution, and metric visualization.

## Current Features

- ROS 2 Humble package: `two_robot_sim`
- lightweight 2D multi-robot simulator
- configurable number of robots
- multiple local planner strategies
- conflict scenarios for planner evaluation
- per-run CSV logging
- summary CSV logging across runs
- batch experiment runner
- benchmark plotting
- launch files for cleaner ROS 2 execution
- live replay and run plotting utilities

## Repository Structure

```text
two_robot_sim/
├── launch/
│   ├── sim.launch.py
│   └── evaluation.launch.py
├── two_robot_sim/
│   ├── sim_node.py
│   ├── planners.py
│   ├── scenarios.py
│   ├── metrics.py
│   ├── run_experiments.py
│   ├── plot_metrics.py
│   ├── live_replay.py
│   └── plot_run.py
├── results/
│   ├── summary.csv
│   ├── planner_summary.csv
│   ├── scenario_planner_summary.csv
│   └── plots/
├── package.xml
└── setup.py

## License

This project is released under the MIT License.

## Demo Animation

Example 10-robot random conflict scenario using the reciprocal planner:

![10-robot random demo](docs/media/random_10_robot_demo.gif)
MD
