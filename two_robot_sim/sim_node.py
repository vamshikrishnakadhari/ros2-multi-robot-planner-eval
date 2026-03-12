import csv
import math
import os
from copy import deepcopy
from datetime import datetime

import rclpy
from rclpy.node import Node

from two_robot_sim.planners import PLANNERS, distance_xy, goal_distance
from two_robot_sim.scenarios import SCENARIOS, generate_random_scenario
from two_robot_sim.metrics import compute_run_summary, append_summary_csv


class MultiRobotSimNode(Node):
    def __init__(self):
        super().__init__("multi_robot_sim")

        self.declare_parameter("scenario", "head_on")
        self.declare_parameter("planner", "baseline")
        self.declare_parameter("dt", 0.1)
        self.declare_parameter("max_steps", 500)
        self.declare_parameter("robot_radius", 0.25)
        self.declare_parameter("goal_tolerance", 0.2)
        self.declare_parameter("collision_distance", 0.5)
        self.declare_parameter("slowdown_distance", 1.0)
        self.declare_parameter("yield_distance", 1.0)
        self.declare_parameter("yield_speed_scale", 0.15)
        self.declare_parameter("avoid_distance", 1.2)
        self.declare_parameter("avoid_turn_gain", 0.3)
        self.declare_parameter("k_theta", 1.5)
        self.declare_parameter("max_omega", 1.2)
        self.declare_parameter("results_dir", "results")

        self.declare_parameter("num_robots", 6)
        self.declare_parameter("world_size", 4.0)
        self.declare_parameter("random_seed", 42)
        self.declare_parameter("min_start_separation", 1.2)
        self.declare_parameter("min_goal_separation", 1.2)
        self.declare_parameter("min_start_goal_distance", 2.0)
        self.declare_parameter("random_max_speed", 0.28)

        self.scenario_name = self.get_parameter("scenario").value
        self.planner_name = self.get_parameter("planner").value

        if self.scenario_name != "random" and self.scenario_name not in SCENARIOS:
            raise ValueError(f"Unknown scenario: {self.scenario_name}")

        if self.planner_name not in PLANNERS:
            raise ValueError(f"Unknown planner: {self.planner_name}")

        self.dt = float(self.get_parameter("dt").value)
        self.max_steps = int(self.get_parameter("max_steps").value)

        self.params = {
            "robot_radius": float(self.get_parameter("robot_radius").value),
            "goal_tolerance": float(self.get_parameter("goal_tolerance").value),
            "collision_distance": float(self.get_parameter("collision_distance").value),
            "slowdown_distance": float(self.get_parameter("slowdown_distance").value),
            "yield_distance": float(self.get_parameter("yield_distance").value),
            "yield_speed_scale": float(self.get_parameter("yield_speed_scale").value),
            "avoid_distance": float(self.get_parameter("avoid_distance").value),
            "avoid_turn_gain": float(self.get_parameter("avoid_turn_gain").value),
            "k_theta": float(self.get_parameter("k_theta").value),
            "max_omega": float(self.get_parameter("max_omega").value),
        }

        self.robots = self._init_robots()
        self.planner_fn = PLANNERS[self.planner_name]

        self.step_count = 0
        self.collision_happened = False
        self.min_inter_robot_distance = float("inf")
        self.sim_finished = False

        self.results_dir = str(self.get_parameter("results_dir").value)
        os.makedirs(self.results_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(
            self.results_dir,
            f"{self.scenario_name}_{self.planner_name}_{timestamp}.csv"
        )
        self.summary_csv_path = os.path.join(self.results_dir, "summary.csv")

        self._init_csv()

        self.get_logger().info(f"Scenario: {self.scenario_name}")
        self.get_logger().info(f"Planner: {self.planner_name}")
        self.get_logger().info(f"Robots: {len(self.robots)}")
        self.get_logger().info(f"Logging to: {self.csv_path}")

        self.timer = self.create_timer(self.dt, self.step_simulation)

    def _init_robots(self):
        if self.scenario_name == "random":
            scenario = generate_random_scenario(
                num_robots=int(self.get_parameter("num_robots").value),
                world_size=float(self.get_parameter("world_size").value),
                min_start_separation=float(self.get_parameter("min_start_separation").value),
                min_goal_separation=float(self.get_parameter("min_goal_separation").value),
                min_start_goal_distance=float(self.get_parameter("min_start_goal_distance").value),
                max_speed=float(self.get_parameter("random_max_speed").value),
                seed=int(self.get_parameter("random_seed").value),
            )
        else:
            scenario = deepcopy(SCENARIOS[self.scenario_name])

        robots = []

        for idx, robot_cfg in enumerate(scenario["robots"]):
            robots.append({
                "name": robot_cfg["name"],
                "x": float(robot_cfg["start"][0]),
                "y": float(robot_cfg["start"][1]),
                "theta": float(robot_cfg["start"][2]),
                "goal": (float(robot_cfg["goal"][0]), float(robot_cfg["goal"][1])),
                "max_speed": float(robot_cfg["max_speed"]),
                "priority": idx,
                "v": 0.0,
                "omega": 0.0,
                "reached_goal": False,
                "time_to_goal": None,
                "path_length": 0.0,
                "last_x": float(robot_cfg["start"][0]),
                "last_y": float(robot_cfg["start"][1]),
            })

        return robots

    def _init_csv(self):
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "step",
                "time",
                "scenario",
                "planner",
                "robot_name",
                "x",
                "y",
                "theta",
                "v",
                "omega",
                "goal_x",
                "goal_y",
                "goal_distance",
                "reached_goal",
                "nearest_robot_distance",
                "collision_flag",
                "path_length",
            ])

    def _nearest_distance_for_robot(self, robot):
        min_dist = float("inf")
        for other in self.robots:
            if other["name"] == robot["name"]:
                continue
            d = distance_xy((robot["x"], robot["y"]), (other["x"], other["y"]))
            min_dist = min(min_dist, d)
        return min_dist if min_dist != float("inf") else 999.0

    def _global_min_distance_and_collision(self):
        min_dist = float("inf")
        collision = False

        for i in range(len(self.robots)):
            for j in range(i + 1, len(self.robots)):
                d = distance_xy(
                    (self.robots[i]["x"], self.robots[i]["y"]),
                    (self.robots[j]["x"], self.robots[j]["y"]),
                )
                min_dist = min(min_dist, d)
                if d < self.params["collision_distance"]:
                    collision = True

        if min_dist == float("inf"):
            min_dist = 999.0

        return min_dist, collision

    def _integrate_robot(self, robot):
        dx = robot["v"] * math.cos(robot["theta"]) * self.dt
        dy = robot["v"] * math.sin(robot["theta"]) * self.dt
        dtheta = robot["omega"] * self.dt

        robot["x"] += dx
        robot["y"] += dy
        robot["theta"] += dtheta

        step_dist = math.hypot(robot["x"] - robot["last_x"], robot["y"] - robot["last_y"])
        robot["path_length"] += step_dist
        robot["last_x"] = robot["x"]
        robot["last_y"] = robot["y"]

    def _update_goal_status(self, robot):
        if robot["reached_goal"]:
            return

        dist = goal_distance(robot)
        if dist <= self.params["goal_tolerance"]:
            robot["reached_goal"] = True
            robot["v"] = 0.0
            robot["omega"] = 0.0
            robot["time_to_goal"] = self.step_count * self.dt
            self.get_logger().info(
                f"{robot['name']} reached goal at t={robot['time_to_goal']:.2f}s"
            )

    def _log_step(self, collision_flag):
        with open(self.csv_path, "a", newline="") as f:
            writer = csv.writer(f)
            sim_time = self.step_count * self.dt

            for robot in self.robots:
                writer.writerow([
                    self.step_count,
                    f"{sim_time:.3f}",
                    self.scenario_name,
                    self.planner_name,
                    robot["name"],
                    f"{robot['x']:.4f}",
                    f"{robot['y']:.4f}",
                    f"{robot['theta']:.4f}",
                    f"{robot['v']:.4f}",
                    f"{robot['omega']:.4f}",
                    f"{robot['goal'][0]:.4f}",
                    f"{robot['goal'][1]:.4f}",
                    f"{goal_distance(robot):.4f}",
                    int(robot["reached_goal"]),
                    f"{self._nearest_distance_for_robot(robot):.4f}",
                    int(collision_flag),
                    f"{robot['path_length']:.4f}",
                ])

    def _all_reached_goal(self):
        return all(robot["reached_goal"] for robot in self.robots)

    def _print_summary(self):
        self.get_logger().info("=== Simulation Summary ===")
        self.get_logger().info(f"Scenario: {self.scenario_name}")
        self.get_logger().info(f"Planner: {self.planner_name}")
        self.get_logger().info(f"Steps: {self.step_count}")
        self.get_logger().info(f"Sim time: {self.step_count * self.dt:.2f}s")
        self.get_logger().info(f"Collision happened: {self.collision_happened}")
        self.get_logger().info(
            f"Min inter-robot distance: {self.min_inter_robot_distance:.3f} m"
        )
        self.get_logger().info(f"Results CSV: {self.csv_path}")

        for robot in self.robots:
            self.get_logger().info(
                f"{robot['name']}: reached={robot['reached_goal']}, "
                f"time_to_goal={robot['time_to_goal']}, "
                f"path_length={robot['path_length']:.3f}"
            )

    def _finish_simulation(self, message=None, warn=False):
        if self.sim_finished:
            return

        self.sim_finished = True

        if message is not None:
            if warn:
                self.get_logger().warn(message)
            else:
                self.get_logger().info(message)

        self._print_summary()

        summary = compute_run_summary(self)
        append_summary_csv(self.summary_csv_path, summary)
        self.get_logger().info(f"Summary CSV updated: {self.summary_csv_path}")

        try:
            self.timer.cancel()
        except Exception:
            pass

    def step_simulation(self):
        if self.sim_finished:
            return

        if self.step_count >= self.max_steps:
            self._finish_simulation("Maximum steps reached.", warn=True)
            return

        commands = []
        for robot in self.robots:
            v, omega = self.planner_fn(robot, self.robots, self.params)
            commands.append((v, omega))

        for robot, (v, omega) in zip(self.robots, commands):
            robot["v"] = v
            robot["omega"] = omega
            self._integrate_robot(robot)
            self._update_goal_status(robot)

        min_dist, collision_flag = self._global_min_distance_and_collision()
        self.min_inter_robot_distance = min(self.min_inter_robot_distance, min_dist)

        self._log_step(collision_flag)

        if collision_flag:
            self.collision_happened = True
            self._finish_simulation("Collision detected. Stopping simulation.", warn=True)
            return

        if self._all_reached_goal():
            self._finish_simulation("All robots reached goals.")
            return

        self.step_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotSimNode()

    try:
        while rclpy.ok() and not node.sim_finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if hasattr(node, "timer"):
                node.timer.cancel()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

