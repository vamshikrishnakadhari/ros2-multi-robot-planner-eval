import csv
import os


def compute_run_summary(node):
    num_robots = len(node.robots)
    reached_flags = [robot["reached_goal"] for robot in node.robots]
    success_all = all(reached_flags)

    time_to_goal_values = [
        robot["time_to_goal"]
        for robot in node.robots
        if robot["time_to_goal"] is not None
    ]

    path_lengths = [robot["path_length"] for robot in node.robots]

    avg_time_to_goal = (
        sum(time_to_goal_values) / len(time_to_goal_values)
        if time_to_goal_values else None
    )
    max_time_to_goal = max(time_to_goal_values) if time_to_goal_values else None

    avg_path_length = (
        sum(path_lengths) / len(path_lengths)
        if path_lengths else None
    )
    max_path_length = max(path_lengths) if path_lengths else None

    return {
        "scenario": node.scenario_name,
        "planner": node.planner_name,
        "num_robots": num_robots,
        "steps": node.step_count,
        "sim_time": node.step_count * node.dt,
        "success_all": int(success_all),
        "robots_reached_goal": sum(1 for flag in reached_flags if flag),
        "collision_happened": int(node.collision_happened),
        "min_inter_robot_distance": node.min_inter_robot_distance,
        "avg_time_to_goal": avg_time_to_goal,
        "max_time_to_goal": max_time_to_goal,
        "avg_path_length": avg_path_length,
        "max_path_length": max_path_length,
        "run_csv": node.csv_path,
    }


def append_summary_csv(summary_csv_path, summary):
    os.makedirs(os.path.dirname(summary_csv_path), exist_ok=True)

    file_exists = os.path.exists(summary_csv_path)

    fieldnames = [
        "scenario",
        "planner",
        "num_robots",
        "steps",
        "sim_time",
        "success_all",
        "robots_reached_goal",
        "collision_happened",
        "min_inter_robot_distance",
        "avg_time_to_goal",
        "max_time_to_goal",
        "avg_path_length",
        "max_path_length",
        "run_csv",
    ]

    with open(summary_csv_path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)

        if not file_exists:
            writer.writeheader()

        writer.writerow({
            "scenario": summary["scenario"],
            "planner": summary["planner"],
            "num_robots": summary["num_robots"],
            "steps": summary["steps"],
            "sim_time": f"{summary['sim_time']:.3f}",
            "success_all": summary["success_all"],
            "robots_reached_goal": summary["robots_reached_goal"],
            "collision_happened": summary["collision_happened"],
            "min_inter_robot_distance": f"{summary['min_inter_robot_distance']:.4f}",
            "avg_time_to_goal": (
                f"{summary['avg_time_to_goal']:.4f}"
                if summary["avg_time_to_goal"] is not None else ""
            ),
            "max_time_to_goal": (
                f"{summary['max_time_to_goal']:.4f}"
                if summary["max_time_to_goal"] is not None else ""
            ),
            "avg_path_length": (
                f"{summary['avg_path_length']:.4f}"
                if summary["avg_path_length"] is not None else ""
            ),
            "max_path_length": (
                f"{summary['max_path_length']:.4f}"
                if summary["max_path_length"] is not None else ""
            ),
            "run_csv": summary["run_csv"],
        })

