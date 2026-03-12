import argparse
import csv
from collections import defaultdict

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def load_run(csv_path):
    rows = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            row["step"] = int(row["step"])
            row["time"] = float(row["time"])
            row["x"] = float(row["x"])
            row["y"] = float(row["y"])
            row["goal_x"] = float(row["goal_x"])
            row["goal_y"] = float(row["goal_y"])
            row["reached_goal"] = int(row["reached_goal"])
            row["collision_flag"] = int(row["collision_flag"])
            rows.append(row)

    if not rows:
        raise ValueError("CSV file is empty or invalid.")

    scenario = rows[0]["scenario"]
    planner = rows[0]["planner"]

    robot_data = defaultdict(list)
    goals = {}
    steps = set()

    for row in rows:
        name = row["robot_name"]
        robot_data[name].append(row)
        goals[name] = (row["goal_x"], row["goal_y"])
        steps.add(row["step"])

    for name in robot_data:
        robot_data[name].sort(key=lambda r: r["step"])

    sorted_steps = sorted(steps)
    return scenario, planner, robot_data, goals, sorted_steps


def compute_limits(robot_data, goals, margin=0.8):
    xs = []
    ys = []

    for rows in robot_data.values():
        for r in rows:
            xs.append(r["x"])
            ys.append(r["y"])

    for gx, gy in goals.values():
        xs.append(gx)
        ys.append(gy)

    return min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin


def main():
    parser = argparse.ArgumentParser(description="Animate a multi-robot run from a CSV log.")
    parser.add_argument("csv_path", help="Path to results CSV")
    parser.add_argument("--interval", type=int, default=80, help="Animation interval in ms")
    args = parser.parse_args()

    scenario, planner, robot_data, goals, sorted_steps = load_run(args.csv_path)
    min_x, max_x, min_y, max_y = compute_limits(robot_data, goals)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title(f"Scenario: {scenario} | Planner: {planner}")
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)

    colors = ["tab:blue", "tab:orange", "tab:green", "tab:red", "tab:purple", "tab:brown"]

    path_lines = {}
    robot_points = {}
    text_time = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")
    text_status = ax.text(0.02, 0.93, "", transform=ax.transAxes, va="top")

    robot_names = sorted(robot_data.keys())

    for i, name in enumerate(robot_names):
        color = colors[i % len(colors)]
        gx, gy = goals[name]

        ax.plot(gx, gy, marker="X", markersize=10, linestyle="None", label=f"{name} goal")
        line_plot, = ax.plot([], [], linewidth=2, color=color, label=f"{name} path")
        point_plot, = ax.plot([], [], marker="o", markersize=8, color=color, linestyle="None", label=name)

        path_lines[name] = line_plot
        robot_points[name] = point_plot

    ax.legend(loc="upper right")
    plt.tight_layout()

    def update(frame_idx):
        current_step = sorted_steps[frame_idx]
        current_time = 0.0
        collision_now = False
        reached_count = 0

        for name in robot_names:
            rows = robot_data[name]
            subrows = [r for r in rows if r["step"] <= current_step]

            xs = [r["x"] for r in subrows]
            ys = [r["y"] for r in subrows]

            if xs and ys:
                path_lines[name].set_data(xs, ys)
                robot_points[name].set_data([xs[-1]], [ys[-1]])
                current_time = subrows[-1]["time"]
                collision_now = collision_now or bool(subrows[-1]["collision_flag"])
                reached_count += int(subrows[-1]["reached_goal"])

        text_time.set_text(f"time: {current_time:.2f} s")
        text_status.set_text(
            f"step: {current_step} | reached: {reached_count}/{len(robot_names)} | collision: {collision_now}"
        )

        return list(path_lines.values()) + list(robot_points.values()) + [text_time, text_status]

    anim = FuncAnimation(
        fig,
        update,
        frames=len(sorted_steps),
        interval=args.interval,
        blit=False,
        repeat=False,
        cache_frame_data=False,
    )

    plt.show()

    return anim


if __name__ == "__main__":
    main()

