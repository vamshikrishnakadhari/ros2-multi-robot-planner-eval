import csv
import math
import os
from collections import defaultdict

import matplotlib.pyplot as plt


RESULTS_DIR = os.path.expanduser("~/multi_robot_ws/results")
SUMMARY_CSV = os.path.join(RESULTS_DIR, "summary.csv")
PLOTS_DIR = os.path.join(RESULTS_DIR, "plots")


def to_float(value):
    if value is None:
        return None
    value = str(value).strip()
    if value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def to_int(value):
    if value is None:
        return None
    value = str(value).strip()
    if value == "":
        return None
    try:
        return int(float(value))
    except ValueError:
        return None


def safe_mean(values):
    vals = [v for v in values if v is not None and not math.isnan(v)]
    if not vals:
        return None
    return sum(vals) / len(vals)


def load_rows():
    if not os.path.exists(SUMMARY_CSV):
        raise FileNotFoundError(f"summary.csv not found: {SUMMARY_CSV}")

    with open(SUMMARY_CSV, newline="") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        raise RuntimeError("summary.csv exists but has no data rows")

    parsed = []
    for r in rows:
        parsed.append({
            "scenario": r["scenario"],
            "planner": r["planner"],
            "num_robots": to_int(r["num_robots"]),
            "steps": to_int(r["steps"]),
            "sim_time": to_float(r["sim_time"]),
            "success_all": to_int(r["success_all"]),
            "robots_reached_goal": to_int(r["robots_reached_goal"]),
            "collision_happened": to_int(r["collision_happened"]),
            "min_inter_robot_distance": to_float(r["min_inter_robot_distance"]),
            "avg_time_to_goal": to_float(r["avg_time_to_goal"]),
            "max_time_to_goal": to_float(r["max_time_to_goal"]),
            "avg_path_length": to_float(r["avg_path_length"]),
            "max_path_length": to_float(r["max_path_length"]),
            "run_csv": r["run_csv"],
        })
    return parsed


def aggregate_by_planner(rows):
    groups = defaultdict(list)
    for r in rows:
        groups[r["planner"]].append(r)

    out = []
    for planner, items in sorted(groups.items()):
        out.append({
            "planner": planner,
            "runs": len(items),
            "success_rate": safe_mean([r["success_all"] for r in items]),
            "collision_rate": safe_mean([r["collision_happened"] for r in items]),
            "avg_sim_time": safe_mean([r["sim_time"] for r in items]),
            "avg_min_inter_robot_distance": safe_mean([r["min_inter_robot_distance"] for r in items]),
            "avg_time_to_goal": safe_mean([r["avg_time_to_goal"] for r in items]),
            "avg_path_length": safe_mean([r["avg_path_length"] for r in items]),
        })
    return out


def aggregate_by_scenario_and_planner(rows):
    groups = defaultdict(list)
    for r in rows:
        key = (r["scenario"], r["planner"])
        groups[key].append(r)

    out = []
    for (scenario, planner), items in sorted(groups.items()):
        out.append({
            "scenario": scenario,
            "planner": planner,
            "runs": len(items),
            "success_rate": safe_mean([r["success_all"] for r in items]),
            "collision_rate": safe_mean([r["collision_happened"] for r in items]),
            "avg_sim_time": safe_mean([r["sim_time"] for r in items]),
            "avg_min_inter_robot_distance": safe_mean([r["min_inter_robot_distance"] for r in items]),
            "avg_time_to_goal": safe_mean([r["avg_time_to_goal"] for r in items]),
            "avg_path_length": safe_mean([r["avg_path_length"] for r in items]),
        })
    return out


def write_csv(path, rows, fieldnames):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def print_table(title, rows, columns):
    print()
    print("=" * 100)
    print(title)
    print("=" * 100)

    widths = {}
    for col in columns:
        widths[col] = max(
            len(col),
            max((len(format_cell(r.get(col))) for r in rows), default=0)
        )

    header = "  ".join(col.ljust(widths[col]) for col in columns)
    print(header)
    print("-" * len(header))

    for row in rows:
        line = "  ".join(format_cell(row.get(col)).ljust(widths[col]) for col in columns)
        print(line)


def format_cell(value):
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def make_bar_plot(labels, values, title, ylabel, output_path):
    plt.figure(figsize=(8, 5))
    plt.bar(labels, values)
    plt.title(title)
    plt.ylabel(ylabel)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()


def main():
    os.makedirs(PLOTS_DIR, exist_ok=True)

    rows = load_rows()
    by_planner = aggregate_by_planner(rows)
    by_scenario_planner = aggregate_by_scenario_and_planner(rows)

    planner_csv = os.path.join(RESULTS_DIR, "planner_summary.csv")
    scenario_planner_csv = os.path.join(RESULTS_DIR, "scenario_planner_summary.csv")

    write_csv(
        planner_csv,
        by_planner,
        [
            "planner",
            "runs",
            "success_rate",
            "collision_rate",
            "avg_sim_time",
            "avg_min_inter_robot_distance",
            "avg_time_to_goal",
            "avg_path_length",
        ],
    )

    write_csv(
        scenario_planner_csv,
        by_scenario_planner,
        [
            "scenario",
            "planner",
            "runs",
            "success_rate",
            "collision_rate",
            "avg_sim_time",
            "avg_min_inter_robot_distance",
            "avg_time_to_goal",
            "avg_path_length",
        ],
    )

    print_table(
        "Planner Summary",
        by_planner,
        [
            "planner",
            "runs",
            "success_rate",
            "collision_rate",
            "avg_sim_time",
            "avg_min_inter_robot_distance",
            "avg_time_to_goal",
            "avg_path_length",
        ],
    )

    print_table(
        "Scenario + Planner Summary",
        by_scenario_planner,
        [
            "scenario",
            "planner",
            "runs",
            "success_rate",
            "collision_rate",
            "avg_sim_time",
            "avg_min_inter_robot_distance",
            "avg_time_to_goal",
            "avg_path_length",
        ],
    )

    planner_labels = [r["planner"] for r in by_planner]

    make_bar_plot(
        planner_labels,
        [0.0 if r["success_rate"] is None else r["success_rate"] for r in by_planner],
        "Success Rate by Planner",
        "success_rate",
        os.path.join(PLOTS_DIR, "success_rate_by_planner.png"),
    )

    make_bar_plot(
        planner_labels,
        [0.0 if r["collision_rate"] is None else r["collision_rate"] for r in by_planner],
        "Collision Rate by Planner",
        "collision_rate",
        os.path.join(PLOTS_DIR, "collision_rate_by_planner.png"),
    )

    make_bar_plot(
        planner_labels,
        [0.0 if r["avg_sim_time"] is None else r["avg_sim_time"] for r in by_planner],
        "Average Simulation Time by Planner",
        "seconds",
        os.path.join(PLOTS_DIR, "avg_sim_time_by_planner.png"),
    )

    make_bar_plot(
        planner_labels,
        [0.0 if r["avg_min_inter_robot_distance"] is None else r["avg_min_inter_robot_distance"] for r in by_planner],
        "Average Min Inter-Robot Distance by Planner",
        "meters",
        os.path.join(PLOTS_DIR, "avg_min_inter_robot_distance_by_planner.png"),
    )

    make_bar_plot(
        planner_labels,
        [0.0 if r["avg_path_length"] is None else r["avg_path_length"] for r in by_planner],
        "Average Path Length by Planner",
        "meters",
        os.path.join(PLOTS_DIR, "avg_path_length_by_planner.png"),
    )

    print()
    print("Saved files:")
    print(f"  {planner_csv}")
    print(f"  {scenario_planner_csv}")
    print(f"  {PLOTS_DIR}")


if __name__ == "__main__":
    main()

