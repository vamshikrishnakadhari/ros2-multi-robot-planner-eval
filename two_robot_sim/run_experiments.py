import csv
import os
import shlex
import subprocess
import sys
import time
from datetime import datetime


RESULTS_DIR = os.path.expanduser("~/multi_robot_ws/results")
SUMMARY_CSV = os.path.join(RESULTS_DIR, "summary.csv")


def build_experiments():
    experiments = []

    planners = ["baseline", "priority", "reciprocal"]

    # Deterministic scenarios
    for planner in planners:
        experiments.append({
            "scenario": "head_on",
            "planner": planner,
            "num_robots": 2,
            "random_seed": 0,
            "max_steps": 1500,
            "avoid_distance": 2.0,
            "avoid_turn_gain": 1.2,
            "collision_distance": 0.45,
        })

        experiments.append({
            "scenario": "four_robot_cross",
            "planner": planner,
            "num_robots": 4,
            "random_seed": 0,
            "max_steps": 2500,
            "avoid_distance": 2.0,
            "avoid_turn_gain": 1.2,
            "collision_distance": 0.45,
        })

    # Random scenarios
    for planner in planners:
        for num_robots in [4, 6]:
            for seed in [42, 99]:
                experiments.append({
                    "scenario": "random",
                    "planner": planner,
                    "num_robots": num_robots,
                    "random_seed": seed,
                    "max_steps": 2500,
                    "avoid_distance": 2.0,
                    "avoid_turn_gain": 1.2,
                    "collision_distance": 0.45,
                })

    return experiments


def make_command(exp):
    return [
        "ros2", "run", "two_robot_sim", "sim_node",
        "--ros-args",
        "-p", f"scenario:={exp['scenario']}",
        "-p", f"num_robots:={exp['num_robots']}",
        "-p", f"random_seed:={exp['random_seed']}",
        "-p", f"planner:={exp['planner']}",
        "-p", f"max_steps:={exp['max_steps']}",
        "-p", f"avoid_distance:={exp['avoid_distance']}",
        "-p", f"avoid_turn_gain:={exp['avoid_turn_gain']}",
        "-p", f"collision_distance:={exp['collision_distance']}",
    ]


def read_summary_count():
    if not os.path.exists(SUMMARY_CSV):
        return 0
    with open(SUMMARY_CSV, newline="") as f:
        return max(sum(1 for _ in f) - 1, 0)


def read_last_summary_row():
    if not os.path.exists(SUMMARY_CSV):
        return None

    with open(SUMMARY_CSV, newline="") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        return None
    return rows[-1]


def matches_experiment(row, exp):
    if row is None:
        return False

    return (
        str(row.get("scenario", "")) == str(exp["scenario"])
        and str(row.get("planner", "")) == str(exp["planner"])
        and int(row.get("num_robots", -1)) == int(exp["num_robots"])
    )


def print_experiment_header(index, total, exp):
    print("=" * 90)
    print(
        f"[{index}/{total}] "
        f"scenario={exp['scenario']} "
        f"planner={exp['planner']} "
        f"num_robots={exp['num_robots']} "
        f"seed={exp['random_seed']}"
    )
    print("=" * 90)


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)

    experiments = build_experiments()
    total = len(experiments)

    print(f"Starting batch experiments at {datetime.now().isoformat(timespec='seconds')}")
    print(f"Results dir: {RESULTS_DIR}")
    print(f"Summary CSV: {SUMMARY_CSV}")
    print(f"Total experiments: {total}")
    print()

    succeeded = 0
    failed = 0
    appended_rows = 0
    failures = []

    for idx, exp in enumerate(experiments, start=1):
        print_experiment_header(idx, total, exp)

        cmd = make_command(exp)
        print("Command:")
        print("  " + shlex.join(cmd))
        print()

        before_count = read_summary_count()
        t0 = time.time()

        try:
            result = subprocess.run(cmd, check=False)
            elapsed = time.time() - t0

            after_count = read_summary_count()
            row_appended = after_count > before_count
            last_row = read_last_summary_row()

            if result.returncode == 0:
                succeeded += 1
                print(f"Run finished with exit code 0 in {elapsed:.2f}s")
            else:
                failed += 1
                failures.append({
                    "experiment": exp,
                    "returncode": result.returncode,
                })
                print(f"Run failed with exit code {result.returncode} in {elapsed:.2f}s")

            if row_appended:
                appended_rows += 1
                print("Summary row appended.")
                if matches_experiment(last_row, exp):
                    print(
                        "Last summary row: "
                        f"success_all={last_row.get('success_all')} "
                        f"collision_happened={last_row.get('collision_happened')} "
                        f"steps={last_row.get('steps')} "
                        f"sim_time={last_row.get('sim_time')} "
                        f"run_csv={last_row.get('run_csv')}"
                    )
                else:
                    print("Warning: summary.csv changed, but last row does not exactly match this experiment.")
            else:
                print("Warning: no new summary row detected.")

        except KeyboardInterrupt:
            print("\nBatch interrupted by user.")
            break
        except Exception as e:
            failed += 1
            failures.append({
                "experiment": exp,
                "returncode": None,
                "error": str(e),
            })
            print(f"Exception while running experiment: {e}")

        print()

    print("#" * 90)
    print("BATCH RUN COMPLETE")
    print("#" * 90)
    print(f"Successful process exits: {succeeded}")
    print(f"Failed process exits:     {failed}")
    print(f"Summary rows appended:    {appended_rows}")
    print(f"Summary CSV path:         {SUMMARY_CSV}")

    if failures:
        print()
        print("Failures:")
        for item in failures:
            exp = item["experiment"]
            print(
                f"  scenario={exp['scenario']} "
                f"planner={exp['planner']} "
                f"num_robots={exp['num_robots']} "
                f"seed={exp['random_seed']} "
                f"returncode={item.get('returncode')} "
                f"error={item.get('error', '')}"
            )

    return 0


if __name__ == "__main__":
    sys.exit(main())

