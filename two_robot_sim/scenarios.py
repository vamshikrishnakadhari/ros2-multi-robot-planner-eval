import math
import random


def make_robot(name, start, goal, max_speed=0.3):
    return {
        "name": name,
        "start": start,   # [x, y, theta]
        "goal": goal,     # [x, y]
        "max_speed": max_speed,
    }


def make_ring_swap(num_robots, radius=3.0, max_speed=0.3):
    robots = []

    for i in range(num_robots):
        angle = 2.0 * math.pi * i / num_robots
        goal_angle = angle + math.pi

        start_x = radius * math.cos(angle)
        start_y = radius * math.sin(angle)

        goal_x = radius * math.cos(goal_angle)
        goal_y = radius * math.sin(goal_angle)

        theta = math.atan2(goal_y - start_y, goal_x - start_x)

        robots.append(
            make_robot(
                name=f"robot{i+1}",
                start=[start_x, start_y, theta],
                goal=[goal_x, goal_y],
                max_speed=max_speed,
            )
        )

    return {
        "description": f"{num_robots} robots swap positions through the center.",
        "robots": robots,
    }


def _distance_xy(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _sample_points(count, low, high, min_separation, rng, max_tries=10000):
    points = []
    tries = 0

    while len(points) < count and tries < max_tries:
        tries += 1
        p = (rng.uniform(low, high), rng.uniform(low, high))

        ok = True
        for q in points:
            if _distance_xy(p, q) < min_separation:
                ok = False
                break

        if ok:
            points.append(p)

    if len(points) < count:
        raise ValueError(
            f"Could not place {count} points with min_separation={min_separation} "
            f"in range [{low}, {high}]. Try fewer robots or a larger world."
        )

    return points


def generate_random_scenario(
    num_robots=6,
    world_size=4.0,
    min_start_separation=1.2,
    min_goal_separation=1.2,
    min_start_goal_distance=2.0,
    max_speed=0.28,
    seed=None,
):
    rng = random.Random(seed)

    low = -world_size
    high = world_size

    starts = _sample_points(
        count=num_robots,
        low=low,
        high=high,
        min_separation=min_start_separation,
        rng=rng,
    )

    goals = _sample_points(
        count=num_robots,
        low=low,
        high=high,
        min_separation=min_goal_separation,
        rng=rng,
    )

    robots = []
    used_goals = set()

    for i, start in enumerate(starts):
        chosen_goal_idx = None
        best_dist = -1.0

        for j, goal in enumerate(goals):
            if j in used_goals:
                continue

            d = _distance_xy(start, goal)
            if d >= min_start_goal_distance and d > best_dist:
                chosen_goal_idx = j
                best_dist = d

        if chosen_goal_idx is None:
            raise ValueError(
                "Could not assign sufficiently separated random goals. "
                "Try reducing min_start_goal_distance or increasing world_size."
            )

        used_goals.add(chosen_goal_idx)
        goal = goals[chosen_goal_idx]

        theta = math.atan2(goal[1] - start[1], goal[0] - start[0])

        robots.append(
            make_robot(
                name=f"robot{i+1}",
                start=[start[0], start[1], theta],
                goal=[goal[0], goal[1]],
                max_speed=max_speed,
            )
        )

    return {
        "description": f"Random scenario with {num_robots} robots.",
        "robots": robots,
    }


SCENARIOS = {
    "head_on": {
        "description": "Two robots swap positions head-on.",
        "robots": [
            make_robot("robot1", [-3.0, 0.0, 0.0], [3.0, 0.0]),
            make_robot("robot2", [3.0, 0.0, math.pi], [-3.0, 0.0]),
        ],
    },
    "crossing": {
        "description": "Two robots cross at the center.",
        "robots": [
            make_robot("robot1", [-3.0, 0.0, 0.0], [3.0, 0.0]),
            make_robot("robot2", [0.0, -3.0, math.pi / 2], [0.0, 3.0]),
        ],
    },
    "three_robot_merge": {
        "description": "Three robots move into a shared region.",
        "robots": [
            make_robot("robot1", [-3.0, -1.0, 0.0], [2.0, 0.0]),
            make_robot("robot2", [-3.0, 1.0, 0.0], [2.0, 0.5]),
            make_robot("robot3", [0.0, 3.0, -math.pi / 2], [1.5, -1.0]),
        ],
    },
    "four_robot_cross": {
        "description": "Four robots approach the center from four directions.",
        "robots": [
            make_robot("robot1", [-3.0, 0.0, 0.0], [3.0, 0.0]),
            make_robot("robot2", [3.0, 0.0, math.pi], [-3.0, 0.0]),
            make_robot("robot3", [0.0, -3.0, math.pi / 2], [0.0, 3.0]),
            make_robot("robot4", [0.0, 3.0, -math.pi / 2], [0.0, -3.0]),
        ],
    },
    "six_robot_cross": make_ring_swap(6, radius=3.2, max_speed=0.28),
    "eight_robot_cross": make_ring_swap(8, radius=3.6, max_speed=0.24),
}

