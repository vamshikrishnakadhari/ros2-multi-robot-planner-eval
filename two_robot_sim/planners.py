import math


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def distance_xy(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def heading_to_goal(robot):
    dx = robot["goal"][0] - robot["x"]
    dy = robot["goal"][1] - robot["y"]
    return math.atan2(dy, dx)


def goal_distance(robot):
    return distance_xy((robot["x"], robot["y"]), robot["goal"])


def nearest_robot_distance(robot, robots):
    min_dist = float("inf")
    for other in robots:
        if other["name"] == robot["name"]:
            continue
        d = distance_xy((robot["x"], robot["y"]), (other["x"], other["y"]))
        min_dist = min(min_dist, d)
    return min_dist if min_dist != float("inf") else 999.0


def unit_vector_to_goal(robot):
    dx = robot["goal"][0] - robot["x"]
    dy = robot["goal"][1] - robot["y"]
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return 0.0, 0.0
    return dx / d, dy / d


def _compute_drive_command(robot, target_heading, speed_scale, params):
    heading_error = wrap_angle(target_heading - robot["theta"])
    omega = clamp(
        params["k_theta"] * heading_error,
        -params["max_omega"],
        params["max_omega"],
    )

    heading_scale = max(0.0, 1.0 - abs(heading_error) / math.pi)
    v = robot["max_speed"] * heading_scale * speed_scale
    v = clamp(v, 0.0, robot["max_speed"])
    return v, omega


def baseline_planner(robot, robots, params):
    if robot["reached_goal"]:
        return 0.0, 0.0

    target_heading = heading_to_goal(robot)
    speed_scale = 1.0

    nearest_dist = nearest_robot_distance(robot, robots)
    if nearest_dist < params["slowdown_distance"]:
        scale = (nearest_dist - params["robot_radius"] * 2.0) / max(
            1e-6,
            params["slowdown_distance"] - params["robot_radius"] * 2.0,
        )
        speed_scale = clamp(scale, 0.20, 1.0)

    return _compute_drive_command(robot, target_heading, speed_scale, params)


def priority_planner(robot, robots, params):
    if robot["reached_goal"]:
        return 0.0, 0.0

    target_heading = heading_to_goal(robot)
    speed_scale = 1.0

    for other in robots:
        if other["name"] == robot["name"]:
            continue

        dx = other["x"] - robot["x"]
        dy = other["y"] - robot["y"]
        d = math.hypot(dx, dy)

        if d > params["yield_distance"]:
            continue

        angle_to_other = math.atan2(dy, dx)
        rel = wrap_angle(angle_to_other - robot["theta"])

        other_has_higher_priority = other["priority"] < robot["priority"]
        in_front_conflict = abs(rel) < 1.2

        if other_has_higher_priority and in_front_conflict:
            side_heading = angle_to_other - math.pi / 2.0
            target_heading = side_heading

            if d < params["collision_distance"] + 0.10:
                speed_scale = min(speed_scale, 0.0)
            else:
                speed_scale = min(speed_scale, max(params["yield_speed_scale"], 0.25))

        elif (not other_has_higher_priority) and in_front_conflict and d < (params["yield_distance"] * 0.7):
            speed_scale = min(speed_scale, 0.60)

    return _compute_drive_command(robot, target_heading, speed_scale, params)


def reciprocal_planner(robot, robots, params):
    if robot["reached_goal"]:
        return 0.0, 0.0

    gx, gy = unit_vector_to_goal(robot)
    vx = gx
    vy = gy

    speed_scale = 1.0
    closest_front_distance = 999.0

    for other in robots:
        if other["name"] == robot["name"]:
            continue

        dx = other["x"] - robot["x"]
        dy = other["y"] - robot["y"]
        d = math.hypot(dx, dy)

        if d >= params["avoid_distance"]:
            continue

        angle_to_other = math.atan2(dy, dx)
        rel = wrap_angle(angle_to_other - robot["theta"])

        # stronger response for robots roughly in front
        front_weight = 1.0 if abs(rel) < 1.6 else 0.35
        weight = ((params["avoid_distance"] - d) / params["avoid_distance"]) * front_weight

        away_x = -dx / max(d, 1e-6)
        away_y = -dy / max(d, 1e-6)

        # deterministic clockwise tangent bias to break symmetry
        tangent_x = away_y
        tangent_y = -away_x

        vx += 1.4 * weight * away_x + params["avoid_turn_gain"] * weight * tangent_x
        vy += 1.4 * weight * away_y + params["avoid_turn_gain"] * weight * tangent_y

        if abs(rel) < 1.6:
            closest_front_distance = min(closest_front_distance, d)

    target_heading = math.atan2(vy, vx)
    heading_error = wrap_angle(target_heading - robot["theta"])

    if closest_front_distance < params["avoid_distance"]:
        close_scale = clamp(closest_front_distance / params["avoid_distance"], 0.30, 1.0)
        speed_scale = min(speed_scale, close_scale)

    if abs(heading_error) > 1.2:
        speed_scale = min(speed_scale, 0.70)

    if goal_distance(robot) < 0.40:
        speed_scale = min(speed_scale, 0.60)

    return _compute_drive_command(robot, target_heading, speed_scale, params)


PLANNERS = {
    "baseline": baseline_planner,
    "priority": priority_planner,
    "reciprocal": reciprocal_planner,
}

