import csv
import os
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def read_csv(csv_path):
    times = []
    r1x, r1y = [], []
    r2x, r2y = [], []
    dist = []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row['time']))
            r1x.append(float(row['robot1_x']))
            r1y.append(float(row['robot1_y']))
            r2x.append(float(row['robot2_x']))
            r2y.append(float(row['robot2_y']))
            dist.append(float(row['distance_between_robots']))

    return times, r1x, r1y, r2x, r2y, dist


def main():
    base_dir = os.path.expanduser('~/multi_robot_ws/src/two_robot_sim/results')

    candidate_files = [
        'head_on_priority_run.csv',
        'head_on_run.csv',
    ]

    csv_path = None
    for name in candidate_files:
        p = os.path.join(base_dir, name)
        if os.path.exists(p):
            csv_path = p
            break

    if csv_path is None:
        print('No CSV file found in:', base_dir)
        return

    print('Reading:', csv_path)
    times, r1x, r1y, r2x, r2y, dist = read_csv(csv_path)

    if len(times) == 0:
        print('CSV is empty.')
        return

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.set_title('Two-Robot Conflict Scenario')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_xlim(-2.5, 2.5)
    ax.set_ylim(-1.5, 1.5)
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')

    conflict_box_x = [-0.3, 0.3, 0.3, -0.3, -0.3]
    conflict_box_y = [-0.3, -0.3, 0.3, 0.3, -0.3]
    ax.plot(conflict_box_x, conflict_box_y, '--', label='Conflict zone')

    ax.plot(r1x[0], r1y[0], 'go', label='Robot1 start')
    ax.plot(r2x[0], r2y[0], 'mo', label='Robot2 start')
    ax.plot(2.0, 0.0, 'gx', markersize=10, label='Robot1 goal')
    ax.plot(-2.0, 0.0, 'mx', markersize=10, label='Robot2 goal')

    ax.plot(r1x, r1y, 'g-', alpha=0.35, label='Robot1 path')
    ax.plot(r2x, r2y, 'm-', alpha=0.35, label='Robot2 path')

    robot1_point, = ax.plot([], [], 'go', markersize=10)
    robot2_point, = ax.plot([], [], 'mo', markersize=10)

    robot1_trail, = ax.plot([], [], 'g-', linewidth=2)
    robot2_trail, = ax.plot([], [], 'm-', linewidth=2)

    status_text = ax.text(
        0.02, 0.95, '', transform=ax.transAxes, verticalalignment='top'
    )

    ax.legend(loc='upper right')

    def init():
        robot1_point.set_data([], [])
        robot2_point.set_data([], [])
        robot1_trail.set_data([], [])
        robot2_trail.set_data([], [])
        status_text.set_text('')
        return robot1_point, robot2_point, robot1_trail, robot2_trail, status_text

    def update(frame):
        robot1_point.set_data([r1x[frame]], [r1y[frame]])
        robot2_point.set_data([r2x[frame]], [r2y[frame]])

        robot1_trail.set_data(r1x[:frame + 1], r1y[:frame + 1])
        robot2_trail.set_data(r2x[:frame + 1], r2y[:frame + 1])

        status_text.set_text(
            f't = {times[frame]:.1f}s\n'
            f'distance = {dist[frame]:.2f}'
        )

        return robot1_point, robot2_point, robot1_trail, robot2_trail, status_text

    ani = FuncAnimation(
        fig,
        update,
        frames=len(times),
        init_func=init,
        interval=100,
        blit=True,
        repeat=False
    )

    plt.show()


if __name__ == '__main__':
    main()
