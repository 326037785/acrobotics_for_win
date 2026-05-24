"""End-to-end Acrobotics introduction suitable for interactive debugging."""

import matplotlib.pyplot as plt
import numpy as np

import acrobotics as ab
from acrobotics.acrolib.geometry import translation
from acrobotics.acrolib.plotting import get_default_axes3d


def build_scene():
    table = ab.Box(2, 2, 0.1)
    obstacle = ab.Box(0.2, 0.2, 1.5)
    return ab.Scene(
        [table, obstacle],
        [translation(0, 0, -0.2), translation(0, 0.5, 0.55)],
    )


def main():
    robot = ab.Kuka()
    scene = build_scene()

    q_start = np.array([0.5, 1.5, -0.3, 0, 0, 0])
    q_goal = np.array([2.5, 1.5, 0.3, 0, 0, 0])
    q_path = np.linspace(q_start, q_goal, 10)
    collision_results = [robot.is_in_collision(q, scene) for q in q_path]
    print(f"Collision results: {collision_results}")

    q_example = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    tf_forward = robot.fk(q_example)
    ik_solution = robot.ik(tf_forward)
    print(f"Inverse kinematics successful? {ik_solution.success}")
    print(f"Number of inverse kinematics solutions: {len(ik_solution.solutions)}")

    fig, ax = get_default_axes3d([-0.8, 0.8], [-0.8, 0.8], [-0.2, 1.4])
    ax.set_axis_off()
    ax.view_init(elev=31, azim=-15)
    scene.plot(ax, c="green")
    robot.animate_path(fig, ax, q_path)
    plt.show()


if __name__ == "__main__":
    main()
