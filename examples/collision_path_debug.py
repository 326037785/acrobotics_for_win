"""Inspect swept collision checking along a robot joint-space path."""

import numpy as np

from acrobotics.acrolib.geometry import translation
from acrobotics.geometry import Scene
from acrobotics.robot_examples import Kuka
from acrobotics.shapes import Box
from acrobotics.tool_examples import torch2


def build_scene():
    table = Box(2, 2, 0.1)
    thin_obstacle = Box(0.01, 0.01, 1.5)
    return Scene(
        [table, thin_obstacle],
        [translation(0, 0, -0.2), translation(0, 0.5, 0.55)],
    )


def main():
    robot = Kuka()
    robot.tool = torch2
    scene = build_scene()

    q_start = np.array([1.0, 1.5, -0.3, 0, 0, 0])
    q_goal = np.array([2.0, 1.5, 0.3, 0, 0, 0])
    path_collides = robot.is_path_in_collision(q_start, q_goal, scene)

    print(f"Start configuration collides: {robot.is_in_collision(q_start, scene)}")
    print(f"Goal configuration collides: {robot.is_in_collision(q_goal, scene)}")
    print(f"Swept path collides: {path_collides}")
    assert path_collides


if __name__ == "__main__":
    main()
