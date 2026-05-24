"""Small Matplotlib plotting helpers."""

import numpy as np
import matplotlib.pyplot as plt


def get_default_axes3d(xlim=(-1, 1), ylim=(-1, 1), zlim=(-1, 1)):
    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    ax.set_xlim3d(xlim)
    ax.set_ylim3d(ylim)
    ax.set_zlim3d(zlim)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    return fig, ax


def plot_reference_frame(ax, tf=None, arrow_length=0.2):
    axes = [
        (np.array([[0, arrow_length], [0, 0], [0, 0]]), "r"),
        (np.array([[0, 0], [0, arrow_length], [0, 0]]), "g"),
        (np.array([[0, 0], [0, 0], [0, arrow_length]]), "b"),
    ]
    for points, color in axes:
        if tf is not None:
            points = tf[:3, :3] @ points + tf[:3, 3][:, None]
        ax.plot(points[0], points[1], points[2], "-", c=color)
