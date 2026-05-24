[![Build Status](https://travis-ci.org/JeroenDM/acrobotics.svg?branch=master)](https://travis-ci.org/JeroenDM/acrobotics) [![codecov](https://codecov.io/gh/JeroenDM/acrobotics/branch/master/graph/badge.svg)](https://codecov.io/gh/JeroenDM/acrobotics) [![PyPI version](https://badge.fury.io/py/acrobotics.svg)](https://badge.fury.io/py/acrobotics)

# Acrobotics

Quickly test motion planning ideas is the goal, and Python seems like a great language for rapid prototyping. There are great libraries for robot simulation and related task, but installing them is can be a hassle and very dependent on operating system and python version.
The drawback is that I have to write a lot of stuff myself. I'm not sure if it is useful to do this. But it will be fun and I will learn a bunch.

This library provides robot kinematics and collision checking for serial kinematic chains. The idea is that this library can be easily swapped by another one providing the same functionality.

The acro part comes from [ACRO](https://iiw.kuleuven.be/onderzoek/acro) a robotics research group at KU Leuven in Belgium.

## Installation

```bash
pip install acrobotics
```

Or for development

```bash
git clone https://github.com/JeroenDM/acrobotics.git
cd acrobotics
python -m pip install -e ".[dev]"
```

Collision checking is implemented in NumPy and SciPy without a native
`python-fcl` dependency, so installation is supported on Windows as well as
Linux and macOS. `Box` collision checks are convex polyhedron tests.
`Cylinder` collision checks use the polygonal approximation configured by
`approx_faces`. Swept collision checks sample the interpolated pose path with
step sizes scaled to the colliding geometry. The small helper API formerly
supplied by `acrolib` is bundled under `acrobotics.acrolib`, avoiding its
legacy build dependency and conflicts with old installed versions.

## Gettings started

(Code for example below: [examples/getting_started.py](examples/getting_started.py).
Additional scripts designed for VS Code step debugging are listed in
[examples/README.md](examples/README.md).)

This library has three main tricks.

### Robot kinematics
`T = robot.fk(joint_values)`
`IKSolution = robot.ik(T)`

Forward kinematics are implemented in a generic `RobotKinematics` class.
```python
import acrobotics as ab

robot = ab.Kuka()

joint_values = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
T_fk = robot.fk(joint_values)
```

Analytical inverse kinematics only for specific robots:
```python
ik_solution = robot.ik(T_fk)  # T_fk is a numpy 4x4 array

print(f"Inverse kinematics successful? {ik_solution.success}")
for q in ik_solution.solutions:
    print(q)
```
```bash
Inverse kinematics successful? True
[ 0.1        -1.0949727   2.84159265  2.87778828  0.79803563 -1.99992985]
[ 0.1        -1.0949727   2.84159265 -0.26380438 -0.79803563  1.1416628 ]
[0.1 0.2 0.3 0.4 0.5 0.6]
[ 0.1         0.2         0.3        -2.74159265 -0.5        -2.54159265]
```

### Collision checking
`bool = robot.is_in_collision(joint_values, planning_scene)`

First create a planning scene with obstacles the robot can collide with.
```python
from acrobotics.acrolib.geometry import translation

table = ab.Box(2, 2, 0.1)
T_table = translation(0, 0, -0.2)

obstacle = ab.Box(0.2, 0.2, 1.5)
T_obs = translation(0, 0.5, 0.55)

scene = ab.Scene([table, obstacle], [T_table, T_obs])
```

Then create a list of robot configurations for wich you want to check collision with the planning scene.
```python
import numpy as np

q_start = np.array([0.5, 1.5, -0.3, 0, 0, 0])
q_goal = np.array([2.5, 1.5, 0.3, 0, 0, 0])
q_path = np.linspace(q_start, q_goal, 10)
```

And then you could do:
```python
print([robot.is_in_collision(q, scene) for q in q_path])
```
```bash
[False, False, False, False, True, True, True, True, False, False]
```

### Visualization
`robot.plot(axes_handle, joint_values)`
`robot.animate_path(figure_handle, axes_handle, joint_path)`

```python
from acrobotics.acrolib.plotting import get_default_axes3d

fig, ax = get_default_axes3d()

scene.plot(ax, c="green")
robot.animate_path(fig, ax, q_path)
```

![animation](examples/robot_animation.gif)

## More details

There's a more in depth explanation in the jupyter-notebooks in the examples folder.

Most of the usefull stuff can be imported similar to common numpy usage:
```Python
import acrobotics as ab
```
For more advanced classes, such as `Robot` to create a custom robot, you have to explicitly import them:
```Python
from acrobotics.robot import Robot
from acrobotics.link import DHLink, JointType, Link
```

## Writing tests and experiments

After installing the repository in editable mode, files under `tests/` can
import `acrobotics` normally on Windows and other supported platforms. See
[docs/writing_tests.md](docs/writing_tests.md) for a minimal test example,
commands, and advice for experiment scripts.

## And motion planning?

The package implements a basic sampling-based and optimization-based planner. Examples on how to use them can be found in the test folder, in [test_planning_sampling_based.py](tests/test_planning_optimization_based.py) and [test_planning_optimization_based.py](tests/test_planning_optimization_based.py). However, there is a non-trivial amount of setting types you have to supply to get it working. These appeared after a major refactor in an attempt to make to code more maintainable, but we went a bit overboard in the settings department...
