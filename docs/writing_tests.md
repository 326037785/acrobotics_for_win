# Writing your own Acrobotics tests

You can add your own Python test files under `tests/` and import the package
normally after an editable development install. The collision backend does
not require `python-fcl`, so this works on Windows without WSL.

## Set up the repository

From the repository root:

```powershell
python -m pip install -e ".[dev]"
python -m pytest
```

The editable install is important: changes under `src/acrobotics/` are used
immediately, and `import acrobotics` works in new test files without changing
`sys.path`.

## Add a pytest file

Pytest discovers files named `test_*.py`. For example, create
`tests/test_my_collision_case.py`:

```python
import numpy as np

import acrobotics as ab
from acrobotics.acrolib.geometry import translation


def test_my_obstacle_collision_case():
    robot = ab.Kuka()
    obstacle = ab.Box(0.2, 0.2, 1.0)
    scene = ab.Scene([obstacle], [translation(0, 0.5, 0.5)])
    q = np.array([0.5, 1.5, -0.3, 0, 0, 0])

    result = robot.is_in_collision(q, scene)

    assert isinstance(result, (bool, np.bool_))
```

Run only that file while iterating:

```powershell
python -m pytest tests/test_my_collision_case.py -q
```

## Use tests for checks, scripts for exploration

Use a `test_*.py` file when the result should be asserted and kept as a
regression test. For plotting, timing, or manually inspecting motion, a plain
script under `examples/` is usually clearer:

```powershell
python examples/my_experiment.py
```

Tests may import public conveniences from `acrobotics`, for example `Box`,
`Cylinder`, `Scene`, and `Kuka`. For lower-level extension work, import the
implementation modules directly:

```python
from acrobotics.link import DHLink, JointType, Link
from acrobotics.robot import Robot
```

## Collision model notes

`Box` is checked as an oriented convex box. `Cylinder` is checked as its
polygonal approximation; increase `approx_faces` if a rounder approximation
matters for the test. Path collision methods sample interpolated poses at a
resolution derived from the geometry, so keep narrow-clearance cases as
explicit tests.
