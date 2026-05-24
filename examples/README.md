# Debuggable Examples

The scripts in this directory are intentionally small and each starts from a
`main()` function. After installing the project in editable mode, open a
script in VS Code, place a breakpoint inside `main()`, and choose **Run and
Debug > Python File**.

```powershell
python -m pip install -e ".[dev]"
python examples/kinematics_debug.py
python examples/collision_path_debug.py
python examples/urdf_roundtrip_debug.py
python examples/getting_started.py
```

Suggested starting points:

| Script | What to inspect | Useful source breakpoint |
| --- | --- | --- |
| `kinematics_debug.py` | Forward and inverse kinematics | `src/acrobotics/robot_examples.py`, `Kuka.ik` |
| `collision_path_debug.py` | Swept collision against a thin obstacle | `src/acrobotics/robot.py`, `Robot.is_path_in_collision` |
| `urdf_roundtrip_debug.py` | URDF export and import without `urdfpy` | `src/acrobotics/urdfio.py`, `export_urdf` |
| `getting_started.py` | Full workflow plus animation | `src/acrobotics/geometry.py`, `Scene.is_in_collision` |

`getting_started.py` opens a Matplotlib animation window. The other scripts
are console-only and are convenient for short debugging sessions.
