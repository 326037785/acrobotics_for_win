"""Export and import a planning scene using the lightweight URDF support."""

from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from acrobotics.acrolib.geometry import translation
from acrobotics.geometry import Scene
from acrobotics.shapes import Box
from acrobotics.urdfio import export_urdf, import_urdf


def build_scene():
    return Scene(
        [Box(0.5, 0.2, 0.1), Box(0.1, 0.1, 0.7)],
        [translation(0, 0, 0.05), translation(0.4, 0, 0.35)],
    )


def main():
    scene = build_scene()

    with TemporaryDirectory() as temporary_directory:
        output_directory = Path(temporary_directory)
        export_urdf(scene, "debug_scene", output_directory)
        imported_scene = import_urdf("debug_scene", output_directory)

        transforms_match = all(
            np.allclose(original, imported)
            for original, imported in zip(scene.tf_s, imported_scene.tf_s)
        )
        print(f"URDF output: {output_directory / 'debug_scene.urdf'}")
        print(f"Number of imported shapes: {len(imported_scene.shapes)}")
        print(f"Transforms preserved: {transforms_match}")
        assert transforms_match


if __name__ == "__main__":
    main()
