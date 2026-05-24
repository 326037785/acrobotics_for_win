import numpy as np
from numpy.testing import assert_almost_equal

from acrobotics.urdfio import import_urdf, export_urdf

TEST_DIR = "./tests"  # is this robust??


def test_complete_in_out(tmp_path):
    scene = import_urdf("example", TEST_DIR)
    export_urdf(scene, "example_after", tmp_path)
    roundtrip = import_urdf("example_after", tmp_path)

    assert len(scene.shapes) == len(roundtrip.shapes)
    for original, actual in zip(scene.shapes, roundtrip.shapes):
        assert_almost_equal(
            np.array([original.dx, original.dy, original.dz]),
            np.array([actual.dx, actual.dy, actual.dz]),
        )
    for original, actual in zip(scene.tf_s, roundtrip.tf_s):
        assert_almost_equal(original, actual)

