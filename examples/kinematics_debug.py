"""Step through forward and inverse kinematics for a Kuka arm."""

import numpy as np

import acrobotics as ab


def main():
    robot = ab.Kuka()
    joint_values = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

    tool_pose = robot.fk(joint_values)
    result = robot.ik(tool_pose)

    print("Forward kinematics pose:")
    print(tool_pose)
    print(f"Inverse kinematics successful? {result.success}")
    print(f"Solutions found: {len(result.solutions)}")

    if result.success:
        reconstructed_pose = robot.fk(result.solutions[0])
        position_error = np.linalg.norm(reconstructed_pose[:3, 3] - tool_pose[:3, 3])
        print(f"First-solution position error: {position_error:.3e}")


if __name__ == "__main__":
    main()
