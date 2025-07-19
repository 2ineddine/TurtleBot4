import numpy as np

def rotation_axis_and_angle(R):
    # Check if R is a valid rotation matrix
    det = np.linalg.det(R)
    if np.isclose(det, -1):
        raise ValueError("Invalid rotation matrix: reflection detected (det(R) = -1).")
    
    # Compute the angle
    theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))

    # Handle special case when angle is ~0 (no rotation)
    if np.isclose(theta, 0):
        return np.array([1, 0, 0]), 0  # Arbitrary axis

    # Handle special case when angle is ~π (180° rotation)
    if np.isclose(theta, np.pi):
        # Find axis using diagonal elements
        axis = np.sqrt((np.diag(R) + 1) / 2)
        # Fix signs
        axis[0] = np.copysign(axis[0], R[2,1] - R[1,2])
        axis[1] = np.copysign(axis[1], R[0,2] - R[2,0])
        axis[2] = np.copysign(axis[2], R[1,0] - R[0,1])
        return axis / np.linalg.norm(axis), theta

    # General case
    axis = np.array([
        R[2,1] - R[1,2],
        R[0,2] - R[2,0],
        R[1,0] - R[0,1]
    ]) / (2 * np.sin(theta))

    return axis / np.linalg.norm(axis), theta
