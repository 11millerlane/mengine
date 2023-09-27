import numpy as np
import os
import math
import mengine as m

"""
16-741 Assignment 2 Problem 1.

Attention: quaternions are represented as [x, y, z, w], same as in pybullet.

"""

np.set_printoptions(precision=4, suppress=True)


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert rotation matrix R (3x3) to quaternion q (1x4)."""
    # input: R: rotation matrix
    # output: q: quaternion
    # ------ TODO: Student answer below -------
    # Establish the naming of indeces for our naming conventions
    r11, r12, r13 = R[0][0], R[0][1], R[0][2]
    r21, r22, r23 = R[1][0], R[1][1], R[1][2]
    r31, r32, r33 = R[2][0], R[2][1], R[2][2]
    # Look at diagonal elements, q_i squared 
    q0sq = 1/4 * (1 + r11 + r22 + r33)
    q1sq = 1/4 * (1 + r11 - r22 - r33)
    q2sq = 1/4 * (1 - r11 + r22 - r33)
    q3sq = 1/4 * (1 - r11 - r22 + r33)
    qsq_list = [q0sq, q1sq, q2sq, q3sq]
    # Find the largest qi squared
    max_index = qsq_list.index(max(qsq_list))
    # Step 2 of the procedure from lecture
    if max_index == 0:
        q_0 = np.sqrt(max(qsq_list))
        q_1 = 1/4* (r32-r23) / q_0
        q_2 = 1/4* (r13-r31) / q_0
        q_3 = 1/4* (r21-r12) / q_0
    elif max_index == 1:
        q_1 = np.sqrt(max(qsq_list))
        q_0 = 1/4* (r32-r23) / q_1
        q_2 = 1/4* (r12+r21) / q_1
        q_3 = 1/4* (r13+r31) / q_1
    elif max_index == 2:
        q_2 = np.sqrt(max(qsq_list))
        q_0 = 1/4* (r13-r31) / q_2
        q_1 = 1/4* (r12+r21) / q_2
        q_3 = 1/4* (r23+r32) / q_2
    elif max_index == 3:
        q_3 = np.sqrt(max(qsq_list))
        q_0 = 1/4* (r21-r12) / q_3
        q_1 = 1/4* (r13+r31) / q_3
        q_2 = 1/4* (r23+r32) / q_3
    else:
        print("rotation_matrix_to_quaternion has malfunctioned!")
    # Returning q in the form [x, y, z, w]
    q_return = np.array([q_1, q_2, q_3, q_0])
    return q_return
    #return np.array([0, 0, 1, 0])
    # ------ Student answer above -------


def rodrigues_formula(n, x, theta):
    # Rodrigues' formula for axis-angle: rotate a point x around an axis n by angle theta
    # input: n, x, theta: axis, point, angle
    # output: x_new: new point after rotation
    # ------ TODO Student answer below -------
    x_new = n*n.dot(x) + np.sin(theta)*np.cross(n, x) - np.cos(theta)*np.cross(n, np.cross(n, x))
    #return np.zeros(3)
    # ------ Student answer above -------


def axis_angle_to_quaternion(axis: np.ndarray, angle: float) -> np.ndarray:
    """Convert axis-angle representation to quaternion."""
    # input: axis: axis of rotation
    #        angle: angle of rotation (radians)
    # output: q: quaternion
    # ------ TODO: Student answer below -------
    q = [np.sin(angle/2)*axis[0], np.sin(angle/2)*axis[1], np.sin(angle/2)*axis[2],np.cos(angle/2)]
    return q
    #return np.array([0, 0, 1, 0])
    # ------ Student answer above -------


def hamilton_product(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    # ------ TODO: Student answer below -------
    #pq = r
    p0, p1, p2, p3 = p[3], p[0], p[1], p[2]
    q0, q1, q2, q3 = q[3], q[0], q[1], q[2]
    r0 = p0*q0 - p1*q1 - p2*q2 - p3*q3
    r1 = p0*q1 + q0*p1 + p2*q3 - p3*q2
    r2 = p0*q2 + q0*p2 + p3*q1 - p1*q3
    r3 = p0*q3 + q0*p3 + p1*q2 - p2*q1
    return [r1, r2, r3, r0]
    #return np.array([0, 0, 1, 0])
    # ------ Student answer above -------


def unit_tests():
    """Simple unit tests.
    Passing these test cases does NOT ensure your implementation is fully correct.
    """
    # test rotation_matrix_to_quaternion
    q = rotation_matrix_to_quaternion(np.diag([1, -1, -1]))
    try:
        assert np.allclose(q, [1, 0, 0, 0]) or np.allclose(q, [-1, 0, 0, 0])
        print("rotation_matrix_to_quaternion passed test case 1")
    except AssertionError:
        print("rotation_matrix_to_quaternion failed test case 1")

    R = np.array([[-0.545, 0.797, 0.260],
                  [0.733, 0.603, -0.313],
                  [-0.407, 0.021, -0.913]])
    q = rotation_matrix_to_quaternion(R)
    try:
        assert np.allclose(q, [0.437, 0.875, -0.0836, 0.191], atol=1e-3)
        print("rotation_matrix_to_quaternion passed test case 2")
    except AssertionError:
        print("rotation_matrix_to_quaternion failed test case 2")

    # test axis_angle_to_quaternion
    q = axis_angle_to_quaternion(np.array([1, 0, 0]), 0.123)
    try:
        assert np.allclose(q, [0.06146124, 0, 0, 0.99810947])
        print("axis_angle_to_quaternion passed test case")
    except AssertionError:
        print("axis_angle_to_quaternion failed test case")

    # test hamilton_product
    p = np.array([0.437, 0.875, -0.0836, 0.191])
    q = np.array([0.06146124, 0, 0, 0.99810947])
    try:
        assert np.allclose(hamilton_product(p, q),
                           [0.4479,  0.8682, -0.1372,  0.1638], atol=1e-3)
        print("hamilton_product passed test case")
    except AssertionError:
        print("hamilton_product failed test case")


if __name__ == '__main__':
    # Create environment and ground plane
    env = m.Env()
    ground = m.Ground([0, 0, -0.5])
    env.set_gui_camera(look_at_pos=[0, 0, 0])

    # Axis-angle definition
    n = np.array([0, 0, 1])
    x = np.array([0.2, 0, 0])

    # Create axis
    axis = m.Shape(m.Cylinder(radius=0.02, length=0.5), static=True, position=[0, 0, 0], orientation=n,
                   rgba=[0.8, 0.8, 0.8, 1])
    # Create point to rotate around axis
    point = m.Shape(m.Sphere(radius=0.02), static=True,
                    position=x, rgba=[0, 0, 1, 0.5])
    point_q = m.Shape(m.Sphere(radius=0.02), static=True,
                      position=x, rgba=[0, 1, 0, 0.5])

    # First we want to implement some converstions and the Hamilton product for quaternions.
    print("Running unit tests...")
    unit_tests()

    x_new_report = []
    x_new_q_report = []

    for i in range(10000):
        theta = np.radians(i)
        # Rodrigues' formula for axis-angle rotation
        x_new = rodrigues_formula(n, x, theta)

        # Axis-angle to quaternion
        theta = np.radians(i-10)  # Offset theta so we can see the two points
        q = axis_angle_to_quaternion(n, theta)

        # rotate using quaternion and the hamilton product
        # ------ TODO Student answer below -------
        qstar = [-q[0],-q[1],-q[2],q[3]]
        x_new_q = hamilton_product(hamilton_product(q, x_new_q), qstar)
        #x_new_q = np.zeros(3)
        # ------ Student answer above -------

        point.set_base_pos_orient(x_new)
        point_q.set_base_pos_orient(x_new_q[:3])

        m.step_simulation(realtime=True)

        if i % 50 == 0 and i < 501:
            x_new_report.append(x_new.tolist())
            x_new_q_report.append(x_new_q[:3].tolist())
        if i == 500:
            print("Point rotated using rodrigues formula: ")
            for row in x_new_report:
                formatted_row = [f"{elem:.4f}" for elem in row]
                print(formatted_row)
            print("Point rotated using hamilton product: ")
            for row in x_new_q_report:
                formatted_row = [f"{elem:.4f}" for elem in row]
                print(formatted_row)
