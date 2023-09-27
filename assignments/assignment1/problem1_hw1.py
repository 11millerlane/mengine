import os
import numpy as np
import mengine as m
np.set_printoptions(precision=3, suppress=True)

# NOTE: This problem asks you to convert between the different rotation representations.

# Create environment and ground plane
env = m.Env()
ground = m.Ground([0, 0, -0.5])
env.set_gui_camera(look_at_pos=[0, 0, 0])

# position definition
x = np.array([0.2, 0, 0])

# Create points to rotate
# point rotated using euler angles
point_e = m.Shape(m.Sphere(radius=0.03), static=True,
                  position=x, rgba=[0, 1, 0, 0.2])
# point rotated using axis-angle
point_aa = m.Shape(m.Sphere(radius=0.025), static=True,
                   position=x, rgba=[1, 0, 0, 0.2])
# point rotated using rotation matrix
point_r = m.Shape(m.Sphere(radius=0.02), static=True,
                  position=x, rgba=[0, 0, 1, 0.2])


def rodrigues_formula(n, x, theta):
    # Rodrigues' formula for axis-angle: rotate a point x around an axis n by angle theta
    # input: n, x, theta: axis, point, angle
    # output: x_new: new point after rotation
    # ------ TODO Student answer below -------
    n = np.array(n)
    n = n.T #I think this fixes the dimensionality errors
    x = np.array(x)
    x_new = n * (n.dot(x)) + np.sin(theta)*(np.cross(n,x)) - np.cross(np.cos(theta)*n, (np.cross(n,x)))
    return x_new
    #return np.zeros(3)
    # ------ Student answer above -------

def rotate_euler(alpha, beta, gamma, x):
    # Rotate a point x using euler angles (alpha, beta, gamma)
    # input: alpha, beta, gamma: euler angles
    # output: x_new: new point after rotation

    # ------ TODO Student answer below -------
    a = alpha
    b = beta
    g = gamma
    R = np.array([  [np.cos(a)*np.cos(b)*np.cos(g)-np.sin(a)*np.sin(g),   -np.cos(g)*np.sin(a)-np.cos(a)*np.cos(b)*np.sin(g),   np.cos(a)*np.sin(b)],
                    [np.cos(a)*np.sin(g)+np.cos(b)*np.cos(g)*np.sin(a),   np.cos(a)*np.cos(g)-np.cos(b)*np.sin(a)*np.sin(g),    np.sin(a)*np.sin(b)],
                    [-np.cos(g)*np.sin(b),                                np.sin(b)*np.sin(g),                                  np.cos(b)]])
    x_new = np.matmul(R, x.T)
    return x_new
    # return np.zeros(3)
    # ------ Student answer above -------


def euler_to_rotation_matrix(alpha, beta, gamma):
    # Convert euler angles (alpha, beta, gamma) to rotation matrix
    # input: alpha, beta, gamma: euler angles
    # output: R: rotation matrix

    # ------ TODO Student answer below -------
    a = alpha
    b = beta
    g = gamma
    R = np.array([  [np.cos(a)*np.cos(b)*np.cos(g)-np.sin(a)*np.sin(g),   -np.cos(g)*np.sin(a)-np.cos(a)*np.cos(b)*np.sin(g),   np.cos(a)*np.sin(b)],
                    [np.cos(a)*np.sin(g)+np.cos(b)*np.cos(g)*np.sin(a),   np.cos(a)*np.cos(g)-np.cos(b)*np.sin(a)*np.sin(g),    np.sin(a)*np.sin(b)],
                    [-np.cos(g)*np.sin(b),                                np.sin(b)*np.sin(g),                                  np.cos(b)]])
    return R
    #return np.zeros((3,3))
    # ------ Student answer above -------


def euler_to_axis_angle(alpha, beta, gamma):
    # Convert euler angles (alpha, beta, gamma) to axis-angle representation (n, theta)
    # input: alpha, beta, gamma: euler angles
    # output: n, theta
    # ------ TODO Student answer below -------
    a = alpha
    b = beta
    g = gamma
    R = np.array([  [np.cos(a)*np.cos(b)*np.cos(g)-np.sin(a)*np.sin(g),   -np.cos(g)*np.sin(a)-np.cos(a)*np.cos(b)*np.sin(g),   np.cos(a)*np.sin(b)],
                    [np.cos(a)*np.sin(g)+np.cos(b)*np.cos(g)*np.sin(a),   np.cos(a)*np.cos(g)-np.cos(b)*np.sin(a)*np.sin(g),    np.sin(a)*np.sin(b)],
                    [-np.cos(g)*np.sin(b),                                np.sin(b)*np.sin(g),                                  np.cos(b)]])
    theta = np.arccos((np.trace(R))/2)
    n_x = (R[2][1]-R[1][2])/(2*np.sin(theta))
    n_y = (R[0][2]-R[2][0])/(2*np.sin(theta))
    n_z = (R[1][0]-R[0][1])/(2*np.sin(theta))
    n = [n_x,n_y,n_z]
    return n, theta
    # return np.zeros(3), 0
    # ------ Student answer above -------


x_new_e = np.array([0.2, 0, 0])
x_new_r = np.array([0.2, 0, 0])
x_new_aa = np.array([0.2, 0, 0])

for alpha, beta, gamma in zip([20, -25, 0], [45, 5, 135], [10, 90, -72]):
    alpha = np.radians(alpha)
    beta = np.radians(beta)
    gamma = np.radians(gamma)

    (n, theta) = euler_to_axis_angle(alpha, beta, gamma)
    R = euler_to_rotation_matrix(alpha, beta, gamma)

    # positions of rotated points for each representation
    x_new_e = rotate_euler(alpha, beta, gamma, x)
    x_new_r = R.dot(x)
    x_new_aa = rodrigues_formula(n, x, theta)

    print('-'*20)
    print('Euler angles:', np.degrees(alpha), np.degrees(beta), np.degrees(gamma))
    print('Axis angle:', n, np.degrees(theta))
    print('Rotation matrix:', R)
    print('x_new_e:', x_new_e)
    print('x_new_r:', x_new_r)
    print('x_new_aa:', x_new_aa)
    print('-'*20)

    point_e.set_base_pos_orient(x_new_e)
    point_r.set_base_pos_orient(x_new_r)
    point_aa.set_base_pos_orient(x_new_aa)

    m.step_simulation(realtime=True)
    input("Press enter to continue...")
