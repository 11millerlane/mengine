import time
import os
import mengine as m
import numpy as np


def invertQ(q):
    """
    Invert a quaternion
    """
    # ------ TODO Student answer below -------
    print("q=",q)
    qstar = [-q[0],-q[1],-q[2],q[3]]
    abs_qsq = q[0]**2+q[1]**2+q[2]**2+q[3]**2
    return qstar / abs_qsq
    return np.array([0, 0, 0, 1])
    # ------ Student answer above -------


def line_intersection(p1, p2, q1, q2):
    """
    Find the intersection of two 3D line segments p1-p2 and q1-q2.
    If there is an intersection, returns the point. Otherwise, returns None.
    """
    # ------ TODO Student answer below -------
    # Renaming the components:
    p1x, p1y, p1z = p1[0], p1[1], p1[2]
    p2x, p2y, p2z = p2[0], p2[1], p2[2]
    q1x, q1y, q1z = q1[0], q1[1], q1[2]
    q2x, q2y, q2z = q2[0], q2[1], q2[2]
    # Line P (p1-p2) in parametric form (on s from 0 to 1):
    pa = p2x - p1x
    pb = p2y - p1y
    pc = p2z - p1z
    """
    xp = p1x + pa*s
    yp = p1y + pb*s
    zp = p1z + pc*s
    """;
    # Line Q in parametric form (on t from 0 to 1):
    qa = q2x - q1x
    qb = q2y - q1y
    qc = q2z - q1z
    """
    xq = q1x + qa*t
    yq = q1y + qb*t
    zq = q1z + qc*t
    """;
    # Solve the System of Equations
    a = np.array([[pa,-qa],[pb,-qb]])
    b = np.array([q1x-p1x,q1y-p1y])
    sol = np.linalg.solve(a,b)
    s, t = sol[0], sol[1]
    atol = 1e-3 # Tolerance of 1e-3
    if atol >= ((pc * s - qc * t) - (q1z - p1z)) >= -atol: #Check with Equation 3
        xp = p1x + pa*s
        yp = p1y + pb*s
        zp = p1z + pc*s
        point = [xp,yp,zp]
        return point
    else:
        return None
    #return None
    # ------ Student answer above -------


# Create environment and ground plane
env = m.Env()
# ground = m.Ground()
env.set_gui_camera(look_at_pos=[0, 0, 0], yaw=30)

fbl = m.URDF(filename=os.path.join(m.directory, 'fourbarlinkage.urdf'),
             static=True, position=[0, 0, 0.3], orientation=[0, 0, 0, 1])
fbl.controllable_joints = [0, 1, 2]
# Create a constraint for the 4th joint to create a closed loop
fbl.create_constraint(parent_link=1, child=fbl, child_link=4, joint_type=m.p.JOINT_POINT2POINT, joint_axis=[
                      0, 0, 0], parent_pos=[0, 0, 0], child_pos=[0, 0, 0])
m.step_simulation(steps=20, realtime=False)

coupler_links = [1, 3, 5]

links = [1, 3]
global_points = []
previous_global_points = []
lines = [None, None]
lines_start_end = [[[0, 0, 0], [0, 0, 0]], [[0, 0, 0], [0, 0, 0]]]

for link in links:
    global_points.append(fbl.get_link_pos_orient(link)[0])
    previous_global_points.append(global_points[-1])
    point = m.Shape(m.Sphere(radius=0.02), static=True,
                    position=global_points[-1], rgba=[0, 0, 1, 1])

intersect_points_local = []
intersect_points_local_bodies = []

for i in range(10000):
    fbl.control([np.radians(i)]*3)

    if i > 3:
        for j, (link, global_position, previous_global_position) in enumerate(zip(links, global_points, previous_global_points)):
            p_new = fbl.get_link_pos_orient(link)[0]
            ic_vector_of_motion = p_new - previous_global_position
            ic_bisector = np.cross(ic_vector_of_motion, [0, 1, 0])
            ic_bisector = ic_bisector / np.linalg.norm(ic_bisector)
            previous_global_points[j] = p_new

            lines[j] = m.Line(p_new-ic_bisector, p_new+ic_bisector,
                              radius=0.005, rgba=[0, 0, 1, 0.5], replace_line=lines[j])
            lines_start_end[j] = (p_new-ic_bisector, p_new+ic_bisector)

        if len(intersect_points_local) < 400:
            # stop drawing if we have drawn 500 points
            intersect_point = line_intersection(
                lines_start_end[0][0], lines_start_end[0][1], lines_start_end[1][0], lines_start_end[1][1])

            if intersect_point is not None:
                m.Shape(m.Sphere(radius=0.005), static=True,
                        position=intersect_point, collision=False, rgba=[1, 0, 0, 1])
                # ------ TODO Student answer below -------
                # draw moving centrode
                # get intersection point in local frame w.r.t. link 4
                local_intersect_point = np.array([0, 0, 0])
                # ------ Student answer above -------

                intersect_points_local.append(local_intersect_point)
                # get global coordinates of intersection point
                intersect_point_local_body = m.Shape(m.Sphere(radius=0.005), static=True,
                                                     position=intersect_point, collision=False, rgba=[0, 1, 0, 1])
                intersect_points_local_bodies.append(
                    intersect_point_local_body)

        # redraw intersection points of moving centrode
        # ------ TODO Student answer below -------
        # Hint: You can use Body.set_base_pos_orient(xyz) to update a body's position
        for body, point_local in zip(intersect_points_local_bodies, intersect_points_local):
            body.set_base_pos_orient([0, 0, 0])
        # ------ Student answer above -------

    m.step_simulation(realtime=True)

    if i == 500 or i == 600 or i == 700:
        print('Please save screenshot and include in writeup')
        input("Press Enter to continue...")
