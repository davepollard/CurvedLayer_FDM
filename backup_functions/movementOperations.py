"""
David Pollard

FARSCOPE CDT
Bristol Robotics Laboratory

Released under the MIT License


Functions for quaternion operations required in the functions

Contains:
 - multiplyQuaternion
 - genQuaternion
 - constrain
 - moveNormC
"""

from __future__ import division
import numpy as np

def multiplyQuaternion(q_1, q_2):
    '''
    Multiplication of two quaternions
    '''
    w1, x1, y1, z1 = q_1
    w2, x2, y2, z2 = q_2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return [w, x, y, z]


def constrain(n, minV, maxV):
    '''
    Constrains value to within min/max region
    '''
    if n < minV:
        n = minV
    elif n > maxV:
        n = maxV
    return n


def genQuaternion(normal):
    '''
    generates quaternion for nozzle orientation opposed to surface normal
    '''
    normal = np.array(normal)
    normal_x = np.array([normal[0], 0, normal[2]])
    normal_y = np.array([0, normal[1], normal[2]])

    d_x = np.linalg.norm(normal_x)
    d_y = np.linalg.norm(normal_y)

    ang_x = 0
    ang_y = 0

    if d_x > 0.0001:
        ang_x = np.pi/2 * np.dot(np.array([1, 0, 0]), normal_x) / d_x

    if d_y > 0.0001:
        ang_y = np.pi/2 * np.dot(np.array([0, 1, 0]), normal_y) / d_y


    ang_x += np.pi

    q_1 = [np.cos(ang_x/2), 0, np.sin(ang_x/2), 0]
    q_2 = [np.cos(ang_y/2), np.sin(ang_y/2), 0, 0]

    q_val = np.array(multiplyQuaternion(q_1, q_2))
    q_val = q_val / np.linalg.norm(q_val)

    return q_val.tolist()


def moveNormC(curr_pose, new_pose, xyz_norm):
    '''
    Returns midpont between two poses based on surface normal
    '''
    x_dist = (new_pose[0][0] - curr_pose[0][0])/2.0
    y_dist = (new_pose[0][1] - curr_pose[0][1])/2.0
    z_dist = (new_pose[0][2] - curr_pose[0][2])/2.0

    mid_point = np.array([curr_pose[0][0]+x_dist, curr_pose[0][1]+y_dist, curr_pose[0][2]+z_dist])
    total_dist = np.linalg.norm(np.array([x_dist, y_dist, z_dist]))

    if total_dist < 1:
        return False

    mid_point += total_dist * np.array(xyz_norm)
    return [mid_point.tolist(), new_pose[1]]
    