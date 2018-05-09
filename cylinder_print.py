"""
David Pollard

FARSCOPE CDT
Bristol Robotics Laboratory

Released under the MIT License


Prints on a rotating cylinder

"""

import sys
import os
import numpy as np
sys.path.append(os.path.join(os.path.dirname(__file__), 'backup_functions'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'robot_code'))

import warnings
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import xyz_generator_functions as xyzGenerator
warnings.filterwarnings("ignore", ".*GUI is implemented.*")

#import abb_testing as abb
import abb
from Robot_Config import robot_config


# Settings
# - toolpath selection defined in backup_functions/xyz_generator_functions.py
# - - 1:   sine wave
# - - 6:   UoB logo
XYZ_SELECTION = 6
ANIMATE_PRINT = False
MAX_FIG_AXIS = 75
NUM_CYLINDER_POINTS = 60
CYLINDER_XY = np.array([31.4, 37])
CYLINDER_R = 34
CYLINDER_H = 100

FRAME_DELAY = 0.05

DEFAULT_MODE = 1

LAYER_OFFSETS = [layer_num/2 for layer_num in range(10)]


def update_plot(cylinder_xyz=None, tip_xyz=None):
    '''
    Initialises plot and adds on path. Deletes previous lines to plot new ones.

    Inputs:
     - cylinder_xyz     list of coordinate values of cylinder surface
     - tip_xyz          coordinate of tip at current point
    '''
    fig_x = [-1*MAX_FIG_AXIS, MAX_FIG_AXIS, MAX_FIG_AXIS, -1*MAX_FIG_AXIS]
    fig_y = [-1*MAX_FIG_AXIS, -1*MAX_FIG_AXIS, MAX_FIG_AXIS, MAX_FIG_AXIS]
    fig_z = [0, 0, 0, 0]

    # Create plot
    try:
        if np.isnan(update_plot.fig):
            # Figure doesn't exist - initialise
            update_plot.fig = plt.figure()
            update_plot.ax = update_plot.fig.add_subplot(111, projection='3d')
            update_plot.ax.plot(fig_x, fig_y, fig_z, 'b.')
            update_plot.ax.plot([0, 5], [0, 0], [0, 0], 'r')
            update_plot.ax.plot([0, 0], [0, 5], [0, 0], 'b')
            update_plot.ax.plot([0, 0], [0, 0], [0, 5], 'b')
            update_plot.ax.set_xlabel('X')
            update_plot.ax.set_ylabel('Y')
            update_plot.ax.set_zlabel('Z')
            update_plot.ax.view_init(45, -20)

    except TypeError:
        # Figure exists, delete cylinder lines
        while len(update_plot.ax.lines) > 4:
            update_plot.ax.lines[-1].remove()


    # Exit early if no input arguement
    if cylinder_xyz is None:
        return

    # Extract cylinder values
    cylinder_x = cylinder_xyz[0]
    cylinder_y = cylinder_xyz[1]
    cylinder_z = cylinder_xyz[2]

    update_plot.ax.plot(cylinder_x, cylinder_y, cylinder_z, 'b')
    update_plot.ax.plot(cylinder_x[0:2], cylinder_y[0:2], cylinder_z[0:2], 'g')

    if tip_xyz is None:
        return

    tip_x = [tip_xyz[0], -1*MAX_FIG_AXIS]
    tip_y = [tip_xyz[1], tip_xyz[1]]
    tip_z = [tip_xyz[2], tip_xyz[2]]

    update_plot.ax.plot(tip_x, tip_y, tip_z, 'k-')


def update_plot_add_trace(x, y, z):
    '''
    Adds a line to plot

    Inputs:
     - x, y, z  lists of relevant coordinates
    '''
    update_plot.ax.plot(x, y, z, 'r')

def update_plot_show():
    '''
    Shows current plot
    '''
    update_plot.fig.show()
    plt.pause(FRAME_DELAY)


update_plot.fig = np.nan
update_plot.ax = np.nan




def get_cylinder_coordinates(curr_cylinder_theta=0):
    '''
    Generate x, y, z coordinates of points around cylinder for plotting
    '''
    # Create cylinder XYZ coordinates [untransformed]
    theta = np.linspace(0, 2*np.pi, NUM_CYLINDER_POINTS) * 0.99
    cylinder_x = CYLINDER_R * np.cos(theta)
    cylinder_y = CYLINDER_R * np.sin(theta)
    cylinder_z = np.array([[0, CYLINDER_H] for _ in range(NUM_CYLINDER_POINTS)])

    # Process
    cylinder_x = cylinder_x.repeat(2) + CYLINDER_XY[0]
    cylinder_y = cylinder_y.repeat(2) + CYLINDER_XY[1]
    cylinder_z = cylinder_z.flatten()

    # Apply rotation
    cylinder_x, cylinder_y = rotate_vector(cylinder_x, cylinder_y, curr_cylinder_theta)

    return [cylinder_x, cylinder_y, cylinder_z]


def rotate_vector(x_vector, y_vector, ang):
    '''
    Apply a 2D transformation to a vector
    '''
    # Double check they are numpy arrays...
    x_vector = np.array(x_vector)
    y_vector = np.array(y_vector)
    new_x = x_vector*np.cos(ang) + -1*y_vector*np.sin(ang)
    new_y = x_vector*np.sin(ang) + y_vector*np.cos(ang)
    return new_x, new_y






def main(mode_selection):
    '''
    Main function to generate toolpath
    '''
    prev_path_x = []
    prev_path_y = []
    prev_path_z = []
    tip_xyz = []
    base_theta = []
    curr_layer = []

    x_path = []
    y_path = []
    z_path = []
    theta_path = []

    # Generate tool path
    initial_x, initial_y, _ = xyzGenerator.retrieve_xyz(XYZ_SELECTION, CYLINDER_R, CYLINDER_H)
    initial_x = np.array(initial_x)
    initial_y = np.array(initial_y)

    for init_x_val, init_y_val in zip(initial_x, initial_y):
        x = np.array(init_x_val)
        theta_path.append(x / CYLINDER_R)
        z_path.append(init_y_val)
        temp_x, temp_y = rotate_vector(CYLINDER_XY[0], CYLINDER_XY[1], theta_path[-1])
        x_path.append(temp_x)
        y_path.append(temp_y)

    x_path = np.array(x_path)
    y_path = np.array(y_path)
    z_path = np.array(z_path)


    for layer_offset in LAYER_OFFSETS:
        base_radius = CYLINDER_R + layer_offset
        for i, _ in enumerate(x_path):
            tip_xyz.append([[x-base_radius, y, z] \
                for x, y, z in zip(x_path[i], y_path[i], z_path[i])])

            # Generate trace path
            # - Untransformed path around cylinder at zero rotation
            prev_x, prev_y = rotate_vector(-1*base_radius, 0, -1*theta_path[i])
            prev_x += CYLINDER_XY[0]
            prev_y += CYLINDER_XY[1]

            prev_path_x.append(prev_x)
            prev_path_y.append(prev_y)
            prev_path_z.append(z_path[i])

            base_theta.append(theta_path[i])
            curr_layer.append(layer_offset)


    # Display
    if mode_selection == 0 and ANIMATE_PRINT:
        for i in range(len(prev_path_x)):
            for j, r in enumerate(base_theta[i]):
                if j%3:
                    continue

                xyz = tip_xyz[i][j]

                cylinder_xyz = get_cylinder_coordinates(r)
                update_plot(cylinder_xyz, xyz)

                # Add previous path traces...
                for k in range(0, i+1):
                    x, y = rotate_vector(prev_path_x[k], prev_path_y[k], r)
                    if k == i:
                        update_plot_add_trace(x[0:j], y[0:j], prev_path_z[k][0:j])
                    else:
                        update_plot_add_trace(x, y, prev_path_z[k])

                update_plot_show()
            # end section loop
        # End display loop

    elif mode_selection == 0 and not ANIMATE_PRINT:
        # Uses final values of above to show print
        r = base_theta[-1][-1]
        xyz = tip_xyz[-1][-1]

        cylinder_xyz = get_cylinder_coordinates(r)
        update_plot(cylinder_xyz, xyz)
        for k, _ in enumerate(prev_path_x):
            x, y = rotate_vector(prev_path_x[k], prev_path_y[k], r)
            update_plot_add_trace(x, y, prev_path_z[k])
        update_plot_show()
        raw_input()

    elif mode_selection == 1 or mode_selection == 2:
        print "----------- Warning: Use RESET_POSITION 2 to reset -------------------"
        if mode_selection == 1:
            R = abb.Robot("127.0.0.1")
        else:
            R = abb.Robot("192.168.125.1")

        R.reset_position(2)
        wobj_coord = robot_config.get_wobj(1)
        tool_coord = robot_config.get_tool(False)
        initial_joint_coord = robot_config.get_initJoint(5)

        R.set_joints(initial_joint_coord)
        R.set_speed([25, 250, 250, 250])
        R.set_workobject(wobj_coord)
        R.set_tool(tool_coord)
        R.set_zone('z1')
        R.set_external_axis([0, 0])

        print_speed = [30, 2000, 2000, 2000]
        fast_print_speed = [50, 2000, 2000, 2000]
        travel_speed = [200, 2000, 2000, 2000]

        q_val = [0.5, 0.5, 0.5, 0.5] # XYZ rotation (0, 90, 90)

        curr_theta = 0
        b_updated_speed = False

        R.set_speed(print_speed)

        for i, _ in enumerate(tip_xyz):
            print "Path %d of %d" % (i, len(tip_xyz))
            if abs(base_theta[i][0]-curr_theta) > 0.5:
                R.set_speed(travel_speed)
                R.set_zone('fine')
                R.set_joints(initial_joint_coord)
                time.sleep(0.5)
                R.set_external_axis([0, np.degrees(base_theta[i][0])])
                R.set_zone('z1')
                R.set_speed(print_speed)

            pose_list = [[XYZ, q_val, np.degrees(theta)] \
                            for XYZ, theta in zip(tip_xyz[i], base_theta[i])]

            R.buffer_set(pose_list, True)
            R.buffer_execute(1)

            curr_theta = base_theta[i][-1]

            if curr_layer[i] >= LAYER_OFFSETS[1] and not b_updated_speed:
                R.set_speed(fast_print_speed)
                b_updated_speed = True


if __name__ == "__main__":
    if len(sys.argv) == 2:
        main(int(sys.argv[1]))
    else:
        main(DEFAULT_MODE)
