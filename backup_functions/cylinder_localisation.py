"""
David Pollard

FARSCOPE CDT
Bristol Robotics Laboratory

Released under the MIT License


Calculates cylinder coordinate from input coordinates of point on surface

To use:
 - set RUN_ERROR_ANALYSIS to False
 - set up parameters in run_localisation_test, and check instructions there
"""

from __future__ import division
from scipy.optimize import minimize
import numpy as np

# Set to true to check robustness to random errors
RUN_ERROR_ANALYSIS = False

#
#  Optimisation functions
#
def get_estimated_xy(t, rot_x, rot_y, cyl_x, cyl_y, cyl_r):
    '''
    Returns estimated xy coordinate of measured point for given parameters
     - NB: theta (t) must be in radians
    '''
    est_x = rot_x + cyl_x*np.cos(t) - cyl_y*np.sin(t) - cyl_r*np.cos(t)
    est_y = rot_y + cyl_x*np.sin(t) + cyl_y*np.cos(t) - cyl_r*np.sin(t)
    return [est_x, est_y]


def optim_fcn(arg, theta, xy_measurement, cyl_r):
    '''
    arg             4 element list of estimated rotation and cylindrer XY coordinates
    theta           nx1 array of bed angles
    xy_measurement  nx2 array of measured XY coordinates corresponding to theta
    '''

    dist_matrix = np.zeros((len(theta), 2))

    for i, t in enumerate(theta):
        est_x, est_y = get_estimated_xy(t, arg[0], arg[1], arg[2], arg[3], cyl_r)

        dist_matrix[i, 0] = est_x - xy_measurement[i, 0]
        dist_matrix[i, 1] = est_y - xy_measurement[i, 1]

    return np.mean(np.abs(dist_matrix))


def run_optim_fcn(measured_xy, theta, cylinder_r):
    '''
    Wrapper to run the optimisation function
    '''
    optim_result = minimize(optim_fcn, [0, 0, 0, 0],\
                        (theta, measured_xy, cylinder_r),\
                        method='Powell')
    return optim_result


#
#   Useful localisation function
#
def run_localisation_test():
    '''
    Runs test and outputs required coordinates for cylinder location

    To use:
     - Measure the xy coord of the point closest to the robot at rotation angle = 0
     - Measure this point at various other angles and record below
     - Run this function
     - Use result[0:2] for the workobject definition
     - Use result[2:4] for the cylinder_xy coordinate in cylindrer_print.py
    '''

    measured_xy_coord = np.array([[623.9, 49.7], \
                                    [636.3, 50.2], \
                                    [606.2, 41.3], \
                                    [596.2, 29], \
                                    [591.3, 10.9]])
    measured_theta = np.array([0, -17.9, 27.9, 49.8, 77])
    cylinder_r = 39.5
    # result: [632, 9.814, 31.4, 39.9]
    measured_theta = np.radians(measured_theta)

    result = run_optim_fcn(measured_xy_coord, measured_theta, cylinder_r)

    print result


#
#   For estimating expected accuracy of method
#
def run_error_test():
    '''
    Runs through a range of theta values with a range of errors
    Outputs to CSV files

    Currently configured to check if how close theta measurements are affect result
    '''
    # Test setup
    num_runs = 2
    error_multipliers = np.array([0, 0.5, 1, 1.5, 2, 2.5])

    # True values
    rot_x_coord = 630
    rot_y_coord = -10
    cyl_x_coord = 30
    cyl_y_coord = 45
    cyl_radius = 30

    true_result = np.array([rot_x_coord, rot_y_coord, cyl_x_coord, cyl_y_coord])

    # Create theta values
    theta = np.linspace(0, 225, 4)

    measured_xy_coord = np.array([get_estimated_xy(t, rot_x_coord, rot_y_coord, cyl_x_coord, cyl_y_coord, cyl_radius) for t in theta])

    # true optimisation
    result = run_optim_fcn(measured_xy_coord, theta, cyl_radius)


    # Initialise
    mean_error = np.zeros(*error_multipliers.shape)
    section_error = np.zeros((len(error_multipliers), 4))

    overall_mean_error = []
    overall_section_error = []
    for max_theta in range(1, 359):
        print 't: %.1f degrees' % max_theta
        theta = np.linspace(0, max_theta, 4)
        for i, e in enumerate(error_multipliers):
            cumulative_error = 0
            result_error = np.zeros((1, 4))

            for _ in range(num_runs):
                error_xy = measured_xy_coord + np.random.randn(*measured_xy_coord.shape)*e
                result = run_optim_fcn(error_xy, theta, cyl_radius)
                cumulative_error += result.fun
                result_error += np.abs((result.x - true_result))

            mean_error[i] = cumulative_error / num_runs
            section_error[i] = result_error / num_runs

        overall_mean_error.append(mean_error)
        overall_section_error.append(np.mean(section_error, 1))

    np.savetxt("optimError.csv", overall_mean_error, delimiter="\t")
    np.savetxt("trueError.csv", overall_section_error, delimiter="\t")




if __name__ == "__main__":
    if RUN_ERROR_ANALYSIS:
        run_error_test()
    else:
        run_localisation_test()
