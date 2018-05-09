"""
David Pollard

FARSCOPE CDT
Bristol Robotics Laboratory

Released under the MIT License


Generates a toolpath to print on a given surface
"""

from __future__ import division
import sys
import os
import numpy as np
sys.path.append(os.path.join(os.path.dirname(__file__), 'backup_functions'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'robot_code'))

from Robot_Config import robot_config
from SurfaceModification import SurfaceMod

from honeycomb_pattern import gen_RectHex

from movementOperations import genQuaternion, moveNormC

DEFAULT_MODE = 0

# SAVE / LOAD NEW PATH
LOAD_PATH = False
SAVE_PATH = True
XYZ_FILENAME = "saved_xyz.npy"
NORM_FILENAME = "saved_norm.npy"

if LOAD_PATH and SAVE_PATH:
    raise Exception("cannot have load and save enabled")


#
#  Parameters
#
class PrintComponentValues(object):
    """
    Class defines parameters used for print
    """
    def __init__(self):

        # Coordinates of reference point of STL file
        self.centre_xyz = [0, 0, 0]
        self.centre_xyz = np.array(self.centre_xyz)
        '''
        Test STL selection:
         0    hemisphere
         1    straight aerofoil
        '''
        test_stl = 1

        '''
        Path selection
         0    squares
         1    hexagons
         2    uob logo
        '''
        path_type = 1

        self.num_layers = 10
        self.layer_height = 0.5

        self.default_q_val = [0, 0, 1, 0]

        # Robot configuration
        self.travel_speed = [500, 1500, 1500, 1500]
        self.extrude_speed = [20, 500, 500, 500]

        self.work_object_coord = robot_config.get_wobj(0)
        self.print_tool_coord = robot_config.get_tool(0)
        self.init_joint_coord = robot_config.get_initJoint(0)

        # STL selection
        if test_stl == 0:
            self.surface_filename = "data_files/printbed_hemisphere.stl"
            self.stl_rotation = [-90, 0, 0]
            self.path_min_max = [(-30, 10), (-20, 20)]
        elif test_stl == 1:
            self.surface_filename = "data_files/Aerofoil_Shape.stl"
            self.stl_rotation = [-90, 0, 0]
            self.path_min_max = [(-30, 30), (-50, 50)]

        self.path = CreatePath(path_type, self.path_min_max)



def CreatePath(path_type, path_min_max):
    '''
    Generate path based on provided min/max values

    Input:
        - path_type        Squares: 0, hexagons: 1, UoB logo: 2
        - path_min_max     [(x_min, x_max), (y_min, y_max)]

    Returns:
        - xyz              list of list of coordinates
    '''
    if path_type == 0:
        template_square = [[-4, -4, 0], [4, -4, 0], [4, 4, 0], [-4, 4, 0], [-4, -4, 0]]

        x_vals = np.linspace(path_min_max[0][0], path_min_max[0][1], \
            np.floor((path_min_max[0][1]-path_min_max[0][0])/10))

        y_vals = np.linspace(path_min_max[1][0], path_min_max[1][1],  \
            np.floor((path_min_max[1][1]-path_min_max[1][0])/10))

        xyz = []

        for x in x_vals:
            for y in y_vals:
                xyz.append([[xs+x, ys+y, zs] for (xs, ys, zs) in template_square])

    elif path_type == 1:
        xy_vals = gen_RectHex(3, path_min_max[0], path_min_max[1])
        x_vals = xy_vals[0]
        y_vals = xy_vals[1]
        z_val = 0
        xyz = []
        for x_path, y_path in zip(x_vals, y_vals):
            xyz.append([[x, y, z_val] for x, y in zip(x_path, y_path)])


    elif path_type == 2:
        logo_path = np.load('data_files/uob_logo_path.npy')
        scale_value = 15
        y_offset = -12
        xyz = [[[x*scale_value, y*scale_value + y_offset, 0] \
                    for x, y in path[0::4]] for path in logo_path]

        rotation_val = -1 * np.pi/2
        xyz = [[[x*np.cos(rotation_val)-y*np.sin(rotation_val), \
                    x*np.sin(rotation_val)+y*np.cos(rotation_val), z] \
                        for x, y, z in path] for path in xyz]
        

    else:
        raise Exception("Unrecognised path input")

    return xyz





#
#  Main
#
def main(mode_selection):
    '''
    Runs main loop to generate toolpath and run

    Inputs:
        mode_selection      use test class, simulator, or actual robot
    '''
    params = PrintComponentValues()

    # Robot initialisation
    if mode_selection == 1:
        R = abb.Robot("127.0.0.1")
    else:
        R = abb.Robot("192.168.125.1")

    R.set_speed(params.travel_speed)
    R.set_workobject(params.work_object_coord)
    R.set_tool(params.print_tool_coord)

    R.reset_position(1)
    R.set_joints(params.init_joint_coord)


    # Print on flat surface
    '''
    for path in params.path:
        pose_list = [[coord, [0,0,1,0]] for coord in path]
        R.buffer_set(pose_list)
        R.buffer_execute(1)
    '''

    # Surface modification
    surface = SurfaceMod(params.surface_filename, 0.1, params.stl_rotation)
    surface.centreMesh()
    surface_axes = surface.generateAxes()
    surface.addAnotherMeshToPlot(surface_axes, surface)

    if not LOAD_PATH:
        surface_path = []
        surface_normal = []
        for i, path in enumerate(params.path):
            print "Calculating normal for path %d of %d"%(i, len(params.path))
            a = []
            b = []
            for xyz in path:
                [c, d] = surface.getPointParams(xyz)
                a.append(c)
                b.append(d)

            surface_path.append(a)
            surface_normal.append(b)

            surface.addPath(surface_axes, a)

        if SAVE_PATH:
            np.save(XYZ_FILENAME, surface_path)
            np.save(NORM_FILENAME, surface_normal)
    else:
        # loading path
        surface_path = np.load(XYZ_FILENAME)
        surface_normal = np.load(NORM_FILENAME)

        for path in surface_path:
            surface.addPath(surface_axes, path)


    if mode_selection == 0:
        surface.showPlot()

    # Generate quaternion values
    path_q_val = []
    for path_normals in surface_normal:
        coord_q_vals = []
        for n in path_normals:
            q_val = genQuaternion(n)
            coord_q_vals.append(q_val)

        if not coord_q_vals:
            coord_q_vals = [0, 0, 1, 0]
        path_q_val.append(coord_q_vals)


    R.set_speed(params.extrude_speed)

    current_pose = [[0, 0, 100], params.default_q_val]
    for z in range(0, params.num_layers):
        print "Layer %d of %d" % (z, params.num_layers)
        for path, norm, coord_q_vals in zip(surface_path, surface_normal, path_q_val):
            # Three lists containing xyz coordinates, normal coordinates, and required quaternion
            #
            #
            offset_path = [p+z*n*params.layer_height+params.centre_xyz for p, n in zip(path, norm)]
            surface.addPath(surface_axes, offset_path)
            pose_list = [[p, q] for p, q in zip(offset_path, coord_q_vals)]
            inter_pose = moveNormC(current_pose, pose_list[0], norm[0])
            if inter_pose and current_pose[0][2] is not 100:
                R.set_speed(params.travel_speed)
                R.buffer_set([inter_pose, pose_list[0]])
                R.buffer_execute_circ()


                R.set_speed(params.extrude_speed)
            R.buffer_set(pose_list)
            R.buffer_execute(1)
            current_pose = pose_list[-1]

    R.reset_position(0)

    surface.showPlot()

    raw_input()
    if mode_selection == 0:
        R.show_motions(False)
        raw_input()






#
#  Call function
#
if __name__ == "__main__":
    # Parse command line arguements
    cmdArgs = sys.argv
    cmd_input = DEFAULT_MODE
    if len(cmdArgs) == 2:
        cmd_input = int(cmdArgs[1])
    else:
        cmd_input = DEFAULT_MODE

    # select testing (0), simulation (1), or robot (2)
    if cmd_input == 0:
        print "Testing mode"
        import abb_testing as abb
    elif cmd_input == 1 or cmd_input == 2:
        print "Sim or Running mode"
        import abb
    else:
        raise Exception("Unknown input passed in")

    main(cmd_input)
