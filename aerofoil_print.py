"""
David Pollard

FARSCOPE CDT
Bristol Robotics Laboratory

Released under the MIT License


Generation of an aerofoil-shaped core
"""
import sys
import os
import numpy as np
sys.path.append(os.path.join(os.path.dirname(__file__), 'backup_functions'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'robot_code'))

from honeycomb_pattern import gen_PolyHex
from SurfaceModification import SurfaceMod
from PrintPathClass import PrintPath
from Robot_Config import robot_config
from movementOperations import genQuaternion, moveNormC

DEFAULT_MODE = 0

#
# Print setup
#
class PrintComponentValues():
    """
    Class to specifiy print parameters for function
    """
    def __init__(self):
        # location of marked corner of substrate
        # - NB: Mesh is centred, offset applied to base path coordinates
        xyz_offset = [2, 77, -1]

        # Select aerofoil shape
        # - 0   straight
        # - 1   tapered
        self.aerofoil_shape = 0

        self.extrude_speed = [30, 1500, 1500, 1500]
        self.travel_speed = [150, 1500, 1500, 1500]

        self.stl_rotation = [-90, 0, 0]

        self.layer_height = 0.5

        # process inputs...
        # - nb: Boundary should be defined with bottom left corner first
        if self.aerofoil_shape == 0:
            stl_filename = "data_files/Aerofoil_Shape.stl"
            centre_z_height = 45
            print_path_boundary = [[-30, -45], [30, -45], [30, 54], [-30, 54]]
        elif self.aerofoil_shape == 1:
            stl_filename = "data_files/taper_aerofoil.stl"
            centre_z_height = "data_files/taper_zPlane.stl"
            print_path_boundary = [[-50, -17], [50, -42], [50, 51], [-50, 51]]



        # Create base path and surface
        toolpath = self.create_2d_tool_path(print_path_boundary)
        self.surface, self.surface_axes = self.create_stl_surface(stl_filename)

        if isinstance(centre_z_height, int):
            self.base_print_path, self.max_path_range = \
                self.create_conforming_path(toolpath, centre_z_height)
        else:
            self.top_surface, _ = self.create_stl_surface(centre_z_height)
            self.base_print_path, self.max_path_range = \
            self.create_conforming_path(toolpath, self.top_surface, False)

        # numLayers
        self.layers_per_half = int(np.ceil(self.max_path_range / self.layer_height))
        self.total_num_layers = 2*self.layers_per_half

        #
        #  Apply offset
        #
        self.final_applied_offset = [xyz_offset[0]-print_path_boundary[0][0], \
                                        xyz_offset[1]-print_path_boundary[0][1], xyz_offset[2]]
        self.surface.offsetMesh(self.final_applied_offset)

        for path in self.base_print_path:
            path.offsetPath(self.final_applied_offset)


        #
        #  Plot
        #
        '''
        self.surface.addAnotherMeshToPlot(self.surface_axes, self.surface)

        #for p in toolpath:
        #    self.surface.addPath(self.surface_axes, p, 'b')


        #for p in self.base_print_path:
        #    self.surface.addPath(self.surface_axes, p.xyz)


        for layer in range(0, self.total_num_layers, 5):
            newLayer = self.get_print_layer(layer)
            for p in newLayer:
                self.surface.addPath(self.surface_axes, p.xyz)

        self.surface.showPlot()
        raw_input()
        '''



    def create_2d_tool_path(self, print_path_boundary):
        '''
        Generates hexagonal pattern
        '''
        x_boundary, y_boundary = zip(*print_path_boundary)
        hexagon_2d_coord = gen_PolyHex(5, [x_boundary, y_boundary])
        x_hexagon = hexagon_2d_coord[0]
        y_hexagon = hexagon_2d_coord[1]
        z_val = 0
        xyz = []
        for x_path, y_path in zip(x_hexagon, y_hexagon):
            xyz.append([[x, y, z_val] for x, y in zip(x_path, y_path)])
        return xyz

    def create_stl_surface(self, stl_filename):
        '''
        Returns surface object and its axes
        '''
        surface = SurfaceMod(stl_filename, 10, self.stl_rotation)
        surface.centreMesh()
        surface_axes = surface.generateAxes()
        return surface, surface_axes

    def create_conforming_path(self, toolpath, centre_z_height, flat_upper_surface=True):
        '''
        Returns conforming toolpath to STL file
        '''
        confoming_path = []
        max_path_z_range = 0
        for path in toolpath:
            # Calculate required parts
            new_path = PrintPath([], [], [])
            for p in path:
                xyz, normal = self.surface.getPointParams(p[0:2])

                # Calculate distance to mesh
                if flat_upper_surface:
                    z = centre_z_height
                else:
                    z = centre_z_height.getDistToMesh(xyz)

                #Add point
                new_path.addPathPoint(xyz, normal, z)

            confoming_path.append(new_path)
            if new_path.pathRange() > max_path_z_range:
                max_path_z_range = new_path.pathRange()

        return confoming_path, max_path_z_range


    def get_print_layer(self, layer_num):
        '''
        Returns a list of PrintPath objects for the provided layer number
         - if layer num is above halfway, it reverses the equivalent layer below
        '''
        confoming_path = None

        prev_path_end_xyz = [0, 0, 0]

        # below
        if layer_num <= self.layers_per_half:
            confoming_path = []
            offset = layer_num * self.layer_height
            for path in self.base_print_path:
                new_path = path.returnBelowCentre(offset)
                if len(new_path) > 1:
                    # select direction
                    if self.require_next_path_reversal(prev_path_end_xyz, \
                            new_path.xyz[0], new_path.xyz[-1]):
                        new_path.reversePath()

                    # Append
                    confoming_path.append(new_path)
                    prev_path_end_xyz = new_path.xyz[-1]

        # Above halfway, reflect layer
        elif layer_num <= self.total_num_layers:
            confoming_path = self.get_print_layer(self.total_num_layers - layer_num)
            for path in confoming_path:
                path.reflectPath()
        return confoming_path


    def require_next_path_reversal(self, current_point, next_path_start, next_path_end):
        '''
        Returns boolean for if the next path needs to be reversed based on input points
        '''
        current_point = np.array(current_point)
        next_path_start = np.array(next_path_start)
        next_path_end = np.array(next_path_end)

        dist_to_start = np.linalg.norm(current_point-next_path_start)
        dist_to_end = np.linalg.norm(current_point-next_path_end)

        if dist_to_start < dist_to_end:
            return False
        else:
            return True





def main(mode_selection):
    '''
    Main function.
    Creates class containing print paramters and repeatedly calls to retrieve each layer
    '''
    planner = PrintComponentValues()
    print "--- initialisation complete ---"

    if mode_selection == 1:
        R = abb.Robot("127.0.0.1")
    else:
        R = abb.Robot("192.168.125.1")

    R.set_workobject(robot_config.get_wobj())
    R.set_tool(robot_config.get_tool())
    R.set_speed(planner.travel_speed)

    R.reset_position(1)                         # NB: SETTING OFF SIGNAL
    R.set_joints(robot_config.get_initJoint())


    current_pose = [[100, 100, 100], [0, 0, 1, 0]]

    R.set_speed(planner.extrude_speed)


    # Loop over layers
    for layer in range(1, planner.total_num_layers):
        print "Layer %d of %d" % (layer, planner.total_num_layers)

        layer_paths = planner.get_print_layer(layer)
        print " - num paths: %d" % len(layer_paths)

        for path in layer_paths:
            q_val = [genQuaternion(n) for n in path.norm]

            # travel move
            new_start_pose = [[path.xyz[0][0], path.xyz[0][1], path.xyz[0][2]+5], q_val[0]]
            inter_pose = moveNormC(current_pose, new_start_pose, path.norm[0])
            if inter_pose and current_pose[0][2] is not 100:
                R.set_speed(planner.travel_speed)
                R.buffer_set([inter_pose, new_start_pose])
                R.buffer_execute_circ()
                R.set_speed(planner.extrude_speed)


            R.buffer_set([[xyz, q] for xyz, q in zip(path.xyz, q_val)])
            R.buffer_execute(1)

            current_pose = [path.xyz[-1], q_val[-1]]


    R.reset_position(0)

    if mode_selection == 0:
        R.show_motions(False)
        raw_input()




if __name__ == "__main__":
    # Parse command line arguements
    cmd_args = sys.argv
    if len(cmd_args) == 2:
        cmd_mode = int(cmd_args[1])
    else:
        cmd_mode = DEFAULT_MODE

    # select testing (0), simulation (1), or robot (2)
    if cmd_mode == 0:
        print "Testing mode"
        import abb_testing as abb
    elif cmd_mode == 1 or cmd_mode == 2:
        print "Sim or Running mode"
        import abb
    else:
        raise Exception("Unknown input passed in")

    main(cmd_mode)
