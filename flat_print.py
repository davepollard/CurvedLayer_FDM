"""
David Pollard

FARSCOPE CDT
Bristol Robotics Laboratory

Released under the MIT License


Prints on a flat surface
"""
from __future__ import division
import sys

sys.path.append("..")

from Robot_Config import robot_config
from curved_stl_print import CreatePath
from movementOperations import moveNormC

DEFAULT_MODE = 0

class PrintParameters(object):
    """ store data needed """

    def __init__(self):
        '''
        Path selection
         0    squares
         1    hexagons
         2    uob logo
        '''
        path_type = 2
        path_min_max = [(-30, 30), (-20, 20)]

        centre_xyz = [50, 70, 90]

        self.num_layers = 10
        self.layer_height = 0.5

        self.default_q_val = [0, 0, 1, 0]

        # Robot configuration
        self.travel_speed = [500, 1500, 1500, 1500]
        self.extrude_speed = [20, 500, 500, 500]

        self.work_object_coord = robot_config.get_wobj(0)
        self.print_tool_coord = robot_config.get_tool(0)
        self.init_joint_coord = robot_config.get_initJoint(0)

        self.path = CreatePath(path_type, path_min_max)

        for i, path in enumerate(self.path):
            self.path[i] = [[x+centre_xyz[0], y+centre_xyz[1], z+centre_xyz[2]]\
                                for x, y, z in path]




def main(mode_selection):
    """ print process """

    params = PrintParameters()

    # Robot initialisation
    if mode_selection == 1:
        R = abb.Robot("127.0.0.1")
    else:
        R = abb.Robot("192.168.125.1")

    R.set_speed(params.travel_speed)
    R.set_workobject(params.work_object_coord)
    R.set_tool(params.print_tool_coord)

    R.reset_position(1)
    R.set_speed(params.travel_speed)
    R.set_joints(params.init_joint_coord)
    R.set_speed(params.extrude_speed)

    current_pose = [[0, 0, 100], params.default_q_val]
    for layer in range(0, params.num_layers):
        print "Layer %d of %d" % (layer, params.num_layers)

        # Generate coordinates
        for path in params.path:

            pose_list = [[[x, y, z + layer*params.layer_height], params.default_q_val]\
                            for x, y, z in path]

            inter_pose = moveNormC(current_pose, pose_list[0], [0, 0, 1])
            if inter_pose and current_pose[0][2] is not 100:
                R.set_speed(params.travel_speed)
                R.buffer_set([inter_pose, pose_list[0]])
                R.buffer_execute_circ()
                R.set_speed(params.extrude_speed)

            R.buffer_set(pose_list)
            R.buffer_execute(1)

            current_pose = pose_list[-1]

    if mode_selection == 0:
        R.show_motions()
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