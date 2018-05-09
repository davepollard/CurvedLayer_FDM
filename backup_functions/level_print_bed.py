"""
David Pollard

FARSCOPE CDT
Bristol Robotics Laboratory

Released under the MIT License


Robot traces square in selected work object to allow leveling of print bed
"""

from __future__ import division
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'robot_code'))

from Robot_Config import robot_config

DEFAULT_MODE = 0

#
#  Main function
#
def main(mode_selection):
    ''' Trace square '''
    # Setup
    use_test_tool = False
    trace_axes = False
    wobj_selection = 0
    init_z = -1.5
    q_val = [0, 0, 1, 0]
    if trace_axes:
        xy_pos = [[5, 5], [105, 5], [5, 5], [5, 105], [5, 5]]
    elif wobj_selection < 3:
        xy_pos = [[10, 0], [100, 10], [100, 200], [0, 200], [10, 0]]
    else:
        xy_pos = [[60, 0], [100, 0], [100, 125], [60, 125], [60, 0]]

    print "Initialising"
    # Initialise robot
    if mode_selection == 1:
        R = abb.Robot("127.0.0.1")
    else:
        R = abb.Robot("192.168.125.1")

    R.set_workobject(robot_config.get_wobj(wobj_selection))
    R.set_tool(robot_config.get_tool(use_test_tool))

    R.set_joints(robot_config.get_initJoint(wobj_selection))
    R.set_external_axis(robot_config.get_extax(wobj_selection))

    print "Finished initialisation"

    # Configure poses
    poses = [[xy+[init_z], q_val] for xy in xy_pos]

    R.set_speed([50, 500, 500, 500])

    R.buffer_set(poses)
    R.buffer_save(1)

    while True:
        R.buffer_load(1)
        R.buffer_execute(0)








#
#  Call function
#
if __name__ == "__main__":
    # Parse command line arguements
    cmd_args = sys.argv
    mode = DEFAULT_MODE
    if len(cmd_args) == 2:
        mode = int(cmd_args[1])
    else:
        mode = DEFAULT_MODE

    # select testing (0), simulation (1), or robot (2)
    if mode == 0:
        print "Testing mode"
        import abb_testing as abb
    elif mode == 1 or mode == 2:
        print "Sim or Running mode"
        import abb
    else:
        raise Exception("Unknown input passed in")

    main(mode)
