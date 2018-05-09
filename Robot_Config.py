"""
David Pollard

FARSCOPE CDT
Bristol Robotics Laboratory

Released under the MIT License


Robot configuration file
"""
import copy

class robot_config:
    #
    # Base parameters
    #
    Tool = [[-44.51, 9.9, 113.95], [1, 0, 0, 0]]
    Test_Tool = [[-47.7913, 12.961, 103.954], [1, 0, 0, 0]]
    
    # Initialise
    WObjData = []
    ExtAxData = []
    InitJointData = []

    
    # Flat plate
    WObjData.append([[560, -93, 290], [1, 0, 0, 0]])
    ExtAxData.append([0, 0])
    InitJointData.append([0, 17, 8, 0, 65, 0])
    
    # Rotated toolplate
    # - generate x, y coordinates from backup_functions/cylinder_localisation.py
    WObjData.append([[633.5, 9.87, 380], [1, 0, 0, 0]])
    ExtAxData.append([0, 0])
    InitJointData.append([30, 20, 30, 90, -80, 60])
    
    #
    # Return functions
    #
    @classmethod
    def get_tool(robot_config, use_test_tool = False):
        if use_test_tool:
            print("Using nozzle model")
            return robot_config.Test_Tool
        else:
            print("Using actual printhead")
            return robot_config.Tool
            
    @classmethod
    def get_wobj(robot_config, WObjNumber=0):
        if WObjNumber < len(robot_config.WObjData):
            return robot_config.WObjData[WObjNumber]
        else:
            raise Exception("Unrecognised WObj Number")

        
    @classmethod
    def get_extax(robot_config, WObjNumber=0):
        if WObjNumber < len(robot_config.ExtAxData):
            return robot_config.ExtAxData[WObjNumber]
        else:    
            raise Exception("Unrecognised external axis")
            
    @classmethod
    def get_initJoint(robot_config, WObjNumber = 0):
        if WObjNumber <len(robot_config.InitJointData):
            return robot_config.InitJointData[WObjNumber]
        else:
            raise Exception("Unrecognised initial joint position number")