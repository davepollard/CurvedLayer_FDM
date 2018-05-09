# CurvedLayer_FDM
Tools for the generation of curved paths for an arm-based FDM system.

These will be presented at ECCM 2018 with the paper "Application of an Arm-Based FDM System for Sandwich Panel Fabrication", by D. Pollard, G. Herrmann, C. Ward, J. Etches.

A video showing the robot operation is available at:
https://youtu.be/v36t1wzLFEM


Main functions:
 - aerofoil_print.py            Generate aerofoil shaped specimen
 - curved_stl_print.py          Project 2D toolpath onto a curved surface
 - cylinder_print.py            Print onto a cylinder located on rotating print bed
 - flat_print.py                Prints a flat shape

Useful supporting functions:
 - cylinder_localisation.py     Locate the centre of rotation and cylinder in 2D
 - honeycomb_pattern.py         Generate hexagonal print path over surface
 - level_print_bed.py           Move nozzle over selectd bed for manual leveling
 - mesh_visualisation_test.py   View an imported STL file
 - movementOperations.py        Various positioning functions
 - PrintPathClass.py            Provides path processing
 - SurfaceModification.py       Interprets and manipulates STL files
 - xyz_generator_functions.py   Generates a range of 2D toolpaths

Robot control functions:
 - abb.py                       Main robot control class
 - abb_testing.py               Implements most of the functions of abb.py, but plots movements
 - MainModule.mod               RAPID main module
 - SERVER.mod                   RAPID processor for interface

 The software for abb contorl is based on Open-ABB by Michael Dawson-Haggerty, released under the MIT license:
 https://github.com/robotics/open_abb

 The implemented robot control functions are similar to those released by David Pollard, released under the MIT license:
 https://github.com/bristolroboticslab/OpenABB-FDM
