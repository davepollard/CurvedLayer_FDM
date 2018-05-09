"""
David Pollard

FARSCOPE CDT
Bristol Robotics Laboratory

Released under the MIT License


 Testing use of mesh interpretation and find the best rotation

    To install stl interpreter:
      Navigate to C:/Python27/Scripts
      >> pip install numpy-stl

 """
from SurfaceModification import SurfaceMod

surfaceClass = SurfaceMod("../data_files/printbed_hemisphere.stl", 3, [-90, 0, 0])
surfaceClass.centreMesh()
axes = surfaceClass.generateAxes()
surfaceClass.addAnotherMeshToPlot(axes, surfaceClass)
surfaceClass.showPlot()

raw_input()
