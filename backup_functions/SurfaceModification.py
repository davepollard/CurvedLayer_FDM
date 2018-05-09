"""
David Pollard

FARSCOPE CDT
Bristol Robotics Laboratory

Released under the MIT License


Interprets an STL file
"""
import numpy as np
from stl import mesh
from mpl_toolkits import mplot3d
from matplotlib import pyplot, path

class SurfaceMod():
    '''
    Class interprets STL file for extraction of the top surface

    To test with this class, use mesh_visualisation_test.py
    '''
    def __init__(self, filename, min_z_val=0.1, req_rotation=None):
        '''
        Initialise class

        Inputs:
         - filename         path to STL file
         - min_z_val        will only use facets above this z height
         - req_rotation     XYZ rotation for mesh. Applied before min_z_val truncation
        '''
        self._raw_data = mesh.Mesh.from_file(filename)
        data = np.zeros(len(self._raw_data), dtype=mesh.Mesh.dtype)
        idx = 0

        if np.sum(req_rotation) is not None:
            self.rotateMesh(req_rotation)

        self._max_z = -9999999999

        for val in self._raw_data.points:
            np_val = np.array(val)
            if all(np_val[[2, 5, 8]] > min_z_val): # select z values
                data['vectors'][idx] = np_val.reshape(3, 3)
                idx += 1

                # update max if required
                if max(np_val[[2, 5, 8]]) > self._max_z:
                    self._max_z = max(np_val[[2, 5, 8]])


        self._mesh_data = mesh.Mesh(data)



    def getPointParams(self, xy_coord):
        '''
        returns XYZ coordinate on surface (x,y same as input) and unit normal
        '''
        if len(xy_coord) == 3:
            xy_coord = xy_coord[0:2]


        # Loop over facets to find which contain the XY location
        closest_p_idx = -1

        for j, p in enumerate(self._mesh_data):
            v_0 = p[0:3]
            v_1 = p[3:6]
            v_2 = p[6:9]

            facet_path = path.Path([v_0[0:2], v_1[0:2], v_2[0:2]])

            if facet_path.contains_point(xy_coord):
                closest_p_idx = j
                break

        # Error in case point not found
        if closest_p_idx == -1:
            raise Exception("Point [%r, %r] not found"%(xy_coord[0], xy_coord[1]))

        # Calculate plane equation
        # -> ax + by + cz = d
        normal_vector = self._mesh_data.normals[closest_p_idx]
        n_0, n_1, n_2 = normal_vector
        d = np.dot(normal_vector, v_0)
        z = (d - n_0*xy_coord[0] - n_1*xy_coord[1])/n_2
        xyz = [xy_coord[0], xy_coord[1], z]

        if np.linalg.norm(normal_vector) > 0.00001:
            unit_normal = normal_vector / np.linalg.norm(normal_vector)
        else:
            unit_normal = np.array([0, 0, 1])

        return [xyz, unit_normal]


    def rotateMesh(self, req_rotation):
        '''
        Applies a rotation to mesh in order X-Y-Z
         - NB: provide angle in degrees
        '''
        if req_rotation[0] != 0:
            self._raw_data.rotate([1, 0, 0], np.radians(req_rotation[0]))
        if req_rotation[1] != 0:
            self._raw_data.rotate([0, 0.5, 0], np.radians(req_rotation[1]))
        if req_rotation[2] != 0:
            self._raw_data.rotate([0, 0, 0.5], np.radians(req_rotation[2]))


    def offsetMesh(self, xyz=None):
        '''
        Applies constant XYZ offset to mesh
        '''
        if xyz is None:
            return
        self._mesh_data.x += xyz[0]
        self._mesh_data.y += xyz[1]
        self._mesh_data.z += xyz[2]


    def centreMesh(self):
        '''
        Centres mesh in XY plane about (0, 0)
        '''
        x_centre_amount = (np.max(self._mesh_data.x)- np.min(self._mesh_data.x))/2
        y_centre_amount = (np.max(self._mesh_data.y)-np.min(self._mesh_data.y))/2
        self._mesh_data.x += -1*np.max(self._mesh_data.x) + x_centre_amount
        self._mesh_data.y += -1*np.max(self._mesh_data.y) + y_centre_amount


    def getMeshVectorData(self):
        '''
        Returns vectors of surface normals
        '''
        return self._mesh_data.vectors


    #
    #  Point ID functions
    #
    def getDistToMesh(self, xyz):
        '''
        Returns surface height for given coordinate
        '''
        if len(xyz) == 3:
            xy = xyz[0:2]
        else:
            raise Exception("Expected three digit coordinate")

        # Loop over facets to find which contain the XY location
        closest_p_idx = -1
        for j, p in enumerate(self._mesh_data):
            v_0 = p[0:3]
            v_1 = p[3:6]
            v_2 = p[6:9]

            facet_path = path.Path([v_0[0:2], v_1[0:2], v_2[0:2]])

            if facet_path.contains_point(xy):
                closest_p_idx = j
                break

        # Error in case point not found
        if closest_p_idx == -1:
            #return None
            raise Exception("Point [%r, %r] not found" % (xy[0], xy[1]))

        # Calculate plane equation
        # -> ax + by + cz = d
        normal_vector = self._mesh_data.normals[closest_p_idx]
        n_x, n_y, n_z = normal_vector
        n_ang = np.dot(normal_vector, v_0)
        z_val = (n_ang - n_x*xy[0] - n_y*xy[1])/n_z

        return z_val





    #
    #  Plotting functions
    #
    def generateAxes(self):
        '''
        Produces axes for plotting surfaces and paths
        '''
        self.fig = pyplot.figure()
        axes = mplot3d.Axes3D(self.fig)
        return axes

    def addPath(self, axes, xyz, plot_options = 'r'):
        '''
        Adds a given path to the axes provided
        '''
        x, y, z = zip(*xyz)
        axes.plot(x, y, z, plot_options)


    def addAnotherMeshToPlot(self, axes, newMesh):
        '''
        Adds a mesh to the provided axes, and scales based on original mesh
        '''
        axes.add_collection3d(mplot3d.art3d.Poly3DCollection(newMesh.getMeshVectorData()))
        scale = self._mesh_data.points.flatten(-1)
        axes.auto_scale_xyz(scale, scale, scale)


    def showPlot(self):
        '''
        Shows current axes generated by generateAxes
        '''
        pyplot.draw()
        self.fig.show()




if __name__ == "__main__":
    print "Testing..."
    surface = SurfaceMod("hemisphere.stl")
    [a, b] = surface.getPointParams([23, 10])
    print a
    print b
