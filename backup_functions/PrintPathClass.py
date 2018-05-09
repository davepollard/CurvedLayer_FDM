"""
David Pollard

FARSCOPE CDT
Bristol Robotics Laboratory

Released under the MIT License


 Print path class - stores and manipulates class data
"""
from __future__ import division
import numpy as np
from copy import copy, deepcopy


class PrintPath:
    """ Stores and manipulates XYZ path """
    def __init__(self, xyzLst=[], normLst=[], zLst=[]):
        try:
            lenLst = [len(xyzLst), len(normLst), len(zLst)]
            if lenLst.count(lenLst[0]) != 3:
                raise Exception("Unrecognised length")
        except TypeError:
            raise Exception("Must pass list inputs")

        # Check element lengths in list

        self.xyz = xyzLst
        self.norm = normLst
        self.z = zLst


    def __len__(self):
        return len(self.z)


    def __copy__(self):
        return PrintPath(self.xyz, self.norm, self.z)


    def pathRange(self):
        '''
        Returns maximum z change for path
        '''
        maxZ = max([tz - z for [_, _, z], tz in zip(self.xyz, self.z)])
        minZ = min([tz - z for [_, _, z], tz in zip(self.xyz, self.z)])
        return maxZ - minZ


    def reflectPath(self):
        '''
        Reflects current path about z centre line
        '''
        self.xyz = [[x, y, 2*tz-z] for [x, y, z], tz in zip(self.xyz, self.z)]
        self.norm = [[n0, -1*n1, n2] for [n0, n1, n2] in self.norm]


    def reversePath(self):
        '''
        Reverses current path
        '''
        self.xyz = self.xyz[::-1]
        self.norm = self.norm[::-1]
        self.z = self.z[::-1]


    def offsetPath(self, offset=[0, 0, 0]):
        '''
        Applies xyz offset to path
        '''
        dx, dy, dz = offset
        self.xyz = [[x+dx, y+dy, z+dz] for x, y, z in self.xyz]
        self.z = [z + dz for z in self.z]


    def offsetPathAlongNormal(self, offset = None):
        if offset is None:
            return
        dx, dy, dz = offset
        self.xyz = [[x + (nx*dx), y + (ny*dy), z + (nz*dz)] \
            for [x, y, z], [nx, ny, nz] in zip(self.xyz, self.norm)]



    def addPathPoint(self, xyz, norm, z):
        self.xyz.append(xyz)
        self.norm.append(norm)
        self.z.append(z)


    def returnBelowCentre(self, offset):
        '''
        returns an offset path with z value lower than the centre z

        NB: Doesn't support multiple sections above... will just link together
        '''
        offsetXYZ = np.array(deepcopy(self.xyz), dtype='float64')
        offsetXYZ += [0, 0, offset]

        newPath = PrintPath([], [], [])
        onPath = False
        prevXYZ = None
        prevZLim = None

        for p, n, z in zip(offsetXYZ, self.norm, self.z):
            if p[2] <= z:
                # check if needs interpolation (starting)
                if not onPath and prevXYZ is not None:
                    interpPoint = self._findIntersection(prevXYZ, prevZLim, p, z)
                    if interpPoint  is not None:
                        newPath.addPathPoint(interpPoint, n, z)

                # add current point
                newPath.addPathPoint(p.tolist(), n, z)
                onPath = True

            else:
                # ending... check if needs interpolation
                if onPath and prevXYZ is not None:
                    interpPoint = self._findIntersection(prevXYZ, prevZLim, p, z)
                    if interpPoint is not None:
                        newPath.addPathPoint(interpPoint, n, z)
                onPath = False

            prevXYZ = p
            prevZLim = z
        return newPath


    def returnAboveCentre(self, offset):
        '''
        Returns an offset path reflected about centre z
        '''
        offsetXYZ = [[x, y, 2*tz - z - offset] for [x, y, z], tz in zip(self.xyz, self.z)]
        newPath = PrintPath([], [], [])

        for p, n, z in zip(offsetXYZ, self.norm, self.z):
            if p[2] >= z:
                newPath.addPathPoint(p, n, z)

        return newPath


    def _findIntersection(self, p0, z0, p1, z1):
        '''
        Calculate point on line p0_z - p1_z which intersects line z0-z1
        '''
        m_p = (p1[2]-p0[2])
        m_z = (z1-z0)

        # handle parallel lines
        if m_p - m_z == 0:
            return None

        x = (z0 - p0[2]) / (m_p - m_z)

        if not 0.05 <= x <= 0.95:
            # Point is no good
            return None

        interpPoint = [a+x*(b-a) for a, b in zip(p0, p1)]

        return interpPoint






#
#  Testing
#
if __name__ == "__main__":
    path1 = PrintPath([[0, 0, 0], [1, 1, 1]], [[0, 0, 1], [0, 0, 1]], [1, 1])
    path2 = copy(path1)

    print path1.xyz
    print path2.xyz
    path1.offsetPath([5, 5, 5])
    print "-----"
    print path1.xyz
    print path2.xyz

    # test path
    xyz = [[i, 2*i, 7 - np.abs(i-10)] for i in range(20)]
    #z = [11 - z for _, _, z in xyz]
    z = [10 for _ in range(20)]
    norm = [[0, 0, 1] for i in range(20)]

    testPath = PrintPath(xyz, norm, z)
    print "Original path length: %d" % len(testPath)

    print "------------"
    belowPath = testPath.returnBelowCentre(9.25)
    print "Path length: %d" % len(belowPath)
    for p, z in zip(belowPath.xyz, belowPath.z):
        print [p, z]









