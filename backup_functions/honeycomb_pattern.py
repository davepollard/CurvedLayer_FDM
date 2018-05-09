"""
David Pollard

FARSCOPE CDT
Bristol Robotics Laboratory

Released under the MIT License


Script returns a hexagonal toolpath covering a shape
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mpltPath

# Set which part to test if called as main
# - 0    Rectangle
# - 1    Circle
# - 2    Polygon
TEST_FCN_TYPE = 1


def is_continuous(bool_selection_array):
    '''
    Checks if True values in array come in single line or not
    '''
    if bool_selection_array[0] or bool_selection_array[-1]:
        print "Warning - hex array not large enough - adjust range_num"
        return True

    if np.sum(np.diff(bool_selection_array)) == 2:
        return True

    return False


def point_within(p, p0, p1):
    '''
    Identifies if point p is within square defined by p0, p1
    '''
    if min(p0[0], p1[0]) <= p[0] <= max(p0[0], p1[0]):
        if min(p0[1], p1[1]) <= p[1] <= max(p0[1], p1[1]):
            return True
    return False



def get_interpolated_point(x_line, y_line, combined_selection_idx, x_limit, y_limit):
    '''
    Interpolates at start and end of a line based on the bounding box
    '''

    new_point = []
    at_end = []

    # find change point
    change_idx = []
    for i in range(len(combined_selection_idx)-1):
        if combined_selection_idx[i] is not combined_selection_idx[i+1]:
            change_idx.append(i)

    # extract x, y points from either side
    for c in change_idx:
        # set first point as interior one
        if combined_selection_idx[c]:
            xy = [[x_line[c], y_line[c]], [x_line[c+1], y_line[c+1]]]
            b = True
        else:
            xy = [[x_line[c+1], y_line[c+1]], [x_line[c], y_line[c]]]
            b = False

        # Construct line equation
        m = (xy[1][1]-xy[0][1]) / (xy[1][0]-xy[0][0])
        c = xy[0][1]-m*xy[0][0]

        # loop through potential lines
        potential_xy_coord = [0, 0, 0, 0]
        #  - x values
        potential_xy_coord[0] = np.array([x_limit[0], m*x_limit[0]+c])
        potential_xy_coord[1] = np.array([x_limit[1], m*x_limit[1]+c])

        if abs(m) > 0.001:
            potential_xy_coord[2] = np.array([(y_limit[0]-c)/m, y_limit[0]])
            potential_xy_coord[3] = np.array([(y_limit[1]-c)/m, y_limit[1]])

        # Find which intersection is within correct area
        p = None

        for pot_xy in potential_xy_coord:
            if pot_xy is not 0 and point_within(pot_xy, xy[0], xy[1]):
                p = pot_xy

        if p is not None:
            new_point.append(p)
            at_end.append(b)

    return new_point, at_end



def get_line_to_insert(x_line, y_line, x_limit, y_limit):
    '''
    Truncates line provided by coordinates to insert based on upper/lower limits provided
    '''
    trimmed_x_line = None
    trimmed_y_line = None

    append_x_selection = (x_line > x_limit[0]) & (x_line < x_limit[1])
    append_y_selection = (y_line > y_limit[0]) & (y_line < y_limit[1])
    combined_selection_idx = append_x_selection & append_y_selection


    if sum(combined_selection_idx) > 0 and is_continuous(combined_selection_idx):
        # Interpolate point
        trimmed_x_line = x_line[combined_selection_idx]
        trimmed_y_line = y_line[combined_selection_idx]

        # Find point on selection matrix

        new_point, at_end = get_interpolated_point(x_line, y_line, combined_selection_idx, x_limit, y_limit)

        for p, b in zip(new_point, at_end):
            if b:
                trimmed_x_line = np.append(trimmed_x_line, p[0])
                trimmed_y_line = np.append(trimmed_y_line, p[1])
            else:
                trimmed_x_line = np.insert(trimmed_x_line, 0, p[0], axis=0)
                trimmed_y_line = np.insert(trimmed_y_line, 0, p[1], axis=0)


    return trimmed_x_line, trimmed_y_line


def gen_full_hex_grid(hex_r, direction=0):
    '''
    Returns lines from a large hexagonal grid which should cover the print area

    Inputs:
     - hex_r            Hexagon radius
     - direction [=0]   Which direction - 0=vertical lines
                                        - 1=diagonal lines

    Returns:
    [x_coords, y_coords]      List of line x, y coordinates for requested direction
    '''
    # Create way too many points
    range_num = 100
    x_section = np.array([0, 0.5*hex_r])
    y_section = np.array([0, np.sqrt(3)*hex_r/2])

    x_coords = []
    y_coords = []

    if direction == 0:
        # Vertical lines

        row_x_coord = np.array([x_section for _ in range(-range_num, range_num)]).flatten()
        row_y_coord = np.array([y_section+i*2*y_section[1] for i in range(-range_num, range_num)]).flatten()

        for i in range(-range_num, range_num):
            x_coords.append(np.array(row_x_coord)+1.5*hex_r*i)
            y_coords.append(np.array(row_y_coord) + y_section[1]*(i%2))

    elif direction == 1:
        # diagonal section
        row_x_coord = np.array([x_section+i*1.5*hex_r for i in range(-range_num, range_num)]).flatten()
        row_y_coord = np.array([y_section+i*hex_r*np.sqrt(3)/2 for i in range(-range_num, range_num)]).flatten()

        for i in range(-range_num, range_num):
            x_coords.append(row_x_coord)
            y_coords.append(row_y_coord+i*np.sqrt(3)*hex_r)

    else:
        raise Exception("Unrecognised direction input")


    return [x_coords, y_coords]


def gen_RectHex(hex_r, x_limit, y_limit):
    '''
    Returns lines that reside within the rectangle specified by limits

    Inputs:
     - hex_r    Required hexagon radius
     - x_limit     [xMin, xMax]
     - y_limit     [yMin, yMax]
    '''
    # From vertical
    x_grid, y_grid = gen_full_hex_grid(hex_r, 0)

    x_coords = []
    y_coords = []

    # loops over line, calls function to extract line
    for x_line, y_line in zip(x_grid, y_grid):
        new_x_line, new_y_line = get_line_to_insert(x_line, y_line, x_limit, y_limit)
        if new_x_line is not None:
            x_coords.append(new_x_line)
            y_coords.append(new_y_line)


    # from diagonal
    x_grid, y_grid = gen_full_hex_grid(hex_r, 1)

    for x_line, y_line in zip(x_grid, y_grid):
        new_x_line, new_y_line = get_line_to_insert(x_line, y_line, x_limit, y_limit)
        if new_x_line is not None:
            x_coords.append(new_x_line)
            y_coords.append(new_y_line)

    return [x_coords, y_coords]

def gen_CircHex(hex_r, circ_r, circ_xy=[0, 0]):
    '''
    Returns lines that reside within the specified circle

    Inputs:
     - hex_r    Required hexagon radius
     - circ_r   Radius of circle
     - circ_xy  [x,y] coordinates of circle centre
    '''
    # Create circle coordinates
    ang = np.linspace(0, 2*np.pi, 100)
    circ_boundary = [[circ_r*np.sin(x)+circ_xy[0], circ_r*np.cos(x)+circ_xy[1]] for x in ang]

    path = mpltPath.Path(circ_boundary)

    x_coords = []
    y_coords = []
    # From horizontal:
    x_grid, y_grid = gen_full_hex_grid(hex_r, 0)

    for x_line, y_line in zip(x_grid, y_grid):
        points = zip(x_line, y_line)
        inside_selection_idx = path.contains_points(points)

        if sum(inside_selection_idx) > 0 and is_continuous(inside_selection_idx):
            x_coords.append(x_line[inside_selection_idx])
            y_coords.append(y_line[inside_selection_idx])

    # From diagonal:
    x_grid, y_grid = gen_full_hex_grid(hex_r, 1)

    for x_line, y_line in zip(x_grid, y_grid):
        points = zip(x_line, y_line)
        inside_selection_idx = path.contains_points(points)

        if sum(inside_selection_idx) > 0 and is_continuous(inside_selection_idx):
            x_coords.append(x_line[inside_selection_idx])
            y_coords.append(y_line[inside_selection_idx])



    return [x_coords, y_coords]


def gen_PolyHex(hex_r, poly_boundary):
    '''
    Returns lines that reside within the specified polygon

    Inputs:
     - hex_r            Required hexagon radius
     - poly_boundary    list of lists containing X, Y coordinates

                    e.g. [[x1,x2,x3,x1],[y1,y2,y3,y1]] for a triangle
    '''
    path = mpltPath.Path(zip(poly_boundary[0], poly_boundary[1]))
    x_coords = []
    y_coords = []
    # From horizontal:
    x_grid, y_grid = gen_full_hex_grid(hex_r, 0)

    for x_line, y_line in zip(x_grid, y_grid):
        points = zip(x_line, y_line)
        inside_selection_idx = path.contains_points(points)

        if sum(inside_selection_idx) > 0:
            if is_continuous(inside_selection_idx):
                x_coords.append(x_line[inside_selection_idx])
                y_coords.append(y_line[inside_selection_idx])
            else:
                cumulative_sum = np.insert(np.cumsum(np.diff(inside_selection_idx)), 0, 0) # extra zero for length matching
                for i in range(max(cumulative_sum)):
                    if i%2 == 1:
                        new_idx = inside_selection_idx & (cumulative_sum == i)
                        x_coords.append(x_line[new_idx])
                        y_coords.append(y_line[new_idx])

    # From diagonal:
    x_grid, y_grid = gen_full_hex_grid(hex_r, 1)

    for x_line, y_line in zip(x_grid, y_grid):
        points = zip(x_line, y_line)
        inside_selection_idx = path.contains_points(points)

        if sum(inside_selection_idx) > 0:# and is_continuous(inside_selection_idx):
            if is_continuous(inside_selection_idx):
                x_coords.append(x_line[inside_selection_idx])
                y_coords.append(y_line[inside_selection_idx])
            else:
                cumulative_sum = np.insert(np.cumsum(np.diff(inside_selection_idx)), 0, 0) # extra zero for length matching
                for i in range(max(cumulative_sum)):
                    if i%2 == 1:
                        new_idx = inside_selection_idx & (cumulative_sum == i)
                        x_coords.append(x_line[new_idx])
                        y_coords.append(y_line[new_idx])

    return [x_coords, y_coords]



def test_fcn(hex_r, x_limit, y_limit):
    '''
    Function to test generators
     - set which test at top
    '''
    if TEST_FCN_TYPE == 0:
        x_poly = [x_limit[0], x_limit[0], x_limit[1], x_limit[1], x_limit[0]]
        y_poly = [y_limit[0], y_limit[1], y_limit[1], y_limit[0], y_limit[0]]
        x_coords, y_coords = gen_RectHex(hex_r, x_limit, y_limit)
    elif TEST_FCN_TYPE == 1:
        x_poly = 50 * np.sin(np.linspace(0, 2*np.pi)) + 20
        y_poly = 50 * np.cos(np.linspace(0, 2*np.pi)) + 20
        x_coords, y_coords = gen_CircHex(hex_r, 50, [20, 20])
    else:
        x_poly = [100, 0, 0, 50, 50, 20, 20, 50, 70]
        y_poly = [-100, -50, 100, 50, 30, 30, -30, -30, -50]
        x_coords, y_coords = gen_PolyHex(hex_r, [x_poly, y_poly])




    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(x_poly, y_poly, 'k--')

    for i, _ in enumerate(x_coords):
        x_path = x_coords[i]
        y_path = y_coords[i]
        ax.plot(x_path, y_path, 'b')

    fig.show()
    raw_input()


if __name__ == "__main__":
    test_fcn(5, [-32, 82], [-73, 33])
