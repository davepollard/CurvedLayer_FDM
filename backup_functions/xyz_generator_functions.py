"""
David Pollard

FARSCOPE CDT
Bristol Robotics Laboratory

Released under the MIT License


Generates 2D toolpath covering a surface
"""
from __future__ import division
import numpy as np
import matplotlib.pyplot as plt

from honeycomb_pattern import gen_RectHex, gen_PolyHex


TEST_SELECTION_VALUE = 6
TEST_VISUALISE_PATH = True
TEST_CHECK_SIZE = True

TEST_BASE_R = 30
TEST_BASE_H = 40


#
# supplementary functions...
#
def frange(start, stop, step):
    '''
    Returns numpy array of equally spaced points in range [start, stop)
    '''
    ret_array = []
    x = start
    while x < stop:
        ret_array.append(x)
        x += step
    return np.array(ret_array)

def retrieve_xyz(selection=0, base_r=TEST_BASE_R, base_h=TEST_BASE_H):
    '''
    Retrieves required XYZ values to be mapped to cylinder surface

    Inputs:
     - selection        Which path to extract
     - base_r [=10]     Radius of cylinder
     - base_h [=10]     Height of cylinder
    '''
    x_coord = []
    y_coord = []
    z_coord = []
    if selection == 0:
        [x_coord, y_coord] = get_path_0(base_h)
    elif selection == 1:
        [x_coord, y_coord] = get_path_1(base_r, base_h)
    elif selection == 2:
        [x_coord, y_coord] = get_path_2(base_r, base_h)
    elif selection == 3:
        [x_coord, y_coord] = get_path_3(base_r, base_h)
    elif selection == 4:
        [x_coord, y_coord] = get_path_4(base_r, base_h)
    elif selection == 5:
        [x_coord, y_coord] = get_path_5(base_r, base_h)
    elif selection == 6:
        [x_coord, y_coord] = get_path_6(base_r, base_h)
    else:
        raise Exception("Requested path not recognised")

    x_coord = np.array(x_coord)
    y_coord = np.array(y_coord)

    # Fill
    x_coord, y_coord = fill_values(x_coord, y_coord)

    # Check length
    for i, [x_path, y_path] in enumerate(zip(x_coord, y_coord)):
        if len(x_path) > 499:
            print "warning... long path exists, splitting"
            if len(x_path) > 998:
                raise Exception("No handling for long paths")
            # split
            split_idx = int(round(len(x_path)/2.0))
            x_coord.append(x_path[split_idx:-1])
            y_coord.append(y_path[split_idx:-1])
            x_coord[i] = x_coord[i][0:split_idx]
            y_coord[i] = y_coord[i][0:split_idx]


    return [x_coord, y_coord, z_coord]


def get_path_0(base_h):
    '''
    Boring square
    '''
    x_val = [0, 20, 20, 0, 0]
    y_val = [0.5*base_h, 0.5*base_h, 0.75*base_h, 0.75*base_h, 0.5*base_h]
    return [[x_val], [y_val]]


def get_path_1(base_r, base_h):
    '''
    Series of sine waves
    '''
    num_waves = 3
    cylinder_circum = np.pi*base_r*2
    x_val = np.linspace(0, cylinder_circum, 30)

    x_scaled_value = 2*num_waves*np.pi*x_val/cylinder_circum
    y_val = 0.8*base_h + 0.15*base_h*np.sin(x_scaled_value)
    y_val2 = 0.8*base_h + 0.15*base_h*np.sin(x_scaled_value + np.pi/2)
    y_val3 = 0.8*base_h + 0.15*base_h*np.sin(x_scaled_value + np.pi)
    y_val4 = 0.8*base_h + 0.15*base_h*np.sin(x_scaled_value + 3*np.pi/2)

    x_final = [x_val, x_val[::-1], x_val, x_val[::-1]]
    y_final = [y_val, y_val2, y_val3, y_val4]
    return [x_final, y_final]


def get_path_2(base_r, base_h):
    '''
    Cross hatch
    '''
    mesh_spacing = 10
    cover_region = [0.5, 0.33]

    cylinder_circum = np.pi*base_r*2
    num_x_points = round(cylinder_circum*cover_region[0]/mesh_spacing)
    num_y_points = round(base_h*(1-cover_region[1])/mesh_spacing)

    # Generate x, y values
    x_range = np.linspace(0, cylinder_circum*cover_region[0], num_x_points)
    y_range = np.linspace(base_h*cover_region[1], base_h, num_y_points)

    x_section = [min(x_range), max(x_range), max(x_range), min(x_range)]
    y_section = [min(y_range), max(y_range), max(y_range), min(y_range)]

    # Create layer 1
    layer_0_x = [val for val in x_range for _ in (0, 1)]
    layer_0_y = sum([y_section for _ in x_range[0::2]], [])
    layer_0_y = layer_0_y[0:len(layer_0_x)]

    # Create layer 2
    layer_1_y = [val for val in y_range for _ in (0, 1)]
    layer_1_x = sum([x_section for _ in y_range[0::2]], [])
    layer_1_x = layer_1_x[0:len(layer_1_y)]


    x_val = [layer_0_x, layer_1_x]
    y_val = [layer_0_y, layer_1_y]


    return [x_val, y_val]


def get_path_3(base_r, base_h):
    '''
    Rectangular hexagon pattern
     - alternate lines reversed for print speed
    '''
    hex_r = 10
    cover_region = [0.5, 0.66]

    cylinder_circum = np.pi*base_r*2

    # Retrive values
    x_val, y_val = gen_RectHex(hex_r, [0, cylinder_circum*cover_region[0]], \
                        [base_h*cover_region[1], base_h])

    for i in range(0, len(x_val), 2):
        x_val[i] = x_val[i][::-1]
        y_val[i] = y_val[i][::-1]


    x_val = np.array(x_val)
    y_val = np.array(y_val)

    return [x_val, y_val]


def get_path_4(base_r, base_h):
    '''
    Series of vertical lines
    '''
    line_length = 0.5*base_h
    x_val = [[x, x] for x in np.linspace(0, base_r*np.pi, 20)]
    y_val = [[base_h-line_length, base_h] for _ in range(20)]
    return [x_val, y_val]


def get_path_5(base_r, base_h):
    '''
    Ellipsoidal hexagonal pattern
    '''
    hex_r = 10
    circ_r = 0.2*base_h

    ang = np.linspace(0, 2*np.pi, 100)
    cover_x = np.array(circ_r*np.cos(ang) + hex_r)*2
    cover_y = circ_r*np.sin(ang) + base_h-circ_r-5

    # Retrive values
    x_val, y_val = gen_PolyHex(hex_r/2, [cover_x, cover_y])

    for i in range(0, len(x_val), 2):
        x_val[i] = x_val[i][::-1]
        y_val[i] = y_val[i][::-1]

    return [x_val, y_val]


def get_path_6(base_r, base_h):
    '''
    University of Bristol Logo - NB: saved in data_files
     - file format is list of x,y tuples
      - - e.g. [[(x0, y0), (x1, y1)], [(x2, y2), (x3, y3)]]
     - values normalised and centred to be in range [-1, 1]
    '''
    filename = '/data_files/uob_logo_path.npy'
    try:
        data = np.load(filename)
    except IOError:
        # if called directly it needs the extra dot
        filename = '.' + filename
        data = np.load(filename)
    else:
        raise IOError("Unable to import logo NPY file")

    # Calculate offset and scale
    x_offset = np.pi*base_r
    y_offset = 0.55 * base_h
    scale_val = 0.4*base_h

    # Downsample - causes errors in robot motion planner if points are too close
    x_val = [[(-1*x*scale_val)+x_offset for x, _ in path[0::3]] for path in data]
    y_val = [[(y*scale_val)+y_offset for _, y in path[0::3]] for path in data]

    return [x_val, y_val]







def fill_values(x_val, y_val):
    '''
    Fills out values based on x spacing
    '''
    fill_max_x = 1
    final_x = []
    final_y = []

    for path_idx, _ in enumerate(x_val):
        x_path = x_val[path_idx]
        y_path = y_val[path_idx]

        new_x_path = [x_path[0]]
        new_y_path = [y_path[0]]


        for i in range(1, len(x_path)):
            if abs(x_path[i]-x_path[i-1]) < fill_max_x:
                new_x_path.append(x_path[i])
                new_y_path.append(y_path[i])

            else:
                num_required_points = np.ceil(np.abs(x_path[i]-x_path[i-1])/fill_max_x)
                new_x_path.extend(np.linspace(x_path[i-1], x_path[i], num_required_points+1))
                new_y_path.extend(np.linspace(y_path[i-1], y_path[i], num_required_points+1))

        final_x.append(new_x_path)
        final_y.append(new_y_path)


    return [final_x, final_y]



def visualise_path(x_coord, y_coord):
    '''
    For visualising the generated pattern in 2D
    '''
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for i, _ in enumerate(x_coord):
        x_path = x_coord[i]
        y_path = y_coord[i]
        z_path = np.zeros(np.shape(x_path))

        ax.plot(x_path, y_path, z_path, 'b')

    if TEST_CHECK_SIZE:
        x = np.pi * TEST_BASE_R * 2
        y = TEST_BASE_H
        x_dim = [0, x, x, 0, 0]
        y_dim = [0, 0, y, y, 0]
        ax.plot(x_dim, y_dim, [0, 0, 0, 0, 0], 'k--')


    fig.show()





if __name__ == "__main__":
    [x_final_path, y_final_path, _] = retrieve_xyz(TEST_SELECTION_VALUE)

    if TEST_VISUALISE_PATH:
        visualise_path(x_final_path, y_final_path)
        raw_input()
