import matplotlib.pyplot as plt
import numpy as np

def show_world(world):
  fig = plt.figure()
  world = world.copy()
  world[world==-1] = -100
  img2 = plt.imshow(world[::-1], cmap='gray_r' ,interpolation='nearest')
  plt.show(block=True)

def occupancygrid_to_numpy(msg):
	return np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

# computes the straight line distance between two points
def euclid_dist(pt1, pt2):
    return np.linalg.norm(np.subtract(pt1, pt2))

# converts row and column coordinates to x and y coordinates
def row_col_to_coord(row_col, resolution, world):
  return (np.array(row_col)[::-1] - np.array(world.shape)[::-1]//2) * resolution 

# converts x and y coordinates to row and column coordinates
def coord_to_row_col(coord, resolution, world):
  return (np.array(coord)[::-1] / resolution).astype(int) + np.array(world.shape)//2

# calculates the transformation matrix from the global coordinate system 
# to the robot's coordinate system
def get_global_to_local_transform(currentX, currentY, currentAngle):
  translationg_mat = np.array(
    [[1,0,-currentX],
     [0,1,-currentY],
     [0,0,1]]
    )
  rotation_mat = np.array(
    [[np.math.cos(currentAngle), np.math.sin(currentAngle), 0],
    [-np.math.sin(currentAngle), np.math.cos(currentAngle), 0],
    [0, 0, 1]]
    )

  composition_mat = np.dot(rotation_mat, translationg_mat)
  return composition_mat