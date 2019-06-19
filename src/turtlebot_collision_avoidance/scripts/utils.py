import numpy as np
from math import sin, cos, atan2
from geometry_msgs.msg._Point import Point

def vec(*args):
	if len(args) == 1:
	  	if type(args[0]) == tuple:
			return np.array(args[0])
		elif type(args[0]) == Point:
			return np.array((args[0].x, args[0].y, args[0].z))
		else:
			return np.array(args)
	else:
		return np.array(args)

def hat(v):
	if v.shape == (3, 1) or v.shape == (3,):
		return np.array([
				[0, -v[2], v[1]],
				[v[2], 0, -v[0]],
				[-v[1], v[0], 0]
			])
	elif v.shape == (6, 1) or v.shape == (6,):
		return np.array([
				[0, -v[5], v[4], v[0]],
				[v[5], 0, -v[3], v[1]],
				[-v[4], v[3], 0, v[2]],
				[0, 0, 0, 0]
			])
	else:
		raise ValueError

# def create_twist(trans, rot):
# 	v = np.cross()
# 	return 

def adj(g):
	if g.shape == (4, 4):
		R = g[0:3,0:3]
		p = g[0:3,3]
		result = np.zeros((6, 6))
		result[0:3,0:3] = R
		result[0:3,3:6] = hat(p) * R
		result[3:6,3:6] = R
		return result
	else:
		raise ValueError

def twist_from_tf(g):
	return vec(g[0,2], g[1,2], atan2(g[1,0], g[0,0]))

def rotation2d(theta):
	return np.array([
			[cos(theta), -sin(theta)],
			[sin(theta), cos(theta)]
		])

def rigid(twist):
	return np.array([
			[cos(twist[2]), -sin(twist[2]), twist[0]],
			[sin(twist[2]), cos(twist[2]), twist[1]],
			[0, 0, 1]
		])

def nan_helper(y):
    """Helper to handle indices and logical indices of NaNs.

    Input:
        - y, 1d numpy array with possible NaNs
    Output:
        - nans, logical indices of NaNs
        - index, a function, with signature indices= index(logical_indices),
          to convert logical indices of NaNs to 'equivalent' indices
    Example:
        >>> # linear interpolation of NaNs
        >>> nans, x= nan_helper(y)
        >>> y[nans]= np.interp(x(nans), x(~nans), y[~nans])
    """
    return np.isnan(y), lambda z: z.nonzero()[0]

def euclidean_dist(pos1, pos2):
    """Euclidean distance between current pose and the goal."""
    return np.linalg.norm(pos1 - pos2)

def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n

