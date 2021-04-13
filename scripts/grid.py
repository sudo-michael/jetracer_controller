import math
import numpy as np

class Grid:
    def __init__(self, min, max, dims, pts_each_dim, pDim=[]):
        self.max = max
        self.min = min
        self.dims = len(pts_each_dim)
        self.pts_each_dim = pts_each_dim
        self.pDim = pDim

        # Make some modifications to the initialized
        for dim in pDim:
            self.max[dim] = self.min[dim] + (self.max[dim] - self.min[dim]) * (
                1 - 1 / self.pts_each_dim[dim]
            )
        self.dx = (self.max - self.min) / (self.pts_each_dim - 1.0)

        """
        Below is re-shaping the self.vs so that we can make use of broadcasting
        self.vs[i] is reshape into (1,1, ... , pts_each_dim[i], ..., 1) such that pts_each_dim[i] is used in ith position
        """
        self.vs = []
        self.__grid_points = []
        for i in range(0, dims):
            tmp = np.linspace(self.min[i], self.max[i], num=self.pts_each_dim[i])
            broadcast_map = np.ones(self.dims, dtype=int)
            broadcast_map[i] = self.pts_each_dim[i]
            self.__grid_points.append(tmp)
            tmp = np.reshape(tmp, tuple(broadcast_map))
            self.vs.append(tmp)

    def get_index(self, state):
        index = []
        for i, s in enumerate(state):
            idx = np.searchsorted(self.__grid_points[i], s)
            if idx > 0 and (
                idx == len(self.__grid_points[i])
                or math.fabs(s - self.__grid_points[i][idx - 1])
                < math.fabs(s - self.__grid_points[i][idx])
            ):
                '''
                if i == 0:
                    rospy.loginfo("x-inbetween: {} - {}".format(self.__grid_points[i][idx-1], self.__grid_points[i][idx]))
                elif i==1:
                    rospy.loginfo("y-inbetween: {} - {}".format(self.__grid_points[i][idx-1], self.__grid_points[i][idx]))
                '''
                index.append(idx - 1)
            else:
                index.append(idx)

        return index

    def get_value(self, V, state):
        index = self.get_index(state)
        print(index)
        return V[tuple(index)]

