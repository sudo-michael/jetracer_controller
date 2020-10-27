import numpy as np
from Grid.GridProcessing import Grid
from Shapes.ShapesFunctions import *

# Specify the  file that includes dynamic systems
from dynamics.Humannoid6D_sys1 import *
from dynamics.DubinsCar4D2 import *
import scipy.io as sio

import math

""" USER INTERFACES
- Define grid

- Generate initial values for grid using shape functions

- Time length for computations

- Run
"""

# area of lab in meters
# (x, y, v, theta)
# g = Grid(np.array([-3.0, -1.0, -1.0, -math.pi / 2]),
         # np.array([4.0, 3.0, 4.0, math.pi / 2]),
         # 4, np.array([60, 60, 20, 36]), [3])

g = Grid(np.array([-1.0, -1.0, -1.0, -math.pi]), np.array([2.0, 2.0, 1.0, math.pi]),
         4, np.array([40, 40, 20, 20]), [3])
# Define my object
my_car = DubinsCar4D()

# Use the grid to initialize initial value function
Initial_value_f = CylinderShape(g, [3,4], np.zeros(4), 0.25)

# Look-back lenght and time step
lookback_length = 2.0
t_step = 0.05

small_number = 1e-5
tau = np.arange(start = 0, stop = lookback_length + small_number, step = t_step)
print("Welcome to optimized_dp \n")

'''
Assign one of the following strings to `compMethod` to specify the characteristics of computation
"none" -> compute Backward Reachable Set
"minVwithV0" -> compute Backward Reachable Tube
"maxVwithVInit" -> compute max V over time
"minVwithVInit" compute min V over time
'''
compMethod = "minVwithV0"
my_object  = my_car
my_shape = Initial_value_f

