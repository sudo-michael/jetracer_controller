import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

import pandas as pd

fig, ax = plt.subplots()
car = pd.read_csv('l2.csv')
car = car[['x', 'y', 'theta']]

car_width = 0.3
car_height = 0.5
# rectangle = plt.Rectangle((0, 0), car_width, car_height, color='b', angle=90.0)

circle1 = plt.Circle((0, 0), 0.2, color='tab:orange')
circle1.set_fill(False)

circle2 = plt.Circle((0, 0), 0.3, color='b')

line, = ax.plot(car['x'], car['y'], color='k')
def init_func():
    ax.add_patch(circle1)
    ax.add_patch(circle2)
    # ax.add_patch(rectangle)
    return circle1,

def update(num, x, y, line, theta):
    line.set_data(x[:num], y[:num])
    line.axes.axis([-10.0, 10.0, -10.0, 10.0])

    # rectangle = plt.Rectangle((x[num], y[num]), 0.3, 0.5, color='b')
    # rectangle.set_xy([x[num], y[num]])
    # rectangle.angle = np.rad2deg(theta[num] + math.pi/2)

    # if rectangle.angle 
    # rectangle.set_xy([x[num] + car_width / 2, y[num] + car_height / 2])
    circle2.set_center([x[num], y[num]])

    return line, circle1, circle2,

ani = animation.FuncAnimation(fig, 
                              update, 
                              len(car), # frames
                              fargs=[car['x'], car['y'], line, car['theta']], # arg to update
                              interval=100,  #delay inbetween frames (ms) 
                              init_func = init_func,
                              blit=False)
plt.show()
# ani.save('test.mp4', writer='ffmpeg', codec='h264')



# record specific topic
# rosbag record -O subset /turtle1/cmd_vel /turtle1/pose