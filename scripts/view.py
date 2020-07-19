#!/usr/bin/env python

from matplotlib import pyplot as plt
import numpy as np

import rospy
from nav_msgs.msg import OccupancyGrid

def cb(msg):
    global DATA
    height = msg.info.height
    width = msg.info.width
    resolution = msg.info.resolution
    data = np.array(object=msg.data, dtype=np.float)
    DATA = np.reshape(a=data, newshape=(height, width))

def plot():
    fig = plt.gcf()
    ax = plt.gca()
    ax.cla()
    ax.contourf(X, Y, DATA)
    ax.scatter(0, 0, s=100, marker="s", color="white")
    ax.axis("equal")
    ax.grid()
    fig.canvas.draw()
    plt.tight_layout()

if __name__ == "__main__":

    _resol = 0.05
    _x = np.arange(-5.0, 5.0, _resol)
    _y = np.arange(-5.0, 5.0, _resol)
    X, Y = np.meshgrid(_x, _y)
    DATA = np.zeros(shape=(200, 200), dtype=float)
    plt.subplots()
    plt.ion()
    plt.show()

    rospy.init_node(name="view_costmap", anonymous=False)
    rospy.Subscriber(name="/move_base/local_costmap/costmap", data_class=OccupancyGrid, callback=cb)

    rate = rospy.Rate(hz=2)
    while not rospy.is_shutdown():
        plot()
        rate.sleep()