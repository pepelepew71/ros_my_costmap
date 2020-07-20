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
    data = np.reshape(a=data, newshape=(height, width))
    DATA = data[::-1]

def plot():
    fig = plt.gcf()
    ax = plt.gca()
    ax.cla()
    ax.imshow(X=DATA, cmap="jet")
    ax.grid()
    fig.canvas.draw()
    plt.tight_layout()

if __name__ == "__main__":

    plt.subplots()
    plt.ion()
    plt.show()

    rospy.init_node(name="local_viewer", anonymous=False)
    rospy.Subscriber(name="/move_base/local_costmap/costmap", data_class=OccupancyGrid, callback=cb)

    rate = rospy.Rate(hz=2)
    while not rospy.is_shutdown():
        plot()
        rate.sleep()
