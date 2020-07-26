#!/usr/bin/env python

"""
Showing costmap with matplotlib
"""

import threading

from matplotlib import pyplot as plt
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import tf
import tf2_ros


class TfListener:
    """
    Use threading to get tf map -> base_link and save the orientation to global vars.

    Attributes:
        _tf_buffer (tf2_ros.Buffer):
    """
    def __init__(self):
        self._tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf_buffer)  # for buffer

    def _loop_update(self):
        """
        Update orientation in loop.
        """
        rate = rospy.Rate(hz=10)
        while not rospy.is_shutdown():
            try:
                t = self._tf_buffer.lookup_transform(
                    target_frame="map",
                    source_frame="base_link",
                    time=rospy.Time())
            except Exception as err:
                rospy.loginfo(err)
            else:
                global ORIENTATION
                ORIENTATION[0] = t.transform.rotation.x
                ORIENTATION[1] = t.transform.rotation.y
                ORIENTATION[2] = t.transform.rotation.z
                ORIENTATION[3] = t.transform.rotation.w
            rate.sleep()

    def start_thread(self):
        """
        Start threading
        """
        thread = threading.Thread(target=self._loop_update, name='_loop_update')
        thread.start()


class Viewer:
    """
    Use matplotlib to show costmap and plot cmd_vel predicted locations.
    """
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        plt.ion()
        plt.show()

    def update(self):
        """
        Update matplotlib ax.
        """
        self._plot_costmap()
        self._plot_robot()
        self._plot_cmdvel()
        self.fig.canvas.draw()

    def _plot_costmap(self):
        """
        Plot costmap.
        """
        self.ax.cla()
        self.ax.imshow(X=DATA[::-1], cmap="jet")  # use img to show costmap
        self.ax.grid(which='both', color='grey')

    def _plot_robot(self):
        """
        Plot robot with orientation (assuming the location is at the center).
        """
        try:
            _, _, t = tf.transformations.euler_from_quaternion(ORIENTATION)
            t = np.degrees(t) - 90.0
            x = INFO.width / 2.0
            y = INFO.height / 2.0
            self.ax.plot(x, y, marker=(3,0,t), markersize=10, linestyle='None')
        except Exception as err:
            rospy.loginfo(err)

    def _plot_cmdvel(self):
        """
        Plot cmd_vel on costmap with samples
        """
        xs_r_img, ys_r_img = self._get_predict_locs()
        if xs_r_img is not None:
            self.ax.scatter(xs_r_img, ys_r_img, c="white", s=1)

    def _get_predict_locs(self):
        """
        Use cmd_vel to get predicted locations of car in image coordinate.

        Return:
            xs_r_img (np.array):
            ys_r_img (np.array):
        """
        dt = 0.5
        count = 4
        _i = np.arange(start=1, stop=count+1, step=1)
        xs_r_img = None
        ys_r_img = None

        if VX != 0.0:

            # -- get xs, ys in car coordidate
            if abs(WZ) < 1e-5:
                xs = VX * dt * _i
                ys = np.zeros(shape=(count,))
            else:
                r = VX / WZ
                ts = WZ * dt * _i
                xs = r * np.sin(ts)
                ys = r * (1.0 - np.cos(ts))

            # -- rotate to map coordinate
            _, _, th0 = tf.transformations.euler_from_quaternion(ORIENTATION)
            xs_r = xs*np.cos(th0) - ys*np.sin(th0)
            ys_r = xs*np.sin(th0) + ys*np.cos(th0)

            # -- scale and offset to img coordinate
            xs_r_img = xs_r/INFO.resolution + INFO.width / 2.0
            ys_r_img = -ys_r/INFO.resolution + INFO.height / 2.0

        else:
            pass

        return xs_r_img, ys_r_img

# --

def cb_costmap(msg):
    """
    Callback for topic costmap.
    """
    global DATA, INFO
    INFO = msg.info
    DATA = np.reshape(a=np.array(object=msg.data, dtype=np.float), newshape=(INFO.height, INFO.width))

def cb_cmdvel(msg):
    """
    Callback for topic cmd_vel.
    """
    global VX, WZ
    VX = msg.linear.x
    WZ = msg.angular.z

if __name__ == "__main__":

    # -- global vars
    INFO = None
    DATA = None
    ORIENTATION = [0, 0, 0, 0]  # quaternion
    VX = 0.0  # m/s
    WZ = 0.0  # rad/s

    # -- ros node and params
    rospy.init_node(name="viewer", anonymous=False)
    top_costmap = rospy.get_param(param_name="~costmap", default="/move_base/local_costmap/costmap")
    top_cmd = rospy.get_param(param_name="~cmd_vel", default="/cmd_vel")

    # -- ros function
    rospy.Subscriber(name=top_costmap, data_class=OccupancyGrid, callback=cb_costmap)
    rospy.Subscriber(name=top_cmd, data_class=Twist, callback=cb_cmdvel)

    # -- get car orientation and save to ORIENTATION
    tf_listener = TfListener()
    tf_listener.start_thread()

    # -- matplotlib show costmap
    viewer = Viewer()
    rate = rospy.Rate(hz=2)
    while not rospy.is_shutdown():
        viewer.update()
        rate.sleep()
