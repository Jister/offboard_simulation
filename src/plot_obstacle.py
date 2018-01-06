#!/usr/bin/env python
import rospy
import xlrd
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import PoseStamped

def pos_callback(data):
    global x
    global y
    x = data.pose.position.x;
    y = data.pose.position.y; 

if __name__ == '__main__':
    rospy.init_node('plot_obstacle', anonymous=True)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pos_callback)
    rate = rospy.Rate(10) 

    pi,sin,cos = np.pi,np.sin,np.cos
    theta = np.linspace(0, 2*pi, 360)
    oo = np.array([[5.0, 3.0], [3.0, -5.0], [10.0, -3.0],[12.0, -1.0],[20.0, -8.0]]) 
    for i in range(0,5) :
        x1 = oo[i][0] + 1.0 * cos(theta)
        y1 = oo[i][1] + 1.0 * sin(theta)
        plt.plot(x1, y1, 'k-')
        plt.fill(x1,y1,'k')
    x2 = [20, 22, 22, 20, 20]
    y2 = [0, 0, 10, 10, 0]
    plt.plot(x2, y2, 'k-')
    plt.fill(x2,y2,'k')
    plt.axis('equal')
    plt.axis([-2, 32, -12, 12])
    plt.ion()

    while not rospy.is_shutdown():
        plt.scatter(x, y, c='r', marker='.')
        plt.pause(0.01)
        rate.sleep()