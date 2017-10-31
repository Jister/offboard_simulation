#!/usr/bin/env python
import rospy
import xlrd
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped

def pos_callback(data):
    global x
    global y
    x = data.pose.position.x;
    y = data.pose.position.y; 

if __name__ == '__main__':
    rospy.init_node('plot', anonymous=True)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pos_callback)
    rate = rospy.Rate(10) 

    data = xlrd.open_workbook('/home/ubuntu/catkin_ws/src/offboard_simulation/data/data20Hz_insert.xls')
    table = data.sheets()[0]
    x1 = table.col_values(0)
    y1 = table.col_values(1)

    plt.plot(x1, y1, '-')
    plt.axis('equal')
    plt.axis([-1, 11, -1, 10])
    plt.ion()

    while not rospy.is_shutdown():
        plt.scatter(x, y, c='r', marker='.')
        plt.pause(0.01)
        rate.sleep()