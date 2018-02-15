"""
This scripts checks if the the point cloud contains RGB information or not (for debug purpose)
"""

import argparse
import os
if __name__ == '__main__':

    # parse command line
    parser = argparse.ArgumentParser(description='''
    This scripts checks if the the point cloud contains RGB information or not (for debug purpose)
    ''')
    parser.add_argument('inputbag', help='input bag file')
    args = parser.parse_args()

    import rospy
    import rosbag
    import sensor_msgs.point_cloud2 as pcl2

    inbag = rosbag.Bag(args.inputbag,'r')
    for topic, msg, t in inbag.read_messages():
        if topic == '/camera/rgb/points':
            for p in pcl2.read_points(msg):
                print(p)
        print("\n\n")

