#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
This scripts reads a bag file containing RGBD data, adds the corresponding
PointCloud2 messages, and saves it again into a bag file. Optional arguments
allow to select only a portion of the original bag file.
"""

import argparse
import sys
import os

if __name__ == '__main__':

    # parse command line
    parser = argparse.ArgumentParser(description='''
    This scripts reads a bag file containing RGBD data, 
    adds the corresponding PointCloud2 messages, and saves it again into a bag file. 
    Optional arguments allow to select only a portion of the original bag file.  
    ''')
    parser.add_argument('--start', help='skip the first N seconds of  input bag file (default: 0.0)',default=0.00)
    parser.add_argument('--duration', help='only process N seconds of input bag file (default: off)')
    parser.add_argument('--nth', help='only process every N-th frame of input bag file (default: 1)',default=1)
    parser.add_argument('--skip', help='skip N blocks in the beginning (default: 1)', default=1)
    parser.add_argument('--compress', help='compress output bag file', action='store_true')
    parser.add_argument('inputbag', help='input bag file')
    parser.add_argument('outputbag', nargs='?',help='output bag file')
    args = parser.parse_args()

    import roslib;
    # roslib.load_manifest('fastfusion_ros')
    import rospy
    import rosbag
    import sensor_msgs.msg
    import cv2 as cv
    from cv_bridge import CvBridge, CvBridgeError
    import struct
    import tf
    import tf.msg

    if not args.outputbag:
        args.outputbag = os.path.splitext(args.inputbag)[0] + "-points.bag"

    print "Processing bag file:"
    print "  in:",args.inputbag
    print "  out:",args.outputbag
    print "  starting from: %s seconds"%(args.start)

    if args.duration:
        print "  duration: %s seconds"%(args.duration)

    print "  saving every %s-th frame"%(args.nth)
    args.skip = float(args.skip)
    print "  skipping %s blocks"%(args.skip)

    inbag = rosbag.Bag(args.inputbag,'r')
    if args.compress:
        param_compression = rosbag.bag.Compression.BZ2
    else:
        param_compression = rosbag.bag.Compression.NONE

    outbag = rosbag.Bag(args.outputbag, 'w', compression=param_compression)

    depth_camera_info = None
    rgb_camera_info = None
    depth_image = None
    rgb_image_color = None
    cortex = None

    nan = float('nan')
    bridge = CvBridge()
    frame = 0
    transforms = dict()

    try:
        time_start = None
        for topic, msg, t in inbag.read_messages():
            if time_start==None:
                time_start=t
            if t - time_start < rospy.Duration.from_sec(float(args.start)):
                continue
            if args.duration and (t - time_start > rospy.Duration.from_sec(float(args.start) + float(args.duration))):
                break
            print "t=%f\r"%(t-time_start).to_sec(),
            if topic == "/tf":
                for transform in msg.transforms:
                    transforms[ (transform.header.frame_id,transform.child_frame_id) ] = transform
                continue
            if topic == "/imu":
                imu = msg
                continue
            if topic == "/camera/depth/camera_info":
                depth_camera_info = msg
                continue
            if topic == "/camera/rgb/camera_info":
                rgb_camera_info = msg
                continue
            if topic == "/camera/rgb/image_color" and rgb_camera_info:
                rgb_image_color = msg
                continue
            if topic == "/camera/depth/image" and depth_camera_info and rgb_image_color and rgb_camera_info:# and imu:
                depth_image = msg
                # now process frame

                if depth_image.header.stamp - rgb_image_color.header.stamp > rospy.Duration.from_sec(1/30.0):
                    continue

                frame += 1
                if frame % float(args.nth) ==0:
                    if args.skip > 0:
                        args.skip -= 1
                    else:
                        # store messages
                        msg = tf.msg.tfMessage()
                        msg.transforms = list( transforms.itervalues() )
                        outbag.write("/tf",msg,t)
                        transforms = dict()

                        messages_with_header = [depth_image, rgb_image_color]
                        # remove leading "/" from frame_id
                        for i in range(len(messages_with_header)):
                            if len(messages_with_header[i].header.frame_id) and messages_with_header[i].header.frame_id == '/':
                                messages_with_header[i].header.frame_id.header.frame_id = \
                                    messages_with_header[i].header.frame_id.header.frame_id[1:]

                        #                    outbag.write("/imu",imu,t)
                        outbag.write("/camera/depth/camera_info",depth_camera_info,t)
                        outbag.write("/camera/depth/image",depth_image,t)
                        outbag.write("/camera/rgb/camera_info",rgb_camera_info,t)
                        outbag.write("/camera/rgb/image_color",rgb_image_color,t)

                depth_image = None
                rgb_image_color = None
                continue
            if topic not in ["/tf","/imu",
                             "/camera/depth/camera_info","/camera/rgb/camera_info",
                             "/camera/rgb/image_color","/camera/depth/image"]:
                # anything else: pass thru
                outbag.write(topic,msg,t)
    except:
        print('Something bad happened, bag not saved in full')

    outbag.close()
    print
