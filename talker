#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from rospy_message_converter  import message_converter


def talker():
    pub = rospy.Publisher('color_dect', Image, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(50) # 10hz
    bridge = CvBridge()
    # Video = cv2.VideoCapture(0)

    while not rospy.is_shutdown():

        # ret, img = Video.read()
        # img = cv2.imread("example_shapes.png")
        # img=cv2.imread("talker_screenshot_25.08.2022.png")
        # img = cv2.imread("talker_screenshot_25.08.2020.png")
        img = cv2.imread("talker_screenshot_25.08.2022.png")
        # if ret == True:
        cv2.imshow("talker",img)
        cv2.waitKey(1)

        # dictionary = {'data':'Howdy'}
        # message = message_converter.convert_dictionary_to_ros_message('std_msgs/String',dictionary)
        # print(message)


        pub.publish(bridge.cv2_to_imgmsg(img,"bgr8"))

        #hello_str = "hello world again %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

