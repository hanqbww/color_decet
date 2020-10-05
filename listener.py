#!/usr/bin/env python
# coding=utf-8

import traceback
import socket
from sensor_msgs.msg import Image
import jsonpath
import json
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from colorlabeler import ColorLabeler
import imutils
import cv2
from std_msgs.msg import String
import time
import threading
from scipy.spatial import distance as dist
import rospy

IP_HOST1 = '10.14.104.196'
IP_PORT1 = 8083

IP_HOST2 = '10.14.104.196'
IP_PORT2 = 8085


# IP_HOST1 = '192.168.0.102'
# IP_PORT1 = 8083
#
# IP_HOST2 = '192.168.0.102'
# IP_PORT2 = 8085

# IP_HOST1 = '10.192.143.107'
# IP_PORT1 = 8083
#
# IP_HOST2 = '10.192.143.107'
# IP_PORT2 = 8085

AGV1_srcpose_x = 222
AGV1_srcpose_y = 333
AGV1_despose_x = 000
AGV1_despose_y = 000

AGV2_srcpose_x = 111
AGV2_srcpose_y = 222
AGV1_despose_x = 333
AGV1_despose_y = 444
# # HOST = '10.180.46.24'
# # HOST = '10.14.104.196'
# # PORT = 8080
# PORT = 8080

d1 = 0
d2 = 0

getAGVposemsg = {
    "command": "get_state",
    "serial_number": "1"
}

autoNavigation1 = {
    "command": "point_free",
    "map_name": "map11",
    "rfid": "1",
    "point": [11, 222, 333],
    "action": "upload",
    "serial_number": "1"
}

autoNavigation = {
    "command": "point_free",
    "map_name": "map11",
    "rfid": "1",
    "point": [11, 222, 333],
    "action": "upload",
    "serial_number": "1"
}

autoNavigation = {
    "command": "point_free",
    "map_name": "map11",
    "rfid": "1",
    "point": [11, 222, 333],
    "action": "upload",
    "serial_number": "1"
}

autoNavigation = {
    "command": "point_free",
    "map_name": "map11",
    "rfid": "1",
    "point": [11, 222, 333],
    "action": "upload",
    "serial_number": "1"
}

Task_Suspend = {
    "command": "goal_pause",
    "serial_number": "1"
}

Task_Resume = {
    "command": "goal_recovery",
    "serial_number": "1"
}


g_connstauts = False


class MyClient(threading.Thread):
    def __init__(self, HOST, PORT):
        threading.Thread.__init__(self)
        self.host = HOST
        self.port = PORT

    def run(self):
        self.sock = self.doconnect()
        while True:
            try:
                pass
            except OSError:
                traceback.print_exc()
                time.sleep(0.5)
                print '\nsocket connect error, doing connect in 1s .... host/port:{}/{}'.\
                      format(self.host, self.port)
                g_connstauts = False
                self.sock = self.doconnect()
            except Exception as e:
                print '\nother error occur:{}'.format(e)
                traceback.print_exc()
                g_connstauts = False
                self.sock = self.doconnect()

    def doconnect(self):
        socket.setdefaulttimeout(5)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        try:
            self.sock.connect((self.host, self.port))
            print   '\n-----------------------------------------'
            print '\nclient start connect to host/port:{}/{}'.\
                  format(self.host, self.port)
            print   '\n-----------------------------------------'
            # time.sleep(0.5)
            print   '\n链接成功'
            g_connstauts = True
            return self.sock
        except socket.timeout:
            print '\nconnect host/port:{}/{},please open tcp_server'.\
                    format(self.host, self.port)
            print
        except socket.error:
            print '\nsocket connect error,dong connect in 3s .... host/port:{}/{}'.\
                    format(self.host, self.port)
            # time.sleep(1)
            g_connstauts = False
            traceback.print_exc()
        except Exception as e:
            traceback.print_exc()
            print   '\ndo connect error: {}'.format(str(e))
            g_connstauts = False
        # return self.sock

    def send_msg(self, msg):
        try:
            msg = json.dumps(msg, sort_keys=True, indent=4, \
                             separators=(',', ': '), ensure_ascii=True)
            self.sock.sendall(msg.encode('UTF-8'))
            time.sleep(2)
        except Exception as e:
            print '\n链接中断，正在重连............host/port:{}/{}'.\
                    format(self.host, self.port)
            # print ('send_msg:()'.format(e))
            self.doconnect()
            # sock.close()

    def recv_msg(self):
        try:
            data = self.sock.recv(1024, 0x40)
            return data
        except Exception as e:
            print "None Data received!......"
            # print('recv_data:{}'.format((e)))
            # sock.close()
            time.sleep(0.5)


class Coloridentifier():
    def __init__(self):
        self.pub = rospy.Publisher('pub_with_sub', String, queue_size=10)
        # self.bridge = CvBridge()
        # self.img = rospy.Subscriber("color_dect",Image,self.callback)
        image = cv2.imread("talker_screenshot_25.08.2021.png")
        # if ret == True:
        cv2.imshow("talker", image)
        cv2.waitKey(1)

        resized = imutils.resize(image, width=300)
        blurred = cv2.GaussianBlur(resized, (5, 5), 0)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        cv2.imshow("gray is ", gray)
        cv2.waitKey(1)
        lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
        # thresh = cv2.threshold(gray,20,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)[1]
        # thresh = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
        #           cv2.THRESH_BINARY_INV,11,2)[1]
        thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)[1]
        cv2.imshow("ROS_Thresh", thresh)
        cv2.waitKey(1)
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        cv2.drawContours(resized, cnts, -1, (0, 255, 0), 2)
        cl = ColorLabeler()

        # --------------------------------------------------------#

        # for c in cnts:
        #     area = [cv2.contourArea(c)]
        # max_area = max(area)
        # if max_area > 2500:
        #     color = cl.label(lab, c)
        # cv2.drawContours(image, [c], -1, (0, 255, 0), 2)

        # -------------------------------------------------------#

        areas = []
        for c in cnts:
            area = [cv2.contourArea(c)]
            areas.append(area)
        max_area = max(areas)
        max_idx = np.argmax(areas)
        print ("max_area", max_area)

        # if max_area > 2500:
        color = cl.label(lab, cnts, max_idx)
        # print cnts[max_idx]
        rospy.loginfo("The s is %s", color)
        self.pub.publish(color)
        return color
        # return
        # print color


def thread1sendPosition(msg, (x, y)):
    thread1.send_msg(msg)
    data1 = thread1.recv_msg()
    print "data1:", data1
    if data1:
        result = jsonpath.jsonpath(json.loads(data1), '$..reply')
        if result[0] == 'get_state':
            result = jsonpath.jsonpath((json.loads(data1)), '$..pose')
            d1 = CountDistance((result[0][0], result[0][1]), (x, y))
            print d1
    else:
        d1 = None
    return d1

def thread2sendPosition(msg, (x, y)):
    thread2.send_msg(msg)
    data2 = thread2.recv_msg()
    print "data2:", data2
    if data2:
        result = jsonpath.jsonpath(json.loads(data2), '$..reply')
        if result[0] == 'get_state':
            result = jsonpath.jsonpath((json.loads(data2)), '$..pose')
            d2 = CountDistance((result[0][0], result[0][1]), (x, y))
            print d2
    else:
        d2 = None
    return d2

def CountDistance((x1, y1), (x2, y2)):
    return dist.euclidean((x1, y1), (x2, y2))

def listener():
    global thread1
    global thread2
    rospy.init_node("color_dect", anonymous=True)
    thread1 = MyClient(IP_HOST1, IP_PORT1)
    thread2 = MyClient(IP_HOST2, IP_PORT2)
    thread1.start()
    thread2.start()

if __name__ == '__main__':
    listener()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        # Coloridentifier()
        thread1sendPosition(getAGVposemsg, (111, 222))
        #get AGV2 information
        thread2sendPosition(getAGVposemsg, (111, 222))
        # Coloridentifier()
        #judge stauts
        if d1 <= 200 and d2 <= 100:
            print 'good'
            thread1.send_msg(autoNavigation1)
            thread2.send_msg(Task_Suspend)
        else:
            print 'NOT get to upload'

    # rospy.spin()
    rate.sleep()
