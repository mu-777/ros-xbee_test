#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import rospy
import serial

from abc import ABCMeta, abstractmethod
from xbee import XBee # https://code.google.com/p/python-xbee/
from std_msgs.msg import Float32

DEFAULT_NODE_NAME = 'xbee_test'
DEFAULT_XBEE_DATA_PUB_TOPIC = 'xbee_data'


def byteToInt(byte):
    if hasattr(byte, 'bit_length'):
        # This is already an int
        return byte
    return ord(byte) if hasattr(byte, 'encode') else byte[0]


def intToByte(i):
    return chr(i) if hasattr(bytes(), 'encode') else bytes([i])


def stringToBytes(s):
    return s.encode('ascii')


class XBeeDataPublisher(object):
    __metaclass__ = ABCMeta  # ABstract Class

    def __init__(self, pub_data):
        port = rospy.get_param('~xbee_port', default='/dev/ttyUSB0')
        baudrate = rospy.get_param('~xbee_baudrate', default=9600)

        self._xbee = XBee(serial.Serial(port, baudrate))
        self._pub_data = pub_data
        self._data_pub = rospy.Publisher(DEFAULT_XBEE_DATA_PUB_TOPIC,
                                         self._pub_data.__class__,
                                         queue_size=1)
        self.is_activated = False

    def activate(self):
        self.is_activated = True

    @abstractmethod
    def _convert_xbee_data(self, binary_data):
        self._pub_data = binary_data

    def publish_data(self):
        if not self.is_activated:
            self.activate()
        self._convert_xbee_data(self._xbee._wait_for_frame())
        # self._convert_xbee_data(self._xbee.wait_read_frame()) is not available for current XBee
        self._data_pub.publish(self._pub_data)


class DistSensorDataViaXBeePublisher(XBeeDataPublisher):
    def __init__(self):
        self._dist = 0.0
        super(DistSensorDataViaXBeePublisher, self).__init__(Float32())

    def _convert_xbee_data(self, binary_data):
        # packet = ""
        # for data in binary_data.data:
        #     packet += hex(byteToInt(data)) + '/'
        # print(packet)

        self._dist = byteToInt(binary_data.data[-2]) * 256 + byteToInt(binary_data.data[-1])
        print(self._dist)
        self._pub_data = self._dist

# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node(DEFAULT_NODE_NAME, anonymous=True)
rate_mgr = rospy.Rate(100)  # Hz

dist_data_publisher = DistSensorDataViaXBeePublisher()
dist_data_publisher.activate()

while not rospy.is_shutdown():
    dist_data_publisher.publish_data()
    rate_mgr.sleep()
