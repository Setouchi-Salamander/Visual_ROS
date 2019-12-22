#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity
import rospy
from Visual_ROS.msg import data
import serial
import struct
import time

# Subscribeする対象のトピックが更新されたら呼び出されるコールバック関数
# 引数にはトピックにPublishされるメッセージの型と同じ型を定義する
def callback(data):
    data_x_high= (data.x & 0xFF00)>>8
    data_x_low= data.x & 0x00FF
    data_y_high= (data.y & 0xFF00)>>8
    data_y_low= data.y & 0x00FF
    data_cnt=data.cnt_taget
    print "x=%d y=%d cnt=%d x_high=%x x_low=%x y_high=%x y_low=%x   "%(data.x ,data.y,data.cnt_taget,data_x_high,data_x_low,data_y_high,data_y_low)

    try:
        port = serial . Serial ( # open port serail0 ,921600
        port = "/ dev / serial0 " ,
        baudrate =115200 ,
        parity = serial . PARITY_NONE ,
        stopbits = serial . STOPBITS_ONE ,
        bytesize = serial . EIGHTBITS ,
        timeout = 0.01
        )
        packed_data_sendUART = struct.pack('B''B''B''B''B''B''B' ,253 , data_x_high , data_x_low , data_y_high  ,data_y_low , data_cnt ,254)
        port.write(packed_data_sendUART)
    except :
        rospy.logerr("cannnot open port")


    


def sender():
    rospy.init_node('sender', anonymous=True)

    # Subscriberとしてimage_dataというトピックに対してSubscribeし、トピックが更新されたときは
  # callbackという名前のコールバック関数を実行する
    rospy.Subscriber('Visual_ROS_node', data, callback)

    # トピック更新の待ちうけを行う関数
    rospy.spin()

if __name__ == '__main__':
    sender()