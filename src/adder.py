#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity
import rospy
from Visual_ROS.msg import data

# Subscribeする対象のトピックが更新されたら呼び出されるコールバック関数
# 引数にはトピックにPublishされるメッセージの型と同じ型を定義する
def callback(data):
    # 受けとったmessageの中身を足し算して出力
    print data.arg_x + data.arg_y

def adder():
    rospy.init_node('adder', anonymous=True)

    # Subscriberとしてimage_dataというトピックに対してSubscribeし、トピックが更新されたときは
  # callbackという名前のコールバック関数を実行する
    rospy.Subscriber('input_data', data, callback)

    # トピック更新の待ちうけを行う関数
    rospy.spin()

if __name__ == '__main__':
    adder()