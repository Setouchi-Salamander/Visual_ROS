#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# 自分で定義したmessageファイルから生成されたモジュール
from Visual_ROS.msg import data

def para_in():
    # 初期化宣言 : このソフトウェアは"para_in"という名前
    rospy.init_node('para_in', anonymous=True)

    # nodeの宣言 : publisherのインスタンスを作る
    # input_dataというtopicにAdder型のmessageを送るPublisherをつくった
    pub = rospy.Publisher('input_data', data, queue_size=100)

    # 1秒間にpublishする数の設定
    r = rospy.Rate(5)

    para_x = 0
    para_y = 2

    # Adder型のmessageのインスタンスを作る
    msg = data()

    # ctl +　Cで終了しない限りwhileループでpublishし続ける

    while not rospy.is_shutdown():

        msg.arg_x = para_x
        msg.arg_y = para_y

        # publishする関数
        pub.publish(msg)
        print "published arg_x=%d arg_y=%d"%(msg.arg_x,msg.arg_y)
        para_x += 1
        para_y += 1

        r.sleep()

if __name__ == '__main__':
    try:
            para_in()

    except rospy.ROSInterruptException: pass