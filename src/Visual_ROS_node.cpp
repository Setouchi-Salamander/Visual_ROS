// ros/ros.h　ROSに関する基本的なAPIのためのヘッダ
#include "ros/ros.h"
// image_tutorial/image.h　image.msgから生成されたメッセージを定義しているヘッダ
#include "Visual_ROS/data.h"


int main(int argc, char **argv)
{
  // 初期化のためのAPI
  ros::init(argc, argv, "para_in");
  // このノードは"para_in"という名前であるという意味
  
  // ノードハンドラの宣言
  ros::NodeHandle n;




  //Publisherとしての定義
  // n.advertise<comp_tutorial::adder>("para_input", 1000);
  // comp_tutorial::adder型のメッセージをpara_inputというトピックへ配信する
  //"1000"はトピックキューの最大値
  ros::Publisher para_pub = n.advertise<Visual_ROS::data>("Visual_ROS_node", 1000);


  //1秒間に1つのメッセージをPublishする
  ros::Rate loop_rate(10);

  //comp_tutrial::adder型のオブジェクトを定義
  //adder.msgで定義したa,bはメンバ変数としてアクセスできる
  Visual_ROS::data msg;

  int count = 0;
  while (ros::ok())//ノードが実行中は基本的にros::ok()=1
  {
    msg.x = count;
    msg.y = count;
    para_pub.publish(msg);//PublishのAPI
    printf("x = %d y = %d \n",msg.x , msg.y );
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }
  return 0;
}