#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  public:
    // コンストラクタ
    ImageConverter() : it_(nh_){
      // カラー画像をサブスクライブ                                                                
      image_sub_ = it_.subscribe("/image_raw", 1, &ImageConverter::imageCb, this);
      // 処理した画像をパブリッシュ                                                                              
      image_pub_ = it_.advertise("/image_topic", 1);
    }

    // デストラクタ
    ~ImageConverter(){
      cv::destroyWindow(OPENCV_WINDOW);
    }
    // コールバック関数
    void imageCb(const sensor_msgs::ImageConstPtr& msg){
      cv::Point2f center, p1;
      float radius;

      cv_bridge::CvImagePtr cv_ptr, cv_ptr2, cv_ptr3;
      try{
        // ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr3 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      cv::Mat hsv_image, color_mask, gray_image, bin_image, cv_image2, cv_image3;
      // RGB表色系をHSV表色系へ変換して、hsv_imageに格納
      cv::cvtColor(cv_ptr->image, hsv_image, CV_BGR2HSV);

      
      // マスク画像は指定した範囲の色に該当する要素は255(8ビットすべて1)、それ以外は0                                                      
      //cv::inRange(hsv_image, cv::Scalar(0, 0, 100, 0) , cv::Scalar(180, 45, 255, 0), color_mask);       // 白
      //cv::inRange(hsv_image, cv::Scalar(150, 100, 50, 0) , cv::Scalar(180, 255, 255, 0), color_mask);   // 赤
      cv::inRange(hsv_image, cv::Scalar(20, 50, 50, 0) , cv::Scalar(100, 255, 255, 0), color_mask);   // 黄

      
      cv::bitwise_and(cv_ptr->image, cv_ptr->image, cv_image2, color_mask);
      // グレースケールに変換
      cv::cvtColor(cv_image2, gray_image, CV_BGR2GRAY);
      // 閾値70で2値画像に変換
      cv::threshold(gray_image, bin_image, 80, 255, CV_THRESH_BINARY); 


      // 輪郭を格納するcontoursにfindContours関数に渡すと輪郭を点の集合として入れてくれる
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(bin_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);    // 輪郭線を格納


      try{
        // 各輪郭をcontourArea関数に渡し、最大面積を持つ輪郭を探す
        double max_area=0;
        int max_area_contour=-1;
        for(int j=0; j<contours.size(); j++){
            double area = cv::contourArea(contours.at(j));
            if(max_area<area){
                max_area=area;
                max_area_contour=j;
            }
        }

        // 最大面積を持つ輪郭の最小外接円を取得
        cv::minEnclosingCircle(contours.at(max_area_contour), center, radius);
        ROS_INFO("radius = %f", radius);

        // 最小外接円を描画
        cv::circle(cv_ptr->image, center, radius, cv::Scalar(0,0,255),3,4);
        cv::circle(cv_image2, center, radius, cv::Scalar(0,0,255),3,4);
        cv::circle(bin_image, center, radius, cv::Scalar(0,0,255),3,4);
      }catch(CvErrorCallback){
        
      }

      // 画面中心から最小外接円の中心へのベクトルを描画
      p1 = cv::Point2f(cv_ptr->image.size().width/2,cv_ptr->image.size().height/2);
      cv::arrowedLine(cv_ptr->image, p1, center, cv::Scalar(0, 255, 0, 0), 3, 8, 0, 0.1);  

      // 画像サイズは縦横1/4に変更
      cv::Mat cv_half_image, cv_half_image2, cv_half_image3, cv_half_image4, cv_half_image5;
      cv::resize(cv_ptr->image, cv_half_image,cv::Size(),0.5,0.5);
      cv::resize(cv_image2, cv_half_image2,cv::Size(),0.5,0.5);
      //cv::resize(cv_ptr3->image, cv_half_image3,cv::Size(),0.5,0.5);
      cv::resize(gray_image, cv_half_image4,cv::Size(),0.5,0.5);
      cv::resize(bin_image, cv_half_image5,cv::Size(),0.5,0.5);

      // ウインドウ表示                                                                         
      cv::imshow("Original Image", cv_half_image);
      cv::imshow("Result Image", cv_half_image2);
 
      cv::imshow("Binary Image", cv_half_image5);
      cv::waitKey(3);
  
      // エッジ画像をパブリッシュ。OpenCVからROS形式にtoImageMsg()で変換。                                                            
      image_pub_.publish(cv_ptr3->toImageMsg());
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
