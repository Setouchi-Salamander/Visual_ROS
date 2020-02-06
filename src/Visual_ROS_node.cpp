
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <opencv2/opencv.hpp>

#include <Visual_ROS/data.h>

#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))

/*  OpenCVの名前空間を使用する  */
using namespace cv;
using namespace std;

// 共通変数
static int NumOfImg; // z方向の画像枚数
static int ImgX;
static int ImgY;
static char file_name[256];
static int base_mode = 0;//機体条件
static int min_light = 30;//最小ライトバーの長さ

static clock_t start;
static clock_t time_end;

static int min_light_delta_h = 12;
static int max_light_delta_h = 450;
static int max_light_delta_v = 50;
static float max_lr_rate = 1.5;
static float max_wh_ratio = 5.2;
static float min_wh_ratio = 1.25;
static float small_armor_wh_threshold = 3.6;
static int bin_cls_thres = 166;
static int target_max_angle = 20;


int R_LOW = 0;
int R_HIGH = 255;
int G_LOW = 0;
int G_HIGH = 255;
int B_LOW = 253;
int B_HIGH = 255;
int Gray_LOW = 190;
int Gray_HIGH = 255;

Mat Org_Img;//元画像
Mat Org_Img1;//元画像
Mat Hsv_Img;//HSV系に変更
Mat Rgb_Img;//RGB系
Mat Mask_Img;//mask後の画像
Mat Mask_Img2;//mask後の画像2
Mat Wise_Img;//論理積
Mat Gray_Img;//グレースケール
Mat Label_Img;//ラベル付け用
Mat Bin_Img;//二値画像(フィルタ後)
Mat Bin_Img2;//二値画像
Mat Result_Img;//ラベル付け画像
Mat stats;
Mat centroids;


cv::Point2f center,center2, result_pt;
cv_bridge::CvImagePtr cv_ptr, cv_ptr2, cv_ptr3;

static const std::string OPENCV_WINDOW = "Image window";

//
struct matched_rect {
	cv::RotatedRect rect;
	float lr_rate;
	float angle_abs;

};

cv::RotatedRect boundingRRect(const cv::RotatedRect& left, const cv::RotatedRect& right) {
	// この関数は、角度を考慮せずに、左右のライトバーをターゲットの回転する四角形に合わせるために使用されます
	const Point& pl = left.center, & pr = right.center;
	Point2f center = (pl + pr) / 2.0;
	//    cv::Size2f wh_l = left.size;
	//    cv::Size2f wh_r = right.size;
		// 这里的目标矩形的height是之前灯柱的width
	double width_l = MIN(left.size.width, left.size.height);
	double width_r = MIN(right.size.width, right.size.height);
	double height_l = MAX(left.size.width, left.size.height);
	double height_r = MAX(right.size.width, right.size.height);
	float width = POINT_DIST(pl, pr) - (width_l + width_r) / 2.0;
	float height = std::max(height_l, height_r);
	//float height = (wh_l.height + wh_r.height) / 2.0;
	float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
	return RotatedRect(center, Size2f(width, height), angle * 180 / CV_PI);
}


class Lockon{

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::NodeHandle n;
  ros::Publisher para_pub = n.advertise<Visual_ROS::data>("Visual_ROS_node", 1000);

  Visual_ROS::data msg_data;
  int count = 0;
    
    public:
    // コンストラクタ
    Lockon() : it_(nh_){

    printf("start \n");
    
      // カラー画像をサブスクライブ                                                                
      image_sub_ = it_.subscribe("/image_raw", 1, &Lockon::imageLock, this);
      // 処理した画像をパブリッシュ                                                                                          
      image_pub_ = it_.advertise("/image_topic", 1);

          
    }

    // デストラクタ
    ~Lockon(){
      cv::destroyWindow(OPENCV_WINDOW);
    }

   // コールバック関数
	void imageLock(const sensor_msgs::ImageConstPtr& msg)
	{

    		  try{
    		    // ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
   		     	 cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    		   	 cv_ptr3 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
     		  }catch (cv_bridge::Exception& e){
        		ROS_ERROR("cv_bridge exception: %s", e.what());
       			return;
    		  }


		//ROS画像
		Org_Img = cv_ptr->image;
	
		Mat element2 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
	

		//HSV
		//cv::cvtColor(Org_Img,Hsv_Img,COLOR_BGR2HSV);

		//RGB
		cv::cvtColor(Org_Img, Rgb_Img, COLOR_BGR2RGB);
		
		//RGB(GRAY)
		cv::cvtColor(Org_Img, Gray_Img, COLOR_BGR2GRAY);
    
	
		//マスク処理（赤）
		// cv::inRange(Hsv_Img, cv::Scalar(150, 100, 180, 0), 			cv::Scalar(180, 255, 255, 0), Mask_Img);
		// cv::inRange(Hsv_Img, cv::Scalar(0, 0, 180, 0), cv::Scalar(30, 255, 255, 0), Mask_Img2);
	
		//cv::inRange(Rgb_Img, cv::Scalar(255, 0, 0, 0), cv::Scalar(255, 255, 255, 0), Mask_Img);

		//マスク処理（青）
		//cv::inRange(Hsv_Img, cv::Scalar(100, 200, 220, 0),cv::Scalar(120, 255, 255, 0), Mask_Img);
		cv::inRange(Rgb_Img, cv::Scalar(R_LOW, G_LOW, B_LOW, 0), cv::Scalar(R_HIGH, G_HIGH, B_HIGH, 0), Mask_Img);
	
		//GRAYの2値化
		threshold(Gray_Img, Mask_Img2, Gray_LOW, Gray_HIGH, THRESH_BINARY);
	
		bitwise_and(Mask_Img, Mask_Img2,Wise_Img);
	
		// Bin_Img2 = Mask_Img + Mask_Img2;//HSV
		Bin_Img2 = Wise_Img;//RGB

		dilate(Bin_Img2, Bin_Img2, element2);
	
		//中央値フィルタ
		//cv::GaussianBlur(Bin_Img2, Bin_Img ,Size(3,3),0);
	
		// 輪郭を格納するcontoursにfindContours関数に渡すと輪郭を点の集合として入れてくれる
		std::vector<std::vector<cv::Point>> contours;

		cv::findContours(Bin_Img2, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);    // 輪郭線を格納

	
		try {
	
			// 輪郭を直線で近似して、勾配範囲に適合する輪郭を見つけます
			std::vector<RotatedRect> RectFirstResult;
			for (size_t i = 0; i < contours.size(); ++i) {
				// fit the lamp contour as a eclipse
				RotatedRect rrect = minAreaRect(contours[i]);
				double max_rrect_len = MAX(rrect.size.width, rrect.size.height);
				double min_rrect_len = MIN(rrect.size.width, rrect.size.height);
	
				/////////////////////////////// 単一のライトバーの条件 //////////////////////////////////////
				// 実際の状況に応じて、角度をわずかに変更する
				bool if1 = (fabs(rrect.angle) < 45.0 && rrect.size.height > rrect.size.width); // 左
				bool if2 = (fabs(rrect.angle) > 60.0 && rrect.size.width > rrect.size.height); // 右
				bool if3 = max_rrect_len > min_light; // ライトバーの最小長
				bool if4;
				if (!base_mode) // ベースを吊るすときの異なる条件
					if4 = (max_rrect_len / min_rrect_len >= 1.1) && (max_rrect_len / min_rrect_len < 15); // ライトバーのアスペクト比
				else
					if4 = (max_rrect_len / min_rrect_len >= 9.9) && (max_rrect_len / min_rrect_len < 30); // ライトバーのアスペクト比
				// 水平で小さすぎる回転長方形をふるいにかけます（元は45で、後で60を追加）
				if ((if1 || if2) && if3 && if4)
				{
					RectFirstResult.push_back(rrect);
					Point2f vertice[4];
					rrect.points(vertice);
					for (int i = 0; i < 4; i++)  // 黄色
						line(Org_Img, vertice[i], vertice[(i + 1) % 4], Scalar(0, 255, 255), 2);
				}
			}

		

			// 2本未満のライトバーは一致しないと見なされます
			if (RectFirstResult.size() < 2) {
				printf("nothing \n");
				result_pt.x = -Org_Img.cols/2;
				result_pt.y = -Org_Img.rows/2;
<<<<<<< HEAD
					
				msg_data.cnt_taget=0;
=======
       				 goto result;
>>>>>>> develop
			}
			else{

			// 回転した長方形を左から右に並べ替えます
			sort(RectFirstResult.begin(), RectFirstResult.end(),
				[](RotatedRect& a1, RotatedRect& a2) {
				return a1.center.x < a2.center.x; });

			Point2f _pt[4], pt[4];
			auto ptangle = [](const Point2f& p1, const Point2f& p2) {
				return fabs(atan2(p2.y - p1.y, p2.x - p1.x) * 180.0 / CV_PI);
			};

		///////////////////////////////////// 一致するライトバーの条件 //////////////////////////////////////////////////////
		//それらを1つずつ比較し、要件を満たしている場合は、ターゲット回転長方形を形成します。
			for (size_t i = 0; i < RectFirstResult.size() - 1; ++i) {

				const RotatedRect& rect_i = RectFirstResult[i];

				const Point& center_i = rect_i.center;

				float xi = center_i.x;
				float yi = center_i.y;
				float leni = MAX(rect_i.size.width, rect_i.size.height);
				float anglei = fabs(rect_i.angle);

				rect_i.points(_pt);

				//pt
				// 0 2
				// 1 3
				 
				 // 右に傾いた長いライトバー
				 // rRect.points注文済み，y最小のポイントは0,時計回り1 2 3
				if (anglei > 45.0) {
					pt[0] = _pt[3];
					pt[1] = _pt[0];
				}
				//左に傾いた
				else {
					pt[0] = _pt[2];
					pt[1] = _pt[3];
				}
	
				for (size_t j = i + 1; j < RectFirstResult.size(); j++) {
					const RotatedRect& rect_j = RectFirstResult[j];
					const Point& center_j = rect_j.center;
					float xj = center_j.x;
					float yj = center_j.y;
					float lenj = MAX(rect_j.size.width, rect_j.size.height);
					float anglej = fabs(rect_j.angle);
					float delta_h = xj - xi;
					float lr_rate = leni > lenj ? leni / lenj : lenj / leni;
					float angleabs;
	
	
					rect_j.points(_pt);
					if (anglej > 45.0) {
						pt[2] = _pt[2];
						pt[3] = _pt[1];
					}
					else {
						pt[2] = _pt[1];
						pt[3] = _pt[0];
					}
					double maxangle = MAX(ptangle(pt[0], pt[2]), ptangle(pt[1], pt[3]));
					//std::cout<<"angle:"<<maxangle<<std::endl;
				   // maxangle = 0;
					if (anglei > 45.0 && anglej < 45.0) { // 八字 / \   //
						angleabs = 90.0 - anglei + anglej;
					}
					else if (anglei <= 45.0 && anglej >= 45.0) { // 倒八字 \ /
						angleabs = 90.0 - anglej + anglei;
					}
					else {
						if (anglei > anglej) angleabs = anglei - anglej; // 在同一边
						else angleabs = anglej - anglei;
					}
	
	
					// if rectangle is m atch condition, put it in candidate vector
					// lr_rate1.3それは少し小さいですが、壊れた3号車に対処するために増やすことができます	
					bool condition1 = delta_h >min_light_delta_h && delta_h < max_light_delta_h;
					bool condition2 = MAX(leni, lenj) >= 113 ? abs(yi - yj) < 166\
						&& abs(yi - yj) < 1.66 * MAX(leni, lenj) :
						abs(yi - yj) < max_light_delta_v\
						&& abs(yi - yj) < 1.2 * MAX(leni, lenj); // && abs(yi - yj) < MIN(leni, lenj)
					bool condition3 = lr_rate < max_lr_rate;
					//                bool condition4 = angleabs < 15 ; // 動きを防止するための大規模なフレームを指摘
					bool condition4;

					Point2f text_center = Point2f((xi + xj) / 2, (yi + yj) / 2);



					if (condition1 && condition2 && condition3) {
						RotatedRect obj_rect = boundingRRect(rect_i, rect_j);
						double w = obj_rect.size.width;
						double h = obj_rect.size.height;
						double wh_ratio = w / h;
	
						// 基本モードはアスペクト比に制限されません
						if (!base_mode) {
							if (wh_ratio > max_wh_ratio || wh_ratio < min_wh_ratio)
								continue;
						}
	

						//予備的に一致した構造情報をベクトルvectorにプッシュします
						//match_rects.push_back(matched_rect{ obj_rect, lr_rate, angleabs });
						// for debug use
						Point2f vertice[4];
						obj_rect.points(vertice);
						for (int i = 0; i < 4; i++)
							line(Org_Img, vertice[i], vertice[(i + 1) % 4], Scalar(255, 255, 255), 2);
						cv::circle(Org_Img, text_center, 4, Scalar(0, 255, 0), 2);
						result_pt = Point2f((xi + xj) / 2, (yi + yj) / 2);
					}
				}
			}

			}

		}
		catch (ErrorCallback) {

		}

	
	
	
		//結果表示
		cv::namedWindow("Result",WINDOW_AUTOSIZE|WINDOW_FREERATIO);

		cv::createTrackbar("R_LOW", "Result", &R_LOW, 255);
		cv::createTrackbar("R_HIGH", "Result", &R_HIGH, 255);
		cv::createTrackbar("G_LOW", "Result", &G_LOW, 255);
		cv::createTrackbar("G_HIGH", "Result", &G_HIGH, 255);
		cv::createTrackbar("B_LOW", "Result", &B_LOW, 255);
		cv::createTrackbar("B_HIGH", "Result", &B_HIGH, 255);
		cv::createTrackbar("Gray_LOW", "Result", &Gray_LOW, 255);
		cv::createTrackbar("Gray_HIGH", "Result", &Gray_HIGH, 255);

		R_HIGH = cv::getTrackbarPos("R_HIGH","Result");
		B_HIGH = cv::getTrackbarPos("B_HIGH","Result");

		cv::imshow("Result", Org_Img);

		//cv::imshow("Result2", Bin_Img2);
		cv::waitKey(1);


    
    msg_data.x = result_pt.x - Org_Img.cols/2;
    msg_data.y = -(result_pt.y - Org_Img.rows/2);
<<<<<<< HEAD
    printf("x = %d y = %d count=%d \n",msg_data.x , msg_data.y,msg_data.cnt_taget );
=======
    count++;
    printf("x = %d y = %d \n",msg_data.x , msg_data.y );
>>>>>>> develop
		// エッジ画像をパブリッシュ。OpenCVからROS形式にtoImageMsg()で変換。                                                        
    image_pub_.publish(cv_ptr3->toImageMsg());

    para_pub.publish(msg_data);//PublishのAPI
	}

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Visual_ROS_node");
  Lockon ic;
  ros::spin();
  return 0;
}
