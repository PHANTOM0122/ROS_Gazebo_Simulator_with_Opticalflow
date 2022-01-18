#include <iostream>
#include <sys/stat.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <algorithm>

using namespace cv;
using namespace std;

int counter = 0;
Mat prvs;
char c;
double angular;
double linear;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	        if (counter == 0){
			Mat frame1;
		    	frame1 = cv_bridge::toCvCopy(msg, "bgr8")->image;
		    	cvtColor(frame1, prvs, COLOR_BGR2GRAY);
		}

		Mat frame2, next;
		frame2 = cv_bridge::toCvCopy(msg, "bgr8")->image;
                
		if (frame2.empty())
		    return;
		// Mission #1
		imshow("mission1", frame2);

		cvtColor(frame2, next, COLOR_BGR2GRAY);
		Mat flow(prvs.size(), CV_32FC2);
		calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
		
		// visualization
		Mat flow_parts[2];
		split(flow, flow_parts);
		Mat magnitude, angle, magn_norm;
		cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
		normalize(magnitude, magn_norm, 0.0f, 1.0f, NORM_MINMAX);
		Mat oang = angle * 3.141592 / 180.0;
		angle *= ((1.f / 360.f) * (180.f / 255.f));

		//build hsv image
		Mat _hsv[3], hsv, hsv8, bgr;
		_hsv[0] = angle;
		_hsv[1] = Mat::ones(angle.size(), CV_32F);
		_hsv[2] = magn_norm;
		merge(_hsv, 3, hsv);
		hsv.convertTo(hsv8, CV_8U, 255.0);
		cvtColor(hsv8, bgr, COLOR_HSV2BGR);


		// representation using vectors
		int step = 10;
		Mat img_vec = frame2;

		float left = 0;
		float right = 0;
		float up = 0;
		float down = 0;
		bool stop = false;
		for (int r=0; r<angle.rows; r+=step) {
			left = 0;
			right = 0;
			up = 0;
			down = 0;
			stop = false;
			Point pt_list[angle.cols];
			for (int c=0; c<angle.cols; c+=step){
				float ang = oang.at<float>(r,c);
				float m = magn_norm.at<float>(r,c) * 20.0;
				Point pt1 = cv::Point(c, r);
				Point pt2 = cv::Point(c + m * cos(ang) , r + m * sin(ang));
				Point direction = pt2 - pt1;
				pt_list[c] = direction;

				line(img_vec, pt1, pt2, Scalar(0, 255, 0), 1, 8, 0); 
			}

			for (int c=0; c<angle.cols; c+=step){
				// Left, Right
				if (pt_list[c].x > 3){
					right += pt_list[c].x;
					stop = false;
					}
				else if (pt_list[c].x < -3){
					left += pt_list[c].x;
					stop = false;
					}

				// Up, Down
				if (pt_list[c].y > 3){
					down += pt_list[c].y;
					stop = false;
					}
				else if (pt_list[c].y < -3){
					up += pt_list[c].y;
					stop = false;
					}
				if (((pt_list[c].x > -3) && (pt_list[c].x < 3)) && ((pt_list[c].y > -3) && (pt_list[c].y < 3)))
					stop = true;
				 
			}

		}
		
		if (abs(left) > abs(right)){
			
			if (abs(left) > max(abs(down), abs(up))){
					
				cout << "Left" << endl;
				angular = 0.3;
				linear = 0;
			}
		}

		else if (abs(left) < abs(right)){
	
			if (abs(right) > max(abs(down), abs(up))){
				cout << "Right" << endl;
				angular = -0.3;
				linear = 0;
			}
		}
		if (abs(up) > abs(down)){
			
			if (abs(up) > max(abs(left), abs(right))){
				cout << "Up" << endl;
				angular = 0;				
				linear = 0.1;
			}
		}
		else if (abs(up) < abs(down)){
			
			if (abs(down) > max(abs(left), abs(right))){
				cout << "Down" << endl;
				angular = 0;				
				linear = -0.1;
			}
		}

	

		imshow("mission2", img_vec);

		int keyboard = waitKey(1);
		if (keyboard == 'q' || keyboard == 27)
		      return;
		prvs = next;
		counter++;	

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_teleop");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 1, imageCallback);
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
  ros::Rate loop_rate(1000);
  
  while(ros::ok()){

  	geometry_msgs::Twist vel;
 	vel.angular.z = angular;
  	vel.linear.x = linear;
  	cout << vel << endl;
  	pub.publish(vel);

	ros::spinOnce();
	loop_rate.sleep(); 
  }		

  destroyAllWindows();
  return(0);
}




























