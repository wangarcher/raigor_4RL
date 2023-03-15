#include <string.h>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <fstream>


#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>

#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "raigor_general_perception/obj_visual_pose.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//tilt threshold 
#define QRLOCATION_INCLINATION_THRESHOLD 0.1
//debugui title set
#define QRLOCATION_DEBUGUI_TITLE "S.S.D.D"
//camera height 
#define QRLOCATION_CAM_HEIGHT 0.15

using namespace std;
using namespace zbar;
using namespace cv;


typedef struct QRPose
{
    float relateX;
    float relateY;
    float relateT;
    float qrX;
    float qrY;
    float qrT;
}
QRPose_t;




class QRLocation
{

public:

QRLocation(ros::NodeHandle &n) :
    n_(n), it_(n_)
{    

    qrt = 0.0;
    qrxy = 0;

    scanner.set_config(zbar::ZBAR_NONE,zbar::ZBAR_CFG_ENABLE,1);
    image_sub_ = it_.subscribe("/d435/r/camera/color/image_raw", 1, &QRLocation::imageCallback, this);
    depth_sub_ = it_.subscribe("/d435/r/camera/depth/image_raw", 1, &QRLocation::depthCallback, this);

    info_pub_ = n_.advertise<raigor_general_perception::obj_visual_pose>("/obj_visual_pose", 1000);
}

~QRLocation()
{ 
    cvReleaseImage(&grayFrame);
    //destroy the window  
    cvDestroyWindow("Image window");
}



void depthCallback(const sensor_msgs::ImageConstPtr& depth_img) {
    //ROS_WARN("%s", depth_img->encoding.c_str());
    cv::Mat data;
    raigor_general_perception::obj_visual_pose obj_pose;

    int u = qrxy  / 1000;
    int v = qrxy - (u * 1000);
    cout << u << v << endl;
    if (qrxy != 0)
    {
        data = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        obj_pose.obj_visual_dist = data.at<uint16_t>(u, v);
        std::cout << "depth data: " << data.at<uint16_t>(u, v) << std::endl;
    }
    obj_pose.obj_visual_u = u;
    obj_pose.obj_visual_v = v;
    obj_pose.obj_visual_t = qrt;
    info_pub_.publish(obj_pose);
    obj_pose.obj_visual_u = 0;
    obj_pose.obj_visual_v = 0;
    obj_pose.obj_visual_t = 0.0;
    obj_pose.obj_visual_dist = 0;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg_ptr, sensor_msgs::image_encodings::BGR8);
        cv::Mat img_mat = cv_ptr->image;
        IplImage media = IplImage(img_mat);
        frame = cvCloneImage(&media);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("error: image trans");
        return;
    }
        //get the img from the video stream
    if(!frame)
        ROS_ERROR("error: copy image");
    if(!grayFrame)
        grayFrame=cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U,1);
    cvCvtColor(frame,grayFrame,CV_BGR2GRAY);

    if(1)
    {
        cvShowImage("Image window",grayFrame);
        cvWaitKey(50);
    }
    //create zbar img
    Image image(frame->width,frame->height,"Y800",grayFrame->imageData,frame->width*frame->height);
    int symbolCount=scanner.scan(image);
    //get the first qrcode
    Image::SymbolIterator symbol=image.symbol_begin();



    QRPose_t qrPose;
    //qrcode integrity check
    
    if(symbol->get_type_name()!="QR-Code")
    {
        //ROS_ERROR("error: qr detection failure");
    }
    else
    {
        //get the information
        char data[128];
        strncpy(data,symbol->get_data().c_str(),sizeof(data)-1);
        data[sizeof(data)-1]=0;
        //the information has to be start with a certainly determined character sequence like "ZK"
        if(strncmp(data,"nrsl",4)!=0)
        {
            //ROS_ERROR("error: qr info mismatched");
        }
        else
        {

            //get four angular points of the qrcode
            float x0=symbol->get_location_x(0);
            float y0=symbol->get_location_y(0);
            float x1=symbol->get_location_x(1);
            float y1=symbol->get_location_y(1);
            float x2=symbol->get_location_x(2);
            float y2=symbol->get_location_y(2);
            float x3=symbol->get_location_x(3);
            float y3=symbol->get_location_y(3);
            cout << "Point A: " << x0 << ", " << y0 << "\n"
                 << "Point B: " << x1 << ", " << y1 << "\n"
                 << "Point C: " << x2 << ", " << y2 << "\n"
                 << "Point D: " << x3 << ", " << y3 <<  endl;

	        vector<Point3f> corners(4);
	        vector<Point2f> corners_trans(4);
	        corners[0] = Point3f(540, 260, 0);
            corners[1] = Point3f(740, 260, 0);
	        corners[2] = Point3f(540, 460, 0);
	        corners[3] = Point3f(740, 460, 0);
            corners_trans[0] = Point2f(x0, y0);
            corners_trans[1] = Point2f(x3, y3);
            corners_trans[2] = Point2f(x1, y1);
            corners_trans[3] = Point2f(x2, y2);
	        //Mat transform = getPerspectiveTransform(corners,corners_trans);
            //cout << transform << endl;
            //left height of the qrcode
            float yB_A=y1-y0;
            //right height of the qrcode
            float yC_D=y2-y3;
            //make sure the qrcode was well placed

            //left width of the qrcode
            float xB_A=x1-x0;
            //right width of the qrcode
            float xC_D=x2-x3;
            //make sure the qrcode was well placed

            //half view angle tangent value

            float k1 = (y2-y0)/(x2-x0);
            float b1 = y0 - k1*x0;
            float k2 = (y3-y1)/(x3-x1);
            float b2 = y1 - k2*x1;
            float crossX = (b1-b2)/(k2-k1);
            float crossY = crossX * k2 + b2;
            float centreX = grayFrame->width/2;
            float centreY = grayFrame->height/2;
            float leftT, rightT, u_x, v_y;
            if(yB_A >= 0)
            {
                if(xB_A > 0)
                {
                    leftT = atan(yB_A/xB_A) - 1.57;
                }
                if(xB_A < 0)
                {
                    leftT = atan(yB_A/xB_A) + 1.57;
                }
                if(xB_A == 0)
                {
                    leftT = 0;
                }
            }
            else    
            {
                if(xB_A > 0)
                {
                    leftT = atan(yB_A/xB_A) - 1.57;
                }
                if(xB_A < 0)
                {
                    leftT = atan(yB_A/xB_A) + 1.57;
                }
                if(xB_A == 0)
                {
                    leftT = -3.14;
                }
            }
            if(yC_D >= 0)
            {
                if(xC_D > 0)
                {
                    rightT = atan(yC_D/xC_D) - 1.57;
                }
                if(xC_D < 0)
                {
                    rightT = atan(yC_D/xC_D) + 1.57;
                }
                if(xC_D == 0)
                {
                    rightT = 0;
                }
            }
            else
            {
                if(xC_D > 0)
                {
                    rightT = atan(yC_D/xC_D) - 1.57;
                }
                if(xC_D < 0)
                {
                    rightT = atan(yC_D/xC_D) + 1.57;
                }
                if(xC_D == 0)
                {
                    rightT = -3.14;
                }
            }
            cout << "leftT: "<< leftT <<  ", rightT: " << rightT << endl;
            if(abs(leftT-rightT) > 1)
            {
                qrt = -abs(leftT);
            }
            else
            {
                qrt = (leftT + rightT) / 2;
            }
            if (isnan(crossX))
            {
                u_x = 0.0;
            }
            else
            {
                u_x = crossX;
            }
            if (isnan(crossY))
            {
                v_y = 0.0;
            }    
            else
            {
                v_y = crossY;
            }
            int media_x = (int) u_x;
            int media_y = (int) v_y;
            qrxy = media_x * 1000 + media_y;


        }
        
    }
}


protected:
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;
    ros::Publisher info_pub_;
    cv_bridge::CvImagePtr cv_ptr;

private:
    int qrxy;
    float qrt;
private:
    IplImage* frame;
    IplImage* grayFrame;
    zbar::ImageScanner scanner;



};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "qrlocation");
  ros::NodeHandle n;
  QRLocation qr(n);
  ros::spin();
  return 0;
}