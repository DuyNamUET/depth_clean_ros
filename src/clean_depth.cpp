#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <librealsense2/rs.hpp> // include RealSense Cross Platform API

#include <opencv2/opencv.hpp>   // include OpenCV API
#include <opencv2/rgbd.hpp>     // openCV RGBD Contrib package
#include <opencv2/highgui/highgui_c.h> // openCV High-level GUI

#define SCALE_FACTOR 1
#include <iostream>
using namespace cv;

Mat depth_raw;

// appl colormap and show on screen
void showOnScreen(const Mat& img_16uc1, const char name[])
{
    Mat show_img = Mat(img_16uc1.size(), CV_8U);
    img_16uc1.convertTo(show_img, CV_8U, (float)255/65535);
    applyColorMap(show_img, show_img, COLORMAP_JET);
    imshow(name, show_img);
    waitKey(1);
    return;
}

void alignedDepthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr =
            cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    depth_raw = cv_ptr->image;
    
    showOnScreen(depth_raw, "raw");
    return;
}

sensor_msgs::Image convert2ROSMsg(const Mat& depth_input)
{
    cv_bridge::CvImage out_bridge;
    out_bridge.image = depth_input;
    out_bridge.encoding = sensor_msgs::image_encodings::TYPE_16UC1;

    sensor_msgs::Image msg = *out_bridge.toImageMsg();
    return msg;
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "clean_depth");
    ros::NodeHandle nh;

    ros::Subscriber aligned_depth_sub = nh.subscribe
            ("/camera/aligned_depth_to_color/image_raw", 30, alignedDepthCallback);
    ros::Publisher clean_depth_pub = nh.advertise
            <sensor_msgs::Image>("/clean_depth", 30);

    //Create a depth cleaner instance
    rgbd::DepthCleaner* depthc = new rgbd::DepthCleaner
            (CV_16U, 7, rgbd::DepthCleaner::DEPTH_CLEANER_NIL);
    
    ros::Rate rate(30);
    while(nh.ok())
    {
        // create matrix for depth cleaner instance for output
        Mat cleaned_depth(depth_raw.size(), CV_8U);

        if (depth_raw.size().height <= 0 || depth_raw.size().width <= 0) {
            ros::spinOnce();
            continue;
        }
        // run cleaner instance
        depth_raw.convertTo(depth_raw, CV_8UC1);
        depthc->operator()(depth_raw, cleaned_depth);
        
        const unsigned char no_depth = 0; // change to 255, if values no_depth uses max value
        Mat temp, temp2;

        Mat small_depthf;

        resize(cleaned_depth, small_depthf, depth_raw.size(), SCALE_FACTOR, SCALE_FACTOR);
        // Inpaint only the masked "unknown" pixels
        inpaint(small_depthf, (small_depthf == no_depth), temp, 5.0, INPAINT_TELEA);

        resize(temp, temp2, cleaned_depth.size());
        temp2.copyTo(cleaned_depth, (cleaned_depth == no_depth));

        clean_depth_pub.publish(convert2ROSMsg(cleaned_depth));
        
        showOnScreen(cleaned_depth, "cleaned");

        rate.sleep();
        ros::spinOnce();
    }
    
    
    destroyAllWindows();

    return 0;
}