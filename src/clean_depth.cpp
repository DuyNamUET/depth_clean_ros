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

void alignedDepthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr =
                cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        depth_raw = cv_ptr->image;
        
        imshow("raw", depth_raw);
        waitKey(1);
    }
    catch(cv_bridge::Exception &e)
    {
        ROS_ERROR("%s", msg->encoding.c_str());
    }
    return;
}

void depthHist(const Mat& depth, Mat& normalize_depth, int method)
{
    normalize_depth = Mat(depth.size(), CV_16U);
    int width = depth.cols;
    int height = depth.rows;

    static uint32_t histogram[0x10000];
    memset(histogram, 0, sizeof(histogram));

    for(int i = 0; i < height; ++i)
    {
        for(int j = 0; j < width; ++j)
        {
            ++histogram[depth.at<ushort>(i, j)];
        }
    }

    for(int i = 2; i < 0x10000; ++i)
    {
        histogram[i] += histogram[i-1];
    }

    for(int i = 0; i < height; ++i)
    {
        for(int j = 0; j < width; ++j)
        {
            if(uint16_t d = depth.at<ushort>(i, j))
            {
                // int f = histogram[d] * 255 / histogram[0xFFFF];
                // normalize_depth.at<uchar>(i, j) = static_cast<uchar>(f);
                normalize_depth.at<ushort>(i, j) = static_cast<ushort>(histogram[d]);
            }
            else
            {
                normalize_depth.at<ushort>(i, j) = 0;
            }
        }
    }

    // apply the corlormap
    // applyColorMap(normalize_depth, normalize_depth, method);
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
        Mat cleaned_depth(depth_raw.size(), CV_16U);

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
        // cout << depth_raw.size().height;

        resize(cleaned_depth, small_depthf, depth_raw.size(), SCALE_FACTOR, SCALE_FACTOR);
        // Inpaint only the masked "unknown" pixels
        inpaint(small_depthf, (small_depthf == no_depth), temp, 5.0, INPAINT_TELEA);

        resize(temp, temp2, cleaned_depth.size());
        temp.copyTo(cleaned_depth, (cleaned_depth == no_depth));

        depthHist(cleaned_depth, cleaned_depth, COLORMAP_JET);

        clean_depth_pub.publish(convert2ROSMsg(cleaned_depth));
        imshow("", cleaned_depth);
        std::cout << cleaned_depth << std::endl;

        rate.sleep();
        ros::spinOnce();
    }
    
    
    destroyAllWindows();

    return 0;
}