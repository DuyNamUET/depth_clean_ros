#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>   // include OpenCV API
#include <opencv2/rgbd.hpp>     // openCV RGBD Contrib package
#include <opencv2/highgui/highgui_c.h> // openCV High-level GUI

#define SCALE_FACTOR 1
#include <iostream>
using namespace cv;

cv_bridge::CvImage cv_br;

// apply colormap and show on screen
void showOnScreen(const Mat& img_16uc1, const char name[])
{
    Mat show_img = Mat(img_16uc1.size(), CV_8U);

    img_16uc1.convertTo(show_img, CV_8U, (float)255/65535);
    convertScaleAbs(show_img, show_img, 8.0);
    applyColorMap(show_img, show_img, COLORMAP_JET);
    imshow(name, show_img);
    waitKey(1);
    return;
}

// callback from topic and convert Mat openCV
void alignedDepthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_br = *cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    return;
}

// convert Mat openCV to ROS message
sensor_msgs::Image convert2ROSMsg(const Mat& depth_input, const cv_bridge::CvImage cv_img)
{
    cv_bridge::CvImage out_bridge;

    out_bridge.header.frame_id = cv_img.header.frame_id;
    out_bridge.header.seq = cv_img.header.seq;
    out_bridge.header.stamp = cv_img.header.stamp;

    out_bridge.image = depth_input;

    out_bridge.encoding = sensor_msgs::image_encodings::TYPE_16UC1;

    sensor_msgs::Image msg = *out_bridge.toImageMsg();
    return msg;
}

void cleanDepth(Mat& clean_depth)
{
    Mat input_depth = Mat(clean_depth.size(), CV_8U);
    clean_depth.convertTo(input_depth, CV_8U, 255.0/65535);
    const unsigned char no_depth = 0;
    Mat temp, temp2;
    Mat small_depthf;

    resize(input_depth, small_depthf, input_depth.size(),
            SCALE_FACTOR, SCALE_FACTOR); 

    // inpaint only the masked "unknown" pixels
    // input inpaint function is CV_8U type
    inpaint(small_depthf, (small_depthf == no_depth), temp, 5.0, INPAINT_TELEA);

    resize(temp, temp2, temp.size());
    temp2.copyTo(input_depth, (input_depth == no_depth));
    input_depth.convertTo(clean_depth, CV_16U, 65535.0/255);
    return;
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "clean_depth");
    ros::NodeHandle nh;

    ros::Subscriber aligned_depth_sub = nh.subscribe
            ("/camera/aligned_depth_to_color/image_raw", 15, alignedDepthCallback);
    ros::Publisher clean_depth_pub = nh.advertise
            <sensor_msgs::Image>("/clean_depth", 15);
    // Create a depth cleaner instance
    rgbd::DepthCleaner* depthc = new rgbd::DepthCleaner
            (CV_16U, 7, rgbd::DepthCleaner::DEPTH_CLEANER_NIL); 
    // ros::spin();
    ros::Rate rate(30);
    while(nh.ok())
    {
        // create matrix that subscribe input topic
        Mat depth_raw = cv_br.image;

        // create matrix for depth cleaner instance for output
        Mat cleaned_depth(depth_raw.size(), CV_16U);

        if(depth_raw.size().height == 0 || depth_raw.size().width == 0)
        {
            ros::spinOnce();
            continue;
        }

        // run cleaner instance
        depthc->operator()(depth_raw, cleaned_depth);

        //clean depth map
        cleanDepth(cleaned_depth);

        clean_depth_pub.publish(convert2ROSMsg(cleaned_depth, cv_br)); // publish topic
        
        // comment two lines below if using for next step that don't need for visualize 
        showOnScreen(depth_raw, "raw");
        showOnScreen(cleaned_depth, "cleaned");

        rate.sleep();
        ros::spinOnce();
    }
    
    destroyAllWindows();
    return 0;
}
