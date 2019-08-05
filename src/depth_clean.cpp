#include <ros/ros.h> // ros library
#include <sensor_msgs/Image.h>
#include <librealsense2/rs.hpp> // include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // include OpenCV API
#include <opencv2/rgbd.hpp>     // openCV RGBD Contrib package
#include <opencv2/highgui/highgui_c.h> // openCV High-level GUI
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <thread>
#include <atomic>
#include <queue>

#define SCALE_FACTOR 1

using namespace cv;

Mat depth_img; // initialize point that subscribe input

class QueuedMat
{
public:
    Mat img;

    // default constructor
    QueuedMat(){};

    // destructor
    ~QueuedMat()
    {
        img.release();
    };

    // copy constructor
    QueuedMat(const QueuedMat& src)
    {
        src.img.copyTo(img);
    };
};

void depthHistogram(const Mat& depth, Mat& normalize_depth, int coloring_method)
{
    normalize_depth = Mat(depth.size(), CV_8U);
    int width = depth.cols;
    int height = depth.rows;

    static uint32_t hist[0x10000];
    memset(hist, 0, sizeof(hist));

    for(int i = 0; i < height; ++i)
    {
        for(int j = 0; j < width; ++j)
        {
            ++hist[depth.at<ushort>(i, j)];
        }
    }

    // build a cumulative histogram
    for(int i = 2; i < 0x10000; ++i)
    {
        hist[i] += hist[i - 1];
    }

    // 0 - 255
    for(int i = 0; i < height; ++i)
    {
        for(int j = 0; j < width; ++j)
        {
            if(uint16_t d = depth.at<ushort>(i, j))
            {
                int f = hist[d] / hist[0xFFFF] * 255;
                normalize_depth.at<u_char>(i, j) = static_cast<u_char>(f);
            }
            else
            {
                normalize_depth.at<u_char>(i, j) = 0;
            }
        }
    }

    // apply to colormap
    applyColorMap(normalize_depth, normalize_depth, coloring_method);
    return;
}

void alignedDepthCallback(sensor_msgs::Image::ConstPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr =
            cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    depth_img = cv_ptr->image;
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "clean_depth");
    ros::NodeHandle nh;

    ros::Subscriber aligned_depth_sub = nh.subscribe(
            "/camera/aligned_depth_to_color/image_raw", 30, &alignedDepthCallback);
    ros::Publisher clean_depth_pub = nh.advertise<sensor_msgs::Image>(
            "/clean_depth", 30);
    ros::Rate rate(30);
    ros::spin();

    rgbd::DepthCleaner* depthc =
            new rgbd::DepthCleaner(CV_16U, 7, rgbd::DepthCleaner::DEPTH_CLEANER_NIL);
    rs2::colorizer color_map;

    // initialize queue
    std::queue<QueuedMat> filtered_queue;
    std::queue<QueuedMat> original_queue;

    // atomic boolean
    std::atomic_bool stopped(false);
    
    // threaded processing
    std::thread processing_thread([&]()
    {
        while(!stopped)
        {
            const int w = depth_img.cols;
            const int h = depth_img.rows;

            QueuedMat depth_queue_mat;
            QueuedMat clean_queue_mat;

            // create an OpenCV matrix for Depth Cleaner for output
            Mat clean_depth(Size(w, h), CV_16U);

            // run the RGBD depth cleaner instance
            depthc->operator()(depth_img, clean_depth);

            const unsigned char no_depth = 0; // change to 255 if values no depth uses max value

            Mat temp, temp2;

            Mat small_depthf;
            resize(clean_depth, small_depthf, Size(), SCALE_FACTOR, SCALE_FACTOR);

            // inpaint only the masked "unknown" pixels
            inpaint(small_depthf, (small_depthf == no_depth), temp, 5.0, INPAINT_TELEA);

            // upscale to original size and replace inpainted regions in original depth image
            resize(temp, temp2, clean_depth.size());
            temp2.copyTo(clean_depth, (clean_depth == no_depth)); // add to the original signal

            // copy the cleaned mat if the isDepthCleaning is true
            clean_queue_mat.img = clean_depth;

            // fill the original depth coming in from the sensor
            depth_queue_mat.img = depth_img;

            // push the mats to queue
            original_queue.push(depth_queue_mat);
            filtered_queue.push(clean_queue_mat);
        }
    });

    Mat filtered_dequeue_mat(Size(1280, 720), CV_16UC1);
    Mat original_dequeue_mat(Size(1280, 720), CV_8UC3);

    // main thread function
    while(nh.ok())
    {
        while(!original_queue.empty())
        {
            original_queue.front().img.copyTo(filtered_dequeue_mat);
            original_queue.pop();
        }

        while(!filtered_queue.empty())
        {
            filtered_queue.front().img.copyTo(filtered_dequeue_mat);
            filtered_queue.pop();
        }

        Mat colored_cleaned_depth;
        Mat colored_original_depth;

        depthHistogram(filtered_dequeue_mat, colored_cleaned_depth, COLORMAP_JET);
        depthHistogram(original_dequeue_mat, colored_original_depth, COLORMAP_JET);
        imshow("cleaned", colored_cleaned_depth);
        imshow("origin", colored_original_depth);
    }

    stopped = true;
    processing_thread.join();
    return 0;
}