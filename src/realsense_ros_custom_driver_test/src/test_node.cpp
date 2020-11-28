#include "realsense_ros_custom_driver/img_stream.h"
#include "iostream"
#include "ros/ros.h"
#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"
#include <signal.h>

using namespace cv;
using namespace std;

const char* image_window = "Source Image";
const char* depth_window = "depth_window";

class SubscribeAndPublish
{
private:

    ros::NodeHandle nh_;
    ros::Subscriber img_stream_sub_;
    ros::Publisher processed_img_stream_pub_;
    rs2_intrinsics intr_;
    Mat x_dist_origin_;
    Mat y_dist_origin_;

public:

    SubscribeAndPublish(int height, int width, double ppx, double ppy, double fx, double fy):
    x_dist_origin_(height,width,CV_64F),
    y_dist_origin_(height,width,CV_64F)
    {
        //Topic you want to subscribe to.
        img_stream_sub_ = nh_.subscribe("/img_stream", 10, &SubscribeAndPublish::img_stream_callback, this);

        //Topic you want to publish.
        processed_img_stream_pub_ = nh_.advertise<realsense_ros_custom_driver::img_stream>("/processed_img_stream", 1);

        // Calculate x and y distance matrices origins.
        for(int row=0;row<height;row++)
        {
            for(int col=0;col<width;col++)
            {
            x_dist_origin_.at<double>(row,col) = (double(col)- ppx)/fx;
            y_dist_origin_.at<double>(row,col) = (double(row)-ppy)/fy;
            }
        }

    }

    void img_stream_callback(const realsense_ros_custom_driver::img_stream& msg)
    {
        // Retrieve intrinsics from recieved messege.
        rs2_intrinsics intr;
        intr.width = msg.depth_width;
        intr.height = msg.depth_height;
        intr.ppx = msg.ppx;
        intr.ppy = msg.ppy;
        intr.fx = msg.fx;
        intr.fy = msg.fy;
        intr.model = rs2_distortion(msg.model);
        for(int i=0;i<5;i++){intr.coeffs[i] = msg.coeffs[i];}

        // Construct depth and color images from the 1D arrays recieved in the messege.
        Mat depth_in_meters(msg.depth_height,msg.depth_width,CV_64F,(void*)msg.depth.data());
        Mat color_img(msg.color_height,msg.color_width,CV_8UC3,(void*)msg.bgr.data());
        Mat depth = depth * msg.depth_scale;

        // Calculate x and y distance matrices in meters.
        Mat x_dist = x_dist_origin_.mul(depth), y_dist = y_dist_origin_.mul(depth);

        // Show The color_img.
        imshow(image_window, color_img);
        waitKey(1);

    }
};

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char *argv[])
{

    namedWindow(image_window);
    //ROS specifics:
    //----------------
    //Initializing ROS node.
    ros::init(argc, argv,"test_node", ros::init_options::NoSigintHandler);
    //Create an object of class SubscribeAndPublish that will take care of everything.
    double ppx = 211.015, fx = 215.523, ppy = 120.4, fy = 215.523;
    //  SubscribeAndPublish SAPObject(480,848, ppx, ppy, fx, fy);
    SubscribeAndPublish SAPObject(240, 424, ppx, ppy, fx, fy);

    signal(SIGINT, mySigintHandler);

    ros::spin();

    return EXIT_SUCCESS;
}