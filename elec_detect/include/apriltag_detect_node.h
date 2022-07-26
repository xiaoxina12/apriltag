#include <iostream>
#include <fstream>
#include <sstream>
#include "yaml-cpp/yaml.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>   //相机对齐
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <signal.h>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/types_c.h>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"
#include <eigen3/Eigen/Geometry>
#include "apriltag_ros/common_functions.h"

using namespace cv;
using namespace std;

std::string filePath = ros::package::getPath("elec_detect");
string config_path = filePath + "/config/intrinsic_calibration.yaml";


std::shared_ptr<apriltag_ros::TagDetector> tag_detector_;
cv::Mat cameraInterMat = cv::Mat(3, 3, CV_32F);
cv::Mat cameraInterMat_T= cv::Mat(3, 3, CV_32F);
cv::Mat point =  cv::Mat(3, 1, CV_32F);
cv::Mat  point_world;
void imageDepthCallback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth);
void tag_detection(const cv_bridge::CvImagePtr& cv_src);
ros::Publisher pub_corner;
ros::Subscriber sub_tag_detections;
image_transport::Subscriber img_sub;
float center_cx = 10,center_cz =10,center_wx=10;
apriltag_ros::AprilTagDetectionArray result;
