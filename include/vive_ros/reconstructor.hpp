#pragma once

#include <cmath>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <opencv/cv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;
using namespace Eigen;

class Reconstructor
{
public:
    Reconstructor();
    ~Reconstructor();
    void Run(int rate);

private:
    ros::NodeHandle nh_;
    ros::Subscriber hmd_pose;
    ros::Publisher marker_visualization_pub;
    tf::TransformListener tf_listener;
    Mat cameraMatrix, distCoeffs;
    Ptr<SURF> detector;
    std::vector<KeyPoint> keypoints_old, keypoints_new;
    Mat img_old, img_new, img_old_keypoints, img_new_keypoints;
    Mat img_old_cpy, img_new_cpy;
    VideoCapture inputCapture;
    BFMatcher matcher;
    Mat descriptors_old, descriptors_new;
    tf::StampedTransform transform_old, transform_new;
    Size image_size;
    Ptr<StereoBM> stereoBM;
    Mat map1, map2;
};