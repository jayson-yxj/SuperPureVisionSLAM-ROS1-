/**
* This file is part of ORB-SLAM3
* Modified to support dynamic object masking
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<mutex>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include "monster/ImagePoseStereo.h"

using namespace std;

#define ZED_LEFT "/zedm/zed_node/left/image_rect_color"
#define ZED_RIGHT "/zedm/zed_node/right/image_rect_color"

#define VIDEO_LEFT "/stereo/raw_left"
#define VIDEO_RIGHT "/stereo/raw_right"

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ros::NodeHandle* nh):mpSLAM(pSLAM),node_handle(nh)
    {
        image_pose_stereo_pub = node_handle->advertise<monster::ImagePoseStereo>("/orb_slam3/image_pose_stereo", 10);
        
        // 订阅动态物体掩码
        mask_sub = node_handle->subscribe("/dynamic_mask", 1, &ImageGrabber::MaskCallback, this);
        
        // 初始化
        mbUseDynamicMask = false;
        mbMaskReceived = false;
        
        ROS_INFO("ImageGrabber: Subscribed to /dynamic_mask for dynamic object filtering");
    }

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
    void MaskCallback(const sensor_msgs::ImageConstPtr& msgMask);
    void PublishImagePoseStereo(const cv_bridge::CvImageConstPtr& cv_ptrLeft,
                                const cv_bridge::CvImageConstPtr& cv_ptrRight,
                                const Sophus::SE3f& Tcw,
                                bool tracking_success);

    ORB_SLAM3::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    Sophus::SE3f Tcw;
    
    // 动态掩码相关
    cv::Mat mDynamicMask;
    bool mbUseDynamicMask;
    bool mbMaskReceived;
    std::mutex mMutexMask;

private:
    ros::NodeHandle* node_handle;
    ros::Publisher image_pose_stereo_pub;
    ros::Subscriber mask_sub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);

    ros::NodeHandle nodeHandler;
    ImageGrabber igb(&SLAM, &nodeHandler);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;
        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;
        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;
        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, VIDEO_LEFT, 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, VIDEO_RIGHT, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();
    return 0;
}

void ImageGrabber::MaskCallback(const sensor_msgs::ImageConstPtr& msgMask)
{
    try {
        cv_bridge::CvImageConstPtr cv_ptrMask = cv_bridge::toCvShare(msgMask, sensor_msgs::image_encodings::MONO8);
        
        std::lock_guard<std::mutex> lock(mMutexMask);
        mDynamicMask = cv_ptrMask->image.clone();
        mbMaskReceived = true;
        
        if (!mbUseDynamicMask) {
            mbUseDynamicMask = true;
            ROS_INFO("Dynamic mask received and enabled for object filtering");
        }
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Mask callback exception: %s", e.what());
    }
}

void ImageGrabber::PublishImagePoseStereo(const cv_bridge::CvImageConstPtr& cv_ptrLeft,
                                          const cv_bridge::CvImageConstPtr& cv_ptrRight,
                                          const Sophus::SE3f& Tcw,
                                          bool tracking_success)
{
    monster::ImagePoseStereo image_pose_stereo_msg;
    image_pose_stereo_msg.header.stamp = cv_ptrLeft->header.stamp;
    image_pose_stereo_msg.header.frame_id = "camera";

    cv::Mat left_img_copy = cv_ptrLeft->image.clone();
    cv::Mat right_img_copy = cv_ptrRight->image.clone();
    
    if (left_img_copy.channels() == 1) {
        cv::cvtColor(left_img_copy, left_img_copy, cv::COLOR_GRAY2BGR);
    }
    if (right_img_copy.channels() == 1) {
        cv::cvtColor(right_img_copy, right_img_copy, cv::COLOR_GRAY2BGR);
    }

    sensor_msgs::ImagePtr img_left_msg = cv_bridge::CvImage(cv_ptrLeft->header, "bgr8", left_img_copy).toImageMsg();
    image_pose_stereo_msg.image_left = *img_left_msg;

    sensor_msgs::ImagePtr img_right_msg = cv_bridge::CvImage(cv_ptrRight->header, "bgr8", right_img_copy).toImageMsg();
    image_pose_stereo_msg.image_right = *img_right_msg;

    Eigen::Matrix3f R = Tcw.rotationMatrix();
    Eigen::Vector3f t = Tcw.translation();

    image_pose_stereo_msg.pose.position.x = t.x();
    image_pose_stereo_msg.pose.position.y = t.y();
    image_pose_stereo_msg.pose.position.z = t.z();

    Eigen::Quaternionf q(R);
    q.normalize();

    image_pose_stereo_msg.pose.orientation.x = q.x();
    image_pose_stereo_msg.pose.orientation.y = q.y();
    image_pose_stereo_msg.pose.orientation.z = q.z();
    image_pose_stereo_msg.pose.orientation.w = q.w();

    image_pose_stereo_msg.tracking_success = tracking_success;
    image_pose_stereo_pub.publish(image_pose_stereo_msg);
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // 保存原始彩色图像用于发布
    cv_bridge::CvImageConstPtr cv_ptrLeft_original;
    cv_bridge::CvImageConstPtr cv_ptrRight_original;
    try
    {
        cv_ptrLeft_original = cv_bridge::toCvCopy(msgLeft, sensor_msgs::image_encodings::BGR8);
        cv_ptrRight_original = cv_bridge::toCvCopy(msgRight, sensor_msgs::image_encodings::BGR8);
        
        static bool first_frame = true;
        if (first_frame) {
            ROS_INFO("=== Image Processing Info ===");
            ROS_INFO("Received image size: %d x %d", cv_ptrLeft_original->image.cols, cv_ptrLeft_original->image.rows);
            first_frame = false;
        }
        
        const int target_width = 640;
        const int target_height = 360;
        
        if (cv_ptrLeft_original->image.cols > target_width || cv_ptrLeft_original->image.rows > target_height) {
            cv_bridge::CvImagePtr cv_ptrLeft_resized(new cv_bridge::CvImage());
            cv_bridge::CvImagePtr cv_ptrRight_resized(new cv_bridge::CvImage());
            
            cv_ptrLeft_resized->header = cv_ptrLeft_original->header;
            cv_ptrLeft_resized->encoding = cv_ptrLeft_original->encoding;
            cv_ptrRight_resized->header = cv_ptrRight_original->header;
            cv_ptrRight_resized->encoding = cv_ptrRight_original->encoding;
            
            cv::resize(cv_ptrLeft_original->image, cv_ptrLeft_resized->image,
                      cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);
            cv::resize(cv_ptrRight_original->image, cv_ptrRight_resized->image,
                      cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);
            
            static bool first_resize = true;
            if (first_resize) {
                ROS_INFO("Image resized to: %d x %d", target_width, target_height);
                first_resize = false;
            }
            
            cv_ptrLeft_original = cv_ptrLeft_resized;
            cv_ptrRight_original = cv_ptrRight_resized;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    // 获取用于SLAM处理的图像
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    bool tracking_success = true;
    
    // 准备图像
    cv::Mat imLeft, imRight;
    if(do_rectify)
    {
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
    }
    else
    {
        imLeft = cv_ptrLeft->image.clone();
        imRight = cv_ptrRight->image.clone();
    }
    
    // 应用动态物体掩码过滤
    if(mbUseDynamicMask && mbMaskReceived)
    {
        std::lock_guard<std::mutex> lock(mMutexMask);
        if(!mDynamicMask.empty() && mDynamicMask.size() == imLeft.size())
        {
            // 掩码: 0=动态物体(过滤), 255=静态区域(保留)
            // 将动态区域设为黑色
            imLeft.setTo(0, mDynamicMask == 0);
            imRight.setTo(0, mDynamicMask == 0);
            
            static int frame_count = 0;
            if(++frame_count % 100 == 0) {
                int dynamic_pixels = cv::countNonZero(mDynamicMask == 0);
                float dynamic_ratio = 100.0f * dynamic_pixels / (mDynamicMask.rows * mDynamicMask.cols);
                ROS_INFO("Dynamic mask applied: %.1f%% pixels filtered", dynamic_ratio);
            }
        }
    }
    
    // 执行SLAM跟踪
    Tcw = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());

    if(Tcw.translation().norm() < 1e-6)
    {
        tracking_success = false;
    }

    PublishImagePoseStereo(cv_ptrLeft_original, cv_ptrRight_original, Tcw, tracking_success);
}