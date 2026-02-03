/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

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
    }

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
    void PublishImagePoseStereo(const cv_bridge::CvImageConstPtr& cv_ptrLeft,
                                const cv_bridge::CvImageConstPtr& cv_ptrRight,
                                const Sophus::SE3f& Tcw,
                                bool tracking_success);

    ORB_SLAM3::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    Sophus::SE3f Tcw; // 指的是ORB定义的Tcw ---> camera to world

private:
    ros::NodeHandle* node_handle;
    ros::Publisher image_pose_stereo_pub;
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

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);

    ros::NodeHandle nodeHandler;
    ImageGrabber igb(&SLAM, &nodeHandler);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
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

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::PublishImagePoseStereo(const cv_bridge::CvImageConstPtr& cv_ptrLeft,
                                          const cv_bridge::CvImageConstPtr& cv_ptrRight,
                                          const Sophus::SE3f& Tcw,
                                          bool tracking_success)
{
    // 创建自定义消息类型
    monster::ImagePoseStereo image_pose_stereo_msg;

    // 设置时间戳用于同步
    image_pose_stereo_msg.header.stamp = cv_ptrLeft->header.stamp;
    image_pose_stereo_msg.header.frame_id = "camera";

    // 显式复制图像数据以避免内存共享问题
    cv::Mat left_img_copy = cv_ptrLeft->image.clone();
    cv::Mat right_img_copy = cv_ptrRight->image.clone();
    
    // 确保图像是 BGR8 格式
    if (left_img_copy.channels() == 1) {
        cv::cvtColor(left_img_copy, left_img_copy, cv::COLOR_GRAY2BGR);
    }
    if (right_img_copy.channels() == 1) {
        cv::cvtColor(right_img_copy, right_img_copy, cv::COLOR_GRAY2BGR);
    }

    // 左图像转换 - 使用复制的数据
    sensor_msgs::ImagePtr img_left_msg = cv_bridge::CvImage(cv_ptrLeft->header, "bgr8", left_img_copy).toImageMsg();
    image_pose_stereo_msg.image_left = *img_left_msg;

    // 右图像转换 - 使用复制的数据
    sensor_msgs::ImagePtr img_right_msg = cv_bridge::CvImage(cv_ptrRight->header, "bgr8", right_img_copy).toImageMsg();
    image_pose_stereo_msg.image_right = *img_right_msg;

    // 位姿
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

    // 跟踪状态
    image_pose_stereo_msg.tracking_success = tracking_success;

    // 发布同步消息
    image_pose_stereo_pub.publish(image_pose_stereo_msg);
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // 首先保存原始彩色图像用于发布（在 ORB-SLAM3 处理之前）
    cv_bridge::CvImageConstPtr cv_ptrLeft_original;
    cv_bridge::CvImageConstPtr cv_ptrRight_original;
    try
    {
        // 强制转换为 BGR8 以保留彩色信息
        cv_ptrLeft_original = cv_bridge::toCvCopy(msgLeft, sensor_msgs::image_encodings::BGR8);
        cv_ptrRight_original = cv_bridge::toCvCopy(msgRight, sensor_msgs::image_encodings::BGR8);
        
        // 如果图像尺寸大于 640x360，则缩放到 640x360
        static bool first_frame = true;
        if (first_frame) {
            ROS_INFO("=== 图像处理信息 ===");
            ROS_INFO("接收到的图像尺寸: %d x %d", cv_ptrLeft_original->image.cols, cv_ptrLeft_original->image.rows);
            first_frame = false;
        }
        
        // 目标分辨率：640x360
        const int target_width = 640;
        const int target_height = 360;
        
        if (cv_ptrLeft_original->image.cols > target_width || cv_ptrLeft_original->image.rows > target_height) {
            // 创建新的 CvImage 对象用于缩放后的图像
            cv_bridge::CvImagePtr cv_ptrLeft_resized(new cv_bridge::CvImage());
            cv_bridge::CvImagePtr cv_ptrRight_resized(new cv_bridge::CvImage());
            
            cv_ptrLeft_resized->header = cv_ptrLeft_original->header;
            cv_ptrLeft_resized->encoding = cv_ptrLeft_original->encoding;
            cv_ptrRight_resized->header = cv_ptrRight_original->header;
            cv_ptrRight_resized->encoding = cv_ptrRight_original->encoding;
            
            // 缩放图像
            cv::resize(cv_ptrLeft_original->image, cv_ptrLeft_resized->image,
                      cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);
            cv::resize(cv_ptrRight_original->image, cv_ptrRight_resized->image,
                      cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);
            
            static bool first_resize = true;
            if (first_resize) {
                ROS_INFO("图像已缩放到: %d x %d", target_width, target_height);
                first_resize = false;
            }
            
            // 使用缩放后的图像
            cv_ptrLeft_original = cv_ptrLeft_resized;
            cv_ptrRight_original = cv_ptrRight_resized;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception when copying original images: %s", e.what());
        return;
    }
    
    // 然后获取用于 ORB-SLAM3 处理的图像（可能会被转换为灰度）
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    bool tracking_success = true;
    
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        Tcw = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        Tcw = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

    // 检查跟踪状态（如果 Tcw 的平移向量为零，可能表示跟踪失败）
    if(Tcw.translation().norm() < 1e-6)
    {
        tracking_success = false;
    }

    // 发布双目图像和位姿 - 使用保存的原始彩色图像
    PublishImagePoseStereo(cv_ptrLeft_original, cv_ptrRight_original, Tcw, tracking_success);
}


