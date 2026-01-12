
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/core/core.hpp>
#include <System.h>

#include <sys/stat.h> 

#include "orb_slam3_ros/ImagePose.h"

#define ZED_RGB_RECT "/zedm/zed_node/rgb/image_rect_color"
#define USB_RAW "/usb_cam/image_raw"
#define FISHEYE_VIDEO "/fisheye/raw"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ros::NodeHandle* nh):mpSLAM(pSLAM),node_handle(nh)
    {
        image_pose_pub = node_handle->advertise<orb_slam3_ros::ImagePose>("/orb_slam3/image_pose", 10);
    }

    // *************************************************************** //
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Point2f cam2img(const cv::Point3f& point3D);
    void FisheyeWarp(const cv::Mat& fisheyeImage, cv::Mat& undistortedImage, const cv::Size& newSize);

    void PublishImagePose(const cv_bridge::CvImageConstPtr& cv_ptr, const Sophus::SE3f& Tcw);
    // *************************************************************** // 

    ORB_SLAM3::System* mpSLAM;

    Sophus::SE3f Tcw;

    Eigen::Vector4f init_pose {1.0, 1.0, 1.0, 1.0};
    Eigen::Vector4f camera_point;
    float translation_size = 5.3;
    
    cv::Point2f imagePoint;

    int index = 0;

    // ************ time *****************//
    std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
    float time_diff_range = 0.;

private:
    cv::Mat K = (cv::Mat_<double>(3, 3) << 
        138.54264656, 0, 331.89824222,    // fx, 0, cx
        0, 138.60053687, 239.70296783,    // 0, fy, cy  
        0, 0, 1
    );

    cv::Mat D = (cv::Mat_<double>(4, 1) << 
        -0.05094921, -0.00983458, 0.00521841, -0.00128268
    );

    ros::NodeHandle* node_handle;
    ros::Publisher image_pose_pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,false); // 可视化开关

    ros::NodeHandle nodeHandler;
    ImageGrabber igb(&SLAM,&nodeHandler);
    ros::Subscriber sub = nodeHandler.subscribe(USB_RAW, 1, &ImageGrabber::GrabImage,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

cv::Point2f ImageGrabber::cam2img(const cv::Point3f& point3D){
    vector<cv::Point2f> imagePoints;
    std::vector<cv::Point3f> objectPoints = {point3D};
    cv::Affine3d affine = cv::Affine3d::Identity();
    cv::fisheye::projectPoints(objectPoints, imagePoints, affine, K, D);
    // std::cout << "相机坐标: (" << point3D.x << ", " << point3D.y << ", " << point3D.z << ")" << std::endl;
    // std::cout << "投影后的2D像素坐标: (" << imagePoints[0].x << ", " << imagePoints[0].y << ")" << std::endl;
    return imagePoints[0];
}

void ImageGrabber::FisheyeWarp(const cv::Mat& fisheyeImage, cv::Mat& undistortedImage, const cv::Size& newSize) {
    cv::Mat map1, map2;
    cv::fisheye::initUndistortRectifyMap(K, D, cv::Mat::eye(3, 3, CV_64F), K, newSize, CV_16SC2, map1, map2);
    cv::remap(fisheyeImage, undistortedImage, map1, map2, cv::INTER_LINEAR);
}

void ImageGrabber::PublishImagePose(const cv_bridge::CvImageConstPtr& cv_ptr, const Sophus::SE3f& Tcw){
    // 自定义消息类型
    orb_slam3_ros::ImagePose image_pose_msg;

    // 设置时间戳用于同步
    image_pose_msg.header.stamp = cv_ptr->header.stamp; // 将新消息的时间戳与原始图像消息的时间戳保持一致
    image_pose_msg.header.frame_id = "camera"; // 指定位姿数据所在的参考坐标系为"camera"

    // 图像转换
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",cv_ptr->image).toImageMsg();
    image_pose_msg.image = *img_msg;

    // 位姿
    Eigen::Matrix3f R = Tcw.rotationMatrix();
    Eigen::Vector3f t = Tcw.translation();

    image_pose_msg.pose.position.x = t.x();
    image_pose_msg.pose.position.y = t.y();
    image_pose_msg.pose.position.z = t.z();

    Eigen::Quaternionf q(R);
    q.normalize();

    image_pose_msg.pose.orientation.x = q.x();
    image_pose_msg.pose.orientation.y = q.y();
    image_pose_msg.pose.orientation.z = q.z();
    image_pose_msg.pose.orientation.w = q.w();

    // 发布同步消息
    image_pose_pub.publish(image_pose_msg);
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

    // 发布同步信息
    auto end_time = std::chrono::steady_clock::now();
    double time_diff = std::chrono::duration<double>(end_time - time_start).count();
    std::cout << "time_diff: " << time_diff << std::endl;
    if(time_diff > this->time_diff_range){
        PublishImagePose(cv_ptr,Tcw);
        time_start = std::chrono::steady_clock::now();
    }
    
    Eigen::Matrix3f R = Tcw.matrix().block<3, 3>(0, 0);
    Eigen::Vector3f t = Tcw.matrix().block<3, 1>(0, 3);

    Eigen::Vector3f t_right = t*translation_size;

    // 四元数解决万向节死锁问题
    Eigen::Quaternionf q(R);
    q.normalize();

    Sophus::SE3f Tcw_right(R,t_right); // 放大位移向量以便观察效果
    
    // std::cout << "\n位移向量: " << t.transpose()*translation_size << std::endl;
    // std::cout << "旋转矩阵: " << R << std::endl;
    // std::cout << "四元数: " << q.coeffs().transpose() << std::endl;

    // cout << "Tcw = " << endl << Tcw.matrix() << endl;
    camera_point = Tcw_right.matrix() * init_pose;
    // cout << "new pos: " << camera_point << endl;
    
    // 创建一个黑色背景图像
    cv::Mat blackImage = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::Mat RGBImage;
    cv::Mat WarpImage;
    
    // 投影到图像坐标系
    cv::cvtColor(cv_ptr->image, RGBImage, cv::COLOR_BGR2RGB);
    imagePoint = cam2img(cv::Point3f(camera_point[0], camera_point[1], camera_point[2]));
    // cv::circle(RGBImage, imagePoint, 10, cv::Scalar(255, 255, 0), -1);

    FisheyeWarp(RGBImage, WarpImage, cv::Size(640, 480));
    // cv::imshow("ImagePoint", RGBImage);
    cv::imshow("WarpImage", WarpImage);
    if (cv::waitKey(1) == 's'){
        cout << "按压 s" <<endl;
        // 保存图片
        std::string img_path = "/home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/src/ORB_SLAM3_ROS/color_img/" + std::to_string(index) + ".png";
        if(!cv::imwrite(img_path, RGBImage)){
            ROS_ERROR("保存图片失败: %s", img_path.c_str());
        } else {
            // 追加写入位移向量(t)和四元数(q)到 info.txt
            std::ofstream ofs("/home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/src/ORB_SLAM3_ROS/color_img/" + std::to_string(index) + ".txt", std::ios::app);
            if (ofs.is_open()){
                ofs << t_right(0) << " " << t_right(1) << " " << t_right(2) << " " 
                    << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
                    << std::endl;
                ofs.close();

                cout << "\n" << t_right(0) << " " << t_right(1) << " " << t_right(2) << " " 
                    << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
                    << std::endl;
                    
            } else {
                ROS_ERROR("无法打开 ./color_img/pose.txt 写入信息");
            }
        }

        index++;
    }
}