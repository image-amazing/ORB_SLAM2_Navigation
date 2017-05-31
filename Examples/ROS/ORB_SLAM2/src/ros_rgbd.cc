/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
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

//----------------------------------------------------------------
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class ImageGrabber
{
public:
    Eigen::Quaterniond quaternion;  //Orientation
    Eigen::Vector3d v_transpose; //Pose
    Eigen::Vector3d euler_angle; //Euler

public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    //void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,Eigen::Quaternionf& quaternion,Eigen::Vector3f& v_transpose);
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
    
    ros::Publisher orb_odom_pub = nh.advertise<nav_msgs::Odometry>("vo", 500);
    tf::TransformBroadcaster odom_broadcaster;
    nav_msgs::Odometry orb_odom;
    ros::Time current_time;

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        current_time = ros::Time::now();
        orb_odom.header.stamp = current_time;
        orb_odom.header.frame_id = "odom";

        //set the position and orientation
        orb_odom.pose.pose.position.x = igb.v_transpose(0);
        orb_odom.pose.pose.position.y = igb.v_transpose(1);
        orb_odom.pose.pose.position.z = igb.v_transpose(2);

        orb_odom.pose.pose.orientation.w = igb.quaternion.w();
        orb_odom.pose.pose.orientation.x = igb.quaternion.x();
        orb_odom.pose.pose.orientation.y = igb.quaternion.y();
        orb_odom.pose.pose.orientation.z = igb.quaternion.z();

        orb_odom.pose.covariance[0] = 0.1;
        orb_odom.pose.covariance[7] = 0.1;
        orb_odom.pose.covariance[14] = 0.1;
        orb_odom.pose.covariance[21] = 0.1;
        orb_odom.pose.covariance[28] = 0.1;
        orb_odom.pose.covariance[35] = 0.1;

        orb_odom_pub.publish(orb_odom);

        //First, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        current_time = ros::Time::now();
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "camera_link";

        odom_trans.transform.translation.x = orb_odom.pose.pose.position.x;
        odom_trans.transform.translation.y = orb_odom.pose.pose.position.y;
        odom_trans.transform.translation.z = orb_odom.pose.pose.position.z;

        geometry_msgs::Quaternion odom_quat;
        odom_quat.x = igb.quaternion.x();
        odom_quat.y = igb.quaternion.y();
        odom_quat.z = igb.quaternion.z();
        odom_quat.w = igb.quaternion.w();
        odom_trans.transform.rotation = odom_quat;
        
        //odom_broadcaster.sendTransform(odom_trans);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    //ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw;
    Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    Eigen::Matrix<double, 4, 4> T;
    cv2eigen(Tcw, T);

    T = T.inverse(); //Twc
    
    Eigen::Matrix<double, 4, 4> Tcp;
    Tcp << 0,0,1,0,-1,0,0,0,0,-1,0,0,0,0,0,1;

    Eigen::Matrix<double, 4, 4> Tpc;
    Tpc << 0,-1,0,0,0,0,-1,0,1,0,0,0,0,0,0,1;

    T = Tcp*T*Tpc;
    Eigen::Matrix<double, 3, 3> R = T.topLeftCorner(3,3);
    Eigen::Quaterniond quaternion_1 = Eigen::Quaterniond ( R );
    quaternion_1.normalize();
    this->quaternion = quaternion_1;
    this->v_transpose = T.topRightCorner(3,1);
    this->euler_angle = R.eulerAngles(2,1,0);
    //cout<< "quaternion = \n" << this->quaternion.coeffs() <<endl;
    //cout<<"yaw pitch roll = "<<euler_angle.transpose()<<endl;
    //cout<< "euler_angles = "<<this->euler_angle<<endl;
    //cout<< "v_transpose = \n" << this->v_transpose <<endl;
}


