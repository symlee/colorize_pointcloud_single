/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Ye Cheng
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/


#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/console/print.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_geometry/pinhole_camera_model.h>
#include "pcl_ros/transforms.h"
#include <colorize_pointcloud_single/ColorizerConfig.h>

std::string image_topic;
std::string info_topic;
std::string cloud_topic;
std::string output_topic;
bool keep_outsiders = false;
int color_r;
int color_g;
int color_b;



image_geometry::PinholeCameraModel cam_model_;
tf::TransformListener *listener;
ros::Publisher pub;


void configCb(colorize_pointcloud_single::ColorizerConfig &newconfig, uint32_t level) {
    if (newconfig.keep_outsiders != keep_outsiders)
        keep_outsiders = newconfig.keep_outsiders;
    if (newconfig.r != color_r)
        color_r = newconfig.r;
    if (newconfig.g != color_g)
        color_g = newconfig.g;
    if (newconfig.b != color_b)
        color_b = newconfig.b;
}

void callback(const sensor_msgs::ImageConstPtr &imgMsg, const sensor_msgs::CameraInfoConstPtr &infoMsg,
              const sensor_msgs::PointCloud2ConstPtr &cloudMsg) {
    //get image
    cv_bridge::CvImagePtr cv_ptr_;
    try {
        cv_ptr_ = cv_bridge::toCvCopy(imgMsg);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat img = cv_ptr_->image;
    // if no rgb frame for coloring:
    if (img.data == NULL)
    {
        ROS_ERROR("No Color Information in Image");
        return;
    }

    //transform pointcloud to image frame
    sensor_msgs::PointCloud2 camCld;   // holds the cloud warped into camera's frame (ROS point cloud)
    try{
//        pcl_ros::transformPointCloud("uye_optical", *cloudMsg, camCld, *listener) ;
        pcl_ros::transformPointCloud(imgMsg->header.frame_id, *cloudMsg, camCld, *listener) ;
    }
    catch (tf::TransformException ex){
      std::cout << "here I am " << std::endl;
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    //get Pointcloud
    cam_model_.fromCameraInfo(infoMsg); //build camera model from camera info
    pcl::PCLPointCloud2* cld = new pcl::PCLPointCloud2;  
    pcl_conversions::toPCL(camCld,*cld);   // holds the cloud warped into camera's frame (PCL point cloud)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // convert binary PCLPointCloud2 to to pcl::PointCloud<T> object
    // may be sufficient to leave the therm and rad clouds as PCLPointCloud2 rather than pcl::PointCloud<XYZRGB>
    pcl::fromPCLPointCloud2(*cld,*pcl_cloud);

    pcl::PointIndices fov_indices;

    cv::Point2d uv;
    for(unsigned long i=0; i < pcl_cloud->size(); ++i){
        //is point in front of camera?
        if(pcl_cloud->points[i].z>0) {
            uv = cam_model_.project3dToPixel(
                    cv::Point3d(pcl_cloud->points.at(i).x, pcl_cloud->points.at(i).y, pcl_cloud->points.at(i).z));
            //is pixel in bounds of image?
            if (uv.x >= 0 && uv.x < img.cols && uv.y >= 0 && uv.y < img.rows){
                if (img.channels() == 1) {
//                    ROS_INFO("Image has 1 channels!");
                    pcl_cloud->points[i].r = img.at<uchar>(cv::Point(uv.x,uv.y));
                    pcl_cloud->points[i].g = img.at<uchar>(cv::Point(uv.x,uv.y));
                    pcl_cloud->points[i].b = img.at<uchar>(cv::Point(uv.x,uv.y));
                }
                if (img.channels() == 3) {
//                    ROS_INFO("Image has 3 channels!");
                    pcl_cloud->points[i].r = (uchar)img.at<cv::Vec3b>(uv)[0];
                    pcl_cloud->points[i].g = (uchar)img.at<cv::Vec3b>(uv)[1];
                    pcl_cloud->points[i].b = (uchar)img.at<cv::Vec3b>(uv)[2];
//                    ROS_INFO("R:%i\tG:%i\tB:%i",pcl_cloud->points[i].r,pcl_cloud->points[i].g,pcl_cloud->points[i].b);
                }
                fov_indices.indices.push_back(i);
            }
            //point is not in FOV of the camera
            else{
                pcl_cloud->points[i].r = (uchar)color_r;
                pcl_cloud->points[i].g = (uchar)color_g;
                pcl_cloud->points[i].b = (uchar)color_b;
            }
        }
        //point is behind camera
        else{
            pcl_cloud->points[i].r = (uchar)color_r;
            pcl_cloud->points[i].g = (uchar)color_g;
            pcl_cloud->points[i].b = (uchar)color_b;
        }
    }
    sensor_msgs::PointCloud2 color_cld;
    if(!keep_outsiders) {
//        ROS_INFO("Only Displaying Points in View");
        pcl::ExtractIndices<pcl::PointXYZRGB> extractFOVPoints;
        extractFOVPoints.setIndices(boost::make_shared<const pcl::PointIndices>(fov_indices));
        extractFOVPoints.setInputCloud(pcl_cloud);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
        extractFOVPoints.filter(*output);
        pcl::toROSMsg(*output, color_cld);
    }
    else
        pcl::toROSMsg(*pcl_cloud, color_cld);

    sensor_msgs::PointCloud2 output_cloud;
    try{
        pcl_ros::transformPointCloud(cloudMsg->header.frame_id, color_cld, output_cloud, *listener) ;
    }
    catch (tf::TransformException ex){
      std::cout << "here I am 2" << std::endl;
         ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    output_cloud.header=cloudMsg->header;

    pub.publish(output_cloud);
    fov_indices.indices.clear();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "colorize_pointcloud_single");
    ros::NodeHandle nh("~");
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    dynamic_reconfigure::Server<colorize_pointcloud_single::ColorizerConfig> server;
    dynamic_reconfigure::Server<colorize_pointcloud_single::ColorizerConfig>::CallbackType f;

    f = boost::bind(&configCb, _1, _2);
    server.setCallback(f);

    listener = new tf::TransformListener();

    ros::param::get("~keep_outsiders", keep_outsiders);

    ros::param::get("~rect_image", image_topic);
    ros::param::get("~camera_info", info_topic);
    ros::param::get("~cloud", cloud_topic);
    ros::param::get("~output_cloud", output_topic);
    ros::param::get("~r", color_r);
    ros::param::get("~g", color_g);
    ros::param::get("~b", color_b);


    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, image_topic, 1);//
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, info_topic, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, cloud_topic, 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), image_sub, info_sub, cloud_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    pub = nh.advertise<sensor_msgs::PointCloud2> (output_topic, 1);

    ros::spin();
    return 0;
}
