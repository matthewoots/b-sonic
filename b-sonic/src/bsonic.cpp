/*
* bsonic.cpp
*
* ---------------------------------------------------------------------
* Created by Matthew (matthewoots@gmail.com) in 2022
*
*  This program is free software; you can redistribute it and/or
*  modify it under the terms of the GNU General Public License
*  as published by the Free Software Foundation; either version 2
*  of the License, or (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
* ---------------------------------------------------------------------
*/

#include <iostream>
#include <string>
#include <mutex>
#include <chrono>
#include <ctime>
#include <math.h>
#include <random>
#include <vector>
#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <pcl/filters/crop_box.h>

#include <ros/ros.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace Eigen;
using namespace std;

class bsonic
{
    private:

    ros::NodeHandle _nh;

    ros::Publisher local_pcl_pub;
    ros::Subscriber pose_sub;
    ros::Timer cloud_timer;

    ros::Time pose_time;

    std::mutex pose_mutex; 

    std::string _id;
    int number_obstacles;
    vector<Eigen::Vector2d> obstacle_location, obstacles_layer;
    double min_height, max_height, radius, resolution, cloud_hz, sensor_range;

    Eigen::Vector3d pose;

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud;

    public:

    bsonic(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
    {
        _nh.param<std::string>("agent_id", _id, "drone0");

        std::vector<double> obstacles;
        std::vector<double> height_list;
        _nh.getParam("height", height_list);

        min_height = height_list[0];
        max_height = height_list[1];

        double height_difference = max_height - min_height;

        _nh.getParam("obstacles", obstacles);
        number_obstacles = (int)obstacles.size() / 2;
        for (int i = 0; i < number_obstacles; i++)
            obstacle_location.push_back(
                Eigen::Vector2d(obstacles[i*2+0],obstacles[i*2+1])
            );
        
        _nh.param<double>("radius", radius, 1.0); 
        _nh.param<double>("resolution", resolution, 1.0);
        _nh.param<double>("cloud_publish_hz", cloud_hz, 1.0); 
        _nh.param<double>("sensor_range", sensor_range, 1.0);

        /** @brief Timer that handles cloud at each interval */
        cloud_timer = _nh.createTimer(
            ros::Duration(1/cloud_hz), 
            &bsonic::cloud_update_timer, this, false, false);

        pose_sub = _nh.subscribe<geometry_msgs::PoseStamped>(
            "/" + _id + "/uav/nwu", 20, &bsonic::pose_callback, this);

        /** @brief Publisher that publishes pointcloud */
        local_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>(
            "/" + _id + "/local_pcl", 20);

        // Construct the obstacles
        int height_layers = (int)ceil(height_difference / resolution);
        // arc_length = resolution, the seperation between 2 points
        double seperating_angle = resolution / radius;

        // 2pi full circle / seperation
        int circle_interval = (int)floor((2 * 3.141593) / seperating_angle);

        for (int i = 0; i < number_obstacles; i++)
        {
            for (int j = 0; j < circle_interval; j++)
            {
                obstacles_layer.push_back(
                    Eigen::Vector2d(
                    radius * sin(seperating_angle*j) + obstacle_location[i].x(),
                    radius * cos(seperating_angle*j) + obstacle_location[i].y())
                );
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(
           new pcl::PointCloud<pcl::PointXYZ>);

        for (int i = 0; i < height_layers; i++)
        {
            for (int j = 0; j < (int)obstacles_layer.size(); j++)
                temp_cloud->points.push_back(
                    pcl::PointXYZ(
                        obstacles_layer[j].x(),
                        obstacles_layer[j].y(),
                        min_height + resolution * i
                    )
                );
        }

        std::cout << "[bsonic] drone id is " << KRED << _id << KNRM << std::endl;

        global_cloud = temp_cloud;
        cloud_timer.start();
    }

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> pose_lock(pose_mutex);

        pose_time = ros::Time::now();

        // Global position in NWU frame
        pose = Eigen::Vector3d(
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z
        );
    }


    void cloud_update_timer(const ros::TimerEvent &)
    {
        std::lock_guard<std::mutex> pose_lock(pose_mutex);
        
        if (global_cloud->points.size() == 0 || (ros::Time::now() - pose_time).toSec() > 1.0)
            return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);

        Eigen::Vector3f min_bnd = Eigen::Vector3f(
            (float)(pose.x() - sensor_range), 
            (float)(pose.y() - sensor_range), 
            (float)(max(pose.z() - sensor_range, min_height)));
        Eigen::Vector3f max_bnd = Eigen::Vector3f(
            (float)(pose.x() + sensor_range), 
            (float)(pose.y() + sensor_range), 
            (float)(min(pose.z() + sensor_range, max_height)));

        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setMin(
            Eigen::Vector4f(min_bnd.x(), min_bnd.y(), min_bnd.z(), 1.0f));
        box_filter.setMax(
            Eigen::Vector4f(max_bnd.x(), max_bnd.y(), max_bnd.z(), 1.0f));

        box_filter.setInputCloud(global_cloud);
        box_filter.filter(*temp_cloud);

        sensor_msgs::PointCloud2 cloud_msg;

        pcl::toROSMsg(*temp_cloud, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "world";
        local_pcl_pub.publish(cloud_msg);
    }

    ~bsonic(){}

};

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "bsonic_node");
    ros::NodeHandle nh("~");
    bsonic bsonic(nh);
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;

}
