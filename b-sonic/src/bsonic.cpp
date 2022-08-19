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
#include <Eigen/Dense>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <ros/ros.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

class bsonic
{
    private:

    ros::NodeHandle _nh;

    public:

    bsonic(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
    {
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
