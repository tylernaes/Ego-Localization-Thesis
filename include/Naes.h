#pragma once
#include "ros/ros.h"
#include "ros/console.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "naes/current_position.h"
#include "naes/wheel_speeds.h"
#include "naes/stop.h"
#include "naes/ego_points.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "stdlib.h"
#include "iostream"
#include <fstream>
#include "string"
#include <vector>
#include <cmath>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

#ifndef NAES_H
#define NAES_H

using namespace std;
using namespace naes;

#endif
