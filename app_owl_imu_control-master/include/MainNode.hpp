#ifndef __OLIVEAPP_MAINNODE__
#define __OLIVEAPP_MAINNODE__

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/byte.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "math.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include<stdlib.h>
#include <cassert>
#include <memory>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>
#include <array>
#include <chrono>
#include <thread>
#include <fstream>
#include <cmath>
#include <sstream>
#include <fstream>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class MainNode;

class MainNode 
{
public:
    MainNode(std::string config_path,std::string node_name);
    ~MainNode();

    //Functions
    void threadMain();
    int rate_control_hz = 10;
    rclcpp::Node *ref_node;
    bool thread_exited = false;
    std::thread *thread_main;

    void Stop();

    float pan = 0;
    float tilt = 0;
   
    //Pub/Sub
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_pan;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_tilt;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;

    void callbackIMU(sensor_msgs::msg::Imu::SharedPtr msg);


    
};

#endif