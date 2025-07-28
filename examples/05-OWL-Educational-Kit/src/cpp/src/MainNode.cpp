#include "MainNode.hpp"

MainNode::MainNode(std::string config_path,std::string node_name) 
{
    ref_node = new rclcpp::Node(node_name);
   
    pub_pan = ref_node->create_publisher<std_msgs::msg::Float32>("olive/actuator/pan/goal/position" , 10);
    pub_tilt = ref_node->create_publisher<std_msgs::msg::Float32>("olive/actuator/tilt/goal/position" , 10);

    sub_imu = ref_node->create_subscription<sensor_msgs::msg::Imu>("olive/imu/one/imu" , 10, std::bind(&MainNode::callbackIMU, this, _1));

     thread_main = new std::thread(&MainNode::threadMain,this);
}

void MainNode::threadMain()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    while(thread_exited == false)
    {
        std_msgs::msg::Float32 msg;

        msg.data = pan;
        pub_pan->publish(msg);

        msg.data = tilt;
        pub_tilt->publish(msg);

        std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000 / 25))); //25hz
        
        std::cout << "Pub " << pan << " " << tilt << std::endl;
    }
}

void MainNode::Stop()
{
    pan = 0;
    tilt = 0;
}

void MainNode::callbackIMU(sensor_msgs::msg::Imu::SharedPtr msg)
{
    //std::cout << "get a new imu message" << std::endl;

    geometry_msgs::msg::Quaternion quat_msg = msg->orientation;
    tf2::Quaternion quat;
    tf2::fromMsg(quat_msg, quat);

    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    
    double d_roll = roll * (180 / M_PI) ;
    double d_pitch = pitch * (180 / M_PI) ;
    double d_yaw = yaw * (180 / M_PI) ;

    pan = yaw;
    tilt = pitch;
    
    
}


MainNode::~MainNode() 
{
   
}
