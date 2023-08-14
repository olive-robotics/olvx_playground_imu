#include "MainNode.hpp"

MainNode::MainNode(std::string config_path,std::string node_name) 
{
    ref_node = new rclcpp::Node(node_name);
   
    pub_front_right = ref_node->create_publisher<std_msgs::msg::Float32>("olive/FrontRight/pwm" , 10);
    pub_front_left = ref_node->create_publisher<std_msgs::msg::Float32>("olive/FrontLeft/pwm" , 10);
    pub_rear_right = ref_node->create_publisher<std_msgs::msg::Float32>("olive/RearRight/pwm" , 10);
    pub_rear_left = ref_node->create_publisher<std_msgs::msg::Float32>("olive/RearLeft/pwm" , 10);
    sub_imu = ref_node->create_subscription<sensor_msgs::msg::Imu>("olive/one/imu" , 10, std::bind(&MainNode::callbackIMU, this, _1));

     thread_main = new std::thread(&MainNode::threadMain,this);
}

void MainNode::threadMain()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    while(thread_exited == false)
    {
        std_msgs::msg::Float32 msg;

        msg.data = front_right;
        pub_front_right->publish(msg);

        msg.data = front_left;
        pub_front_left->publish(msg);

        msg.data = rear_right;
        pub_rear_right->publish(msg);

        msg.data = rear_left;
        pub_rear_left->publish(msg);

        std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000 / rate_control_hz))); //25hz
    }
}


void MainNode::goForward()
{
    front_right = -0.5;
    front_left = 0.5;
    rear_right = -0.5;
    rear_left = 0.5;
}

void MainNode::goBackward()
{
    front_right = 0.5;
    front_left = -0.5;
    rear_right = 0.5;
    rear_left = -0.5;
}

void MainNode::goRight()
{
    front_right = -0.5;
    front_left = -0.5;
    rear_right = -0.5;
    rear_left = -0.5;
}

void MainNode::goLeft()
{
    front_right = 0.5;
    front_left = 0.5;
    rear_right = 0.5;
    rear_left = 0.5;
}

void MainNode::goTurnLeft()
{
    front_right = 0.5;
    front_left = 0.5;
    rear_right = -0.5;
    rear_left = -0.5;
}

void MainNode::goTurnRight()
{
    front_right = -0.5;
    front_left = -0.5;
    rear_right = 0.5;
    rear_left = 0.5;
}

void MainNode::goStop()
{
    front_right = 0;
    front_left = 0;
    rear_right = 0;
    rear_left = 0;
}

void MainNode::callbackIMU(sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::cout << "get a new imu message" << std::endl;

    geometry_msgs::msg::Quaternion quat_msg = msg->orientation;
    tf2::Quaternion quat;
    tf2::fromMsg(quat_msg, quat);

    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    
    double d_roll = roll * (180 / M_PI) ;
    double d_pitch = pitch * (180 / M_PI) ;
    double d_yaw = yaw * (180 / M_PI) ;

    std::cout << d_roll << " " << d_pitch << " " << d_yaw << std::endl;

    if ( d_roll > -40 && d_roll < 40 && d_pitch > -40 && d_pitch < 40 && d_yaw > -40 && d_yaw < 40)
    {
        goStop();
        std::cout << "goStop" << std::endl;
    }
    else if (d_roll > 40 && d_pitch > -40 && d_pitch < 40 && d_yaw > -40 && d_yaw < 40)
    {
        goRight();
        std::cout << "goRight" << std::endl;
        
    }
    else if (d_roll < -40 && d_pitch > -40 && d_pitch < 40 && d_yaw > -40 && d_yaw < 40)
    {
        goLeft();
        std::cout << "goLeft" << std::endl;
    }
    else if (d_roll > -40 && d_roll < 40 && d_pitch > 40 && d_yaw > -40 && d_yaw < 40)
    {
        goForward();
        std::cout << "goForward" << std::endl;
    }
    else if (d_roll > -40 && d_roll < 40 && d_pitch < -40 && d_yaw > -40 && d_yaw < 40)
    {
        goBackward();
        std::cout << "goBackward" << std::endl;
    }
    else if (d_roll > -40 && d_roll < 40 && d_pitch > -40 && d_pitch < 40 && d_yaw > 40 )
    {
        goTurnLeft();
        std::cout << "goTurnLeft" << std::endl;
    }
    else if (d_roll > -40 && d_roll < 40 && d_pitch > -40 && d_pitch < 40 && d_yaw < -40)
    {
        goTurnRight();
        std::cout << "goTurnRight" << std::endl;
    }
}


MainNode::~MainNode() 
{
   
}
