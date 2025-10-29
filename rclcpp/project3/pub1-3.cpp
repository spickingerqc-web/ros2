#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

int main(int argc, char * argv[])
{
    char a;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_pub1_3");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", qos_profile );
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(1.0); 
    while(rclcpp::ok())
    {
        std::cin >> a;

        message.linear.x = 0.0;
        message.linear.y = 0.0;
        message.linear.z = 0.0;
        message.angular.x = 0.0;
        message.angular.y = 0.0;
        message.angular.z = 0.0;

        switch(a)
        {
            case 'f':
                message.linear.x =3.0;
                RCLCPP_INFO(node->get_logger(), "Publish: f 누름");
                break;
            case 'b':
                message.linear.x =-3.0;
                break;
            case 'l':
                message.angular.z =3.0;
                break;
            case 'r':
                message.angular.z =-3.0;
                break;
        }
        if((a=='f')||(a=='b')||(a=='l')||(a=='r')){
            mypub->publish(message);
        }
        a = '0';
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}