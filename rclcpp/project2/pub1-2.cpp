#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("nodepub3");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("topic_pub1_2", qos_profile );
    
    geometry_msgs::msg::Vector3 message;
    rclcpp::WallRate loop_rate(1.0);
    while(rclcpp::ok())
    {
        std::cout << "첫번째 실수값을 입력해주세요";
        std::cin >> message.x;
        std::cout << "두번째 실수값을 입력해주세요";
        std::cin >> message.y;
        std::cout << "세번째 실수값을 입력해주세요";
        std::cin >> message.z;
        RCLCPP_INFO(node->get_logger(), "Publish: %f,%f,%f", message.x,message.y,message.z);
        mypub->publish(message);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}