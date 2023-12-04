#include<rclcpp/rclcpp.hpp>
#include"mapping.hpp"

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    printf("**********flash3d-lio*********\n");
    auto node=std::make_shared<LioNode>("flash3d_mapping");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;

}