#include <string>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc,char ** argv){
rclcpp::init(argc,argv);
auto node = rclcpp::Node::make_shared("bad_node");
auto publisher=node->create_publisher<std_msgs::msg::String>("topic",10);
rclcpp::Rate loop_rate(1);

while(rclcpp::ok()){
 std_msgs::msg::String msg;
 msg.data="hello world";
 publisher->publish(msg);
 rclcpp::spin_some(node);
 loop_rate.sleep();
}

rclcpp::shutdown();
return 0;}
