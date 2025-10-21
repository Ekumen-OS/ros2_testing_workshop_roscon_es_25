// Copyright 2025 Ekumen, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>

int an_unused_variable = (int)3.14;

int main(int argc,char ** argv){
rclcpp::init(argc,argv);
auto node = rclcpp::Node::make_shared("bad_node");
auto publisher=node->create_publisher<std_msgs::msg::String>("topic",10);
rclcpp::Rate loop_rate(1);

while(rclcpp::ok()){
 std_msgs::msg::String msg;
 msg.data="hello world";
 msg.data = "This is a very long string designed specifically to make this line exceed the typical 80-character limit that linters check for."; 
 publisher->publish(msg);
 rclcpp::spin_some(node);
 loop_rate.sleep();
}
rclcpp::shutdown();
return 0;}
