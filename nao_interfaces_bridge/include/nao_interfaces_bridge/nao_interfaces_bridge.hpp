// Copyright 2023 Kenji Brameld
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

#ifndef NAO_INTERFACES_BRIDGE__NAO_INTERFACES_BRIDGE_HPP_
#define NAO_INTERFACES_BRIDGE__NAO_INTERFACES_BRIDGE_HPP_

#include "message_filters/time_synchronizer.h"
#include "nao_sensor_msgs/msg/accelerometer.hpp"
#include "nao_sensor_msgs/msg/angle.hpp"
#include "nao_sensor_msgs/msg/gyroscope.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace nao_interfaces_bridge
{

class NaoInterfacesBridge : public rclcpp::Node
{
public:
  explicit NaoInterfacesBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  // Subscriptions
  rclcpp::Subscription<nao_sensor_msgs::msg::Accelerometer>::SharedPtr accelerometer_sub_;
  rclcpp::Subscription<nao_sensor_msgs::msg::Angle>::SharedPtr angle_sub_;
  rclcpp::Subscription<nao_sensor_msgs::msg::Gyroscope>::SharedPtr gyroscope_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

}  // namespace nao_interfaces_bridge

#endif  // NAO_INTERFACES_BRIDGE__NAO_INTERFACES_BRIDGE_HPP_
