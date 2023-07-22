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

#include "nao_interfaces_bridge/nao_interfaces_bridge.hpp"

namespace nao_interfaces_bridge
{

using nao_sensor_msgs::msg::Accelerometer;
using nao_sensor_msgs::msg::Angle;
using nao_sensor_msgs::msg::Gyroscope;
using sensor_msgs::msg::Imu;

NaoInterfacesBridge::NaoInterfacesBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node{"nao_interfaces_bridge", options}
{
  // Message filter subscribers
  rclcpp::QoS qos(10);
  auto rmw_qos_profile = qos.get_rmw_qos_profile();
  accelerometer_sub_.subscribe(this, "sensors/accelerometer", rmw_qos_profile);
  angle_sub_.subscribe(this, "sensors/angle", rmw_qos_profile);
  gyroscope_sub_.subscribe(this, "sensors/gyroscope", rmw_qos_profile);

  // Publishers
  imu_pub_ = create_publisher<Imu>("imu", 10);

  // Synchronizer
  synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<Accelerometer, Angle, Gyroscope>>(
    accelerometer_sub_, angle_sub_, gyroscope_sub_, 10);
  synchronizer_->registerCallback(&NaoInterfacesBridge::synchronizerCallback, this);
}

void NaoInterfacesBridge::synchronizerCallback(
  const Accelerometer::SharedPtr, const Angle::SharedPtr, const Gyroscope::SharedPtr)
{
  imu_pub_->publish(Imu{});
}

}  // namespace nao_interfaces_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nao_interfaces_bridge::NaoInterfacesBridge)
