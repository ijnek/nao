// Copyright 2021 Kenji Brameld
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
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

std::vector<std::string> joint_names = {
  "HeadYaw",
  "HeadPitch",
  "LShoulderPitch",
  "LShoulderRoll",
  "LElbowYaw",
  "LElbowRoll",
  "LWristYaw",
  "LHipYawPitch",
  "LHipRoll",
  "LHipPitch",
  "LKneePitch",
  "LAnklePitch",
  "LAnkleRoll",
  "RHipRoll",
  "RHipPitch",
  "RKneePitch",
  "RAnklePitch",
  "RAnkleRoll",
  "RShoulderPitch",
  "RShoulderRoll",
  "RElbowYaw",
  "RElbowRoll",
  "RWristYaw",
  "LHand",
  "RHand",
};

class ZeroJointStatePublisher : public rclcpp::Node
{
public:
  ZeroJointStatePublisher()
  : Node("ZeroJointStatePublisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&ZeroJointStatePublisher::timer_callback, this));

    for (std::string joint : joint_names) {
      state.name.push_back(joint);
      state.position.push_back(0);
    }
  }

private:
  void timer_callback()
  {
    state.header.stamp = now();
    publisher_->publish(state);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

  sensor_msgs::msg::JointState state;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZeroJointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
