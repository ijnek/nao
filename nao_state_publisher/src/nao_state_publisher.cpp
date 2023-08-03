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
#include <map>
#include <utility>
#include "robot_state_publisher/robot_state_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nao_lola_sensor_msgs/msg/joint_positions.hpp"
#include "nao_lola_sensor_msgs/msg/joint_indexes.hpp"

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

class NaoStatePublisher : public robot_state_publisher::RobotStatePublisher
{
public:
  NaoStatePublisher()
  : robot_state_publisher::RobotStatePublisher(rclcpp::NodeOptions())
  {
    subscriber_ = this->create_subscription<nao_lola_sensor_msgs::msg::JointPositions>(
      "sensors/joint_positions", 10, std::bind(
        &NaoStatePublisher::callbackNaoJoints, this,
        std::placeholders::_1));
  }

private:
  void callbackNaoJoints(const nao_lola_sensor_msgs::msg::JointPositions::SharedPtr sensor_joints)
  {
    std::map<std::string, double> joint_positions;
    for (unsigned i = 0; i < nao_lola_sensor_msgs::msg::JointIndexes::NUMJOINTS; ++i) {
      joint_positions.insert({joint_names[i], sensor_joints->positions[i]});
    }

    // ensure mimic joints are mimiced (see robot_state_publisher.cpp for more info)
    for (const std::pair<const std::string, urdf::JointMimicSharedPtr> & i : mimic_) {
      if (joint_positions.find(i.second->joint_name) != joint_positions.end()) {
        double pos = joint_positions[i.second->joint_name] * i.second->multiplier +
          i.second->offset;
        joint_positions.insert(std::make_pair(i.first, pos));
      }
    }

    publishTransforms(joint_positions, now());
  }

  rclcpp::Subscription<nao_lola_sensor_msgs::msg::JointPositions>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NaoStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
