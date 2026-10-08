// Copyright 2026 SO-ARM simulation contributors
// SPDX-License-Identifier: Apache-2.0

#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace so_arm_gazebo
{
// Assisted grasp: creates a physical fixed joint at the current relative pose.
// ROS callbacks queue requests; only the Gazebo update thread touches physics.
class GraspPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = std::move(model);
    node_ = gazebo_ros::Node::Get(sdf);
    object_name_ = Read<std::string>(sdf, "object_model", "pick_cube");
    object_link_name_ = Read<std::string>(sdf, "object_link", "link");
    gripper_link_name_ = Read<std::string>(sdf, "gripper_link", "gripper_link");
    gripper_joint_name_ = Read<std::string>(sdf, "gripper_joint", "gripper");
    tool_offset_ = Read<ignition::math::Vector3d>(
      sdf, "tool_offset", ignition::math::Vector3d(0, -0.100, 0));
    max_distance_ = Read<double>(sdf, "max_attach_distance", 0.018);
    closed_position_ = Read<double>(sdf, "closed_position", 0.0);
    closed_tolerance_ = Read<double>(sdf, "closed_tolerance", 0.08);
    if (!node_ || !std::isfinite(max_distance_) || max_distance_ <= 0 ||
      !std::isfinite(closed_position_) || !std::isfinite(closed_tolerance_) ||
      closed_tolerance_ < 0 || !tool_offset_.IsFinite())
    {
      gzerr << "SO-ARM grasp plugin has invalid configuration.\n";
      return;
    }
    service_ = node_->create_service<std_srvs::srv::SetBool>(
      "set_attached", [this](
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response)
      {
        auto command = std::make_shared<Command>();
        command->attach = request->data;
        std::unique_lock<std::mutex> lock(mutex_);
        pending_.push_back(command);
        if (!condition_.wait_for(lock, std::chrono::seconds(5), [&] {
            return command->done || stopping_;
          }))
        {
          command->cancelled = true;
          response->success = false;
          response->message = "Physics update timed out; unpause Gazebo and retry.";
          return;
        }
        response->success = command->done && command->success;
        response->message = command->done ? command->message : "Simulator is shutting down.";
      });
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo &) { OnUpdate(); });
    RCLCPP_INFO(node_->get_logger(), "Assisted grasp ready: %s, maximum TCP distance %.3f m",
      service_->get_service_name(), max_distance_);
  }

  ~GraspPlugin() override
  {
    update_.reset();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_all();
    service_.reset();
  }

  void Reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Release();
    for (const auto & command : pending_) {
      command->done = true;
      command->message = "World reset interrupted the request.";
    }
    pending_.clear();
    condition_.notify_all();
  }

private:
  struct Command
  {
    bool attach{false};
    bool done{false};
    bool success{false};
    bool cancelled{false};
    std::string message;
  };

  template<typename T>
  static T Read(const sdf::ElementPtr & sdf, const std::string & key, const T & fallback)
  {
    return sdf->HasElement(key) ? sdf->Get<T>(key) : fallback;
  }

  void OnUpdate()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    while (!pending_.empty()) {
      auto command = pending_.front();
      pending_.pop_front();
      if (command->cancelled) {
        continue;
      }
      try {
        if (command->attach) {
          Attach(*command);
        } else {
          Release();
          command->success = true;
          command->message = "Object released; gravity and table collisions are active.";
        }
      } catch (const std::exception & error) {
        Release();
        command->message = std::string("Grasp operation failed: ") + error.what();
      }
      command->done = true;
    }
    condition_.notify_all();
  }

  void Attach(Command & command)
  {
    if (attachment_) {
      command.success = true;
      command.message = "Object is already attached.";
      return;
    }
    auto gripper = model_->GetLink(gripper_link_name_);
    auto gripper_joint = model_->GetJoint(gripper_joint_name_);
    auto object = model_->GetWorld()->ModelByName(object_name_);
    if (!gripper || !gripper_joint || !object) {
      command.message = "Robot gripper or configured object was not found.";
      return;
    }
    auto link = object->GetLink(object_link_name_);
    if (!link || object->IsStatic()) {
      command.message = "Object must have the configured dynamic link.";
      return;
    }
    if (std::abs(gripper_joint->Position(0) - closed_position_) > closed_tolerance_) {
      command.message = "Close the gripper before attaching.";
      return;
    }
    const auto pose = gripper->WorldPose();
    const auto tcp = pose.Pos() + pose.Rot().RotateVector(tool_offset_);
    const double distance = tcp.Distance(link->WorldPose().Pos());
    if (!std::isfinite(distance) || distance > max_distance_) {
      command.message = "Object is outside the TCP grasp radius (distance " +
        std::to_string(distance) + " m).";
      return;
    }
    // Suppress contact with the printed jaws while held. The demo is an assisted
    // grasp, not a friction/contact grasp. Restore every mask when releasing.
    object_link_ = link;
    for (const auto & collision : link->GetCollisions()) {
      collision_masks_.emplace_back(collision, collision->GetSurface()->collideBitmask);
      collision->GetSurface()->collideBitmask = 0;
      collision->SetCollideBits(0);
    }
    attachment_ = model_->GetWorld()->Physics()->CreateJoint("fixed", model_);
    attachment_->SetName("so_arm_assisted_grasp");
    attachment_->Load(gripper, link, ignition::math::Pose3d::Zero);
    attachment_->Init();
    command.success = true;
    command.message = "Attached at current pose; TCP distance " +
      std::to_string(distance) + " m.";
    RCLCPP_INFO(node_->get_logger(), "%s", command.message.c_str());
  }

  void Release()
  {
    if (attachment_) {
      attachment_->Detach();
      attachment_->Fini();
      attachment_.reset();
    }
    for (const auto & entry : collision_masks_) {
      entry.first->GetSurface()->collideBitmask = entry.second;
      entry.first->SetCollideBits(entry.second);
    }
    collision_masks_.clear();
    if (object_link_) {
      // Release is requested at a stopped waypoint to avoid a numerical impulse.
      object_link_->SetLinearVel(ignition::math::Vector3d::Zero);
      object_link_->SetAngularVel(ignition::math::Vector3d::Zero);
      object_link_.reset();
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr attachment_;
  gazebo::physics::LinkPtr object_link_;
  gazebo::event::ConnectionPtr update_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  std::vector<std::pair<gazebo::physics::CollisionPtr, unsigned int>> collision_masks_;
  ignition::math::Vector3d tool_offset_;
  std::string object_name_, object_link_name_, gripper_link_name_, gripper_joint_name_;
  double max_distance_{0.018}, closed_position_{0}, closed_tolerance_{0.08};
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<std::shared_ptr<Command>> pending_;
  bool stopping_{false};
};

GZ_REGISTER_MODEL_PLUGIN(GraspPlugin)
}  // namespace so_arm_gazebo
