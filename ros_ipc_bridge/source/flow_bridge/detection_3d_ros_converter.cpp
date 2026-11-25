// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/detection_3d_ros_converter.hpp"

#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/object_hypothesis_ros_converter.hpp"

namespace qrb::adapter
{

namespace
{

void BoxToContract(const vision_msgs::msg::BoundingBox3D & source,
    ::qrb::contract::vision::v1::BoundingBox3D & target)
{
  auto * position = target.mutable_center()->mutable_position();
  position->set_x(source.center.position.x);
  position->set_y(source.center.position.y);
  position->set_z(source.center.position.z);
  auto * orientation = target.mutable_center()->mutable_orientation();
  orientation->set_x(source.center.orientation.x);
  orientation->set_y(source.center.orientation.y);
  orientation->set_z(source.center.orientation.z);
  orientation->set_w(source.center.orientation.w);
  target.mutable_size()->set_x(source.size.x);
  target.mutable_size()->set_y(source.size.y);
  target.mutable_size()->set_z(source.size.z);
}

void BoxToRos(const ::qrb::contract::vision::v1::BoundingBox3D & source,
    vision_msgs::msg::BoundingBox3D & target)
{
  target.center.position.x = source.center().position().x();
  target.center.position.y = source.center().position().y();
  target.center.position.z = source.center().position().z();
  target.center.orientation.x = source.center().orientation().x();
  target.center.orientation.y = source.center().orientation().y();
  target.center.orientation.z = source.center().orientation().z();
  target.center.orientation.w = source.center().orientation().w();
  target.size.x = source.size().x();
  target.size.y = source.size().y();
  target.size.z = source.size().z();
}

}  // namespace

ConversionResult Detection3DRosConversion::RosToContract(const Detection3DRosMessage & source,
    Detection3DRosContract & target)
{
  ConversionResult converted = DetectionArrayRosToContract(source, target, BoxToContract);
  if (!converted)
    return converted;
  return ValidateDetection3DArray(target);
}

ConversionResult Detection3DRosConversion::ContractToRos(const Detection3DRosContract & source,
    Detection3DRosMessage & target)
{
  ConversionResult validation = ValidateDetection3DArray(source);
  if (!validation)
    return validation;
  DetectionArrayToRos(source, target, BoxToRos);
  return ConversionResult::Success();
}

}  // namespace qrb::adapter
