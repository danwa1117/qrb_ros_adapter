// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/detection_2d_ros_converter.hpp"

#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/object_hypothesis_ros_converter.hpp"

namespace qrb::adapter
{

namespace
{

void BoxToContract(const vision_msgs::msg::BoundingBox2D & source,
    ::qrb::contract::vision::v1::BoundingBox2D & target)
{
  auto * center = target.mutable_center();
  center->mutable_position()->set_x(source.center.position.x);
  center->mutable_position()->set_y(source.center.position.y);
  center->set_theta(source.center.theta);
  target.set_size_x(source.size_x);
  target.set_size_y(source.size_y);
}

void BoxToRos(const ::qrb::contract::vision::v1::BoundingBox2D & source,
    vision_msgs::msg::BoundingBox2D & target)
{
  target.center.position.x = source.center().position().x();
  target.center.position.y = source.center().position().y();
  target.center.theta = source.center().theta();
  target.size_x = source.size_x();
  target.size_y = source.size_y();
}

}  // namespace

ConversionResult Detection2DRosConversion::RosToContract(const Detection2DRosMessage & source,
    Detection2DRosContract & target)
{
  ConversionResult converted = DetectionArrayRosToContract(source, target, BoxToContract);
  if (!converted)
    return converted;
  return ValidateDetection2DArray(target);
}

ConversionResult Detection2DRosConversion::ContractToRos(const Detection2DRosContract & source,
    Detection2DRosMessage & target)
{
  ConversionResult validation = ValidateDetection2DArray(source);
  if (!validation)
    return validation;
  DetectionArrayToRos(source, target, BoxToRos);
  return ConversionResult::Success();
}

}  // namespace qrb::adapter
