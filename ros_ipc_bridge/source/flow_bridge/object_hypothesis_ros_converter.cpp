// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/object_hypothesis_ros_converter.hpp"

#include <algorithm>

namespace qrb::adapter
{

void ObjectHypothesisToContract(const vision_msgs::msg::ObjectHypothesisWithPose & source,
    ::qrb::contract::vision::v1::ObjectHypothesisWithPose & target)
{
  target.mutable_hypothesis()->set_class_id(source.hypothesis.class_id);
  target.mutable_hypothesis()->set_score(source.hypothesis.score);
  PoseWithCovarianceToContract(source.pose, *target.mutable_pose());
  // A NaN is not zero, so it is kept and rejected by the validation.
  const bool known = std::any_of(source.pose.covariance.begin(), source.pose.covariance.end(),
      [](double covariance) { return covariance != 0.0; });
  if (!known)
    target.mutable_pose()->clear_covariance();
}

void ObjectHypothesisToRos(const ::qrb::contract::vision::v1::ObjectHypothesisWithPose & source,
    vision_msgs::msg::ObjectHypothesisWithPose & target)
{
  target.hypothesis.class_id = source.hypothesis().class_id();
  target.hypothesis.score = source.hypothesis().score();
  if (source.has_pose() && source.pose().has_pose())
    PoseWithCovarianceToRos(source.pose(), target.pose);
  else
    target.pose = geometry_msgs::msg::PoseWithCovariance();  // the identity, unknown covariance
}

}  // namespace qrb::adapter
