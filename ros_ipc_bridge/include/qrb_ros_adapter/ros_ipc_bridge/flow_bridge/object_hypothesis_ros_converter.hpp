// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_OBJECT_HYPOTHESIS_ROS_CONVERTER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_OBJECT_HYPOTHESIS_ROS_CONVERTER_HPP

#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include "qrb/contract/common/v1/header.pb.h"
#include "qrb/contract/geometry/v1/pose_with_covariance.pb.h"
#include "qrb/contract/vision/v1/object_hypothesis_with_pose.pb.h"
#include "qrb_ros_adapter/ipc_runtime/conversion_result.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_geometry.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_header.hpp"

namespace qrb::adapter
{

/**
 * @brief Field-for-field copies shared by the 2-D and 3-D detection converters.
 *
 * The pose is copied with PoseWithCovarianceToContract/Ros of ros_contract_geometry.hpp. Unlike
 * a Pose, a hypothesis keeps its covariance only when it is not all zero: an empty one means
 * unknown, which is how ROS reads a zero matrix, and a detection costs 300 bytes less on the wire
 * per hypothesis. A Contract without a pose reads as the identity. None of the copies can fail;
 * the Contract validation checks the values.
 */
void ObjectHypothesisToContract(const vision_msgs::msg::ObjectHypothesisWithPose & source,
    ::qrb::contract::vision::v1::ObjectHypothesisWithPose & target);

void ObjectHypothesisToRos(const ::qrb::contract::vision::v1::ObjectHypothesisWithPose & source,
    vision_msgs::msg::ObjectHypothesisWithPose & target);

/**
 * @brief Rejects a ROS detection whose own frame differs from that of its array.
 *
 * The Contract has one header per array, so a detection's header is not carried. An empty
 * frame_id, or the array's, loses nothing; another frame would silently move the box into the
 * array's frame.
 */
template <typename DetectionT>
ConversionResult CheckDetectionFrame(const DetectionT & detection,
    const std_msgs::msg::Header & arrayHeader,
    std::size_t index)
{
  if (!detection.header.frame_id.empty() && detection.header.frame_id != arrayHeader.frame_id) {
    return ConversionResult::Failure(
        "detection " + std::to_string(index) + " is in a different frame than its array: '" +
        detection.header.frame_id + "' vs '" + arrayHeader.frame_id + "'");
  }
  return ConversionResult::Success();
}

/**
 * @brief The loop Detection2DArray and Detection3DArray share, apart from the bounding box.
 *
 * @param boxToContract  Copies a detection's `bbox` into the Contract's.
 * Does not validate; the caller validates the filled Contract.
 */
template <typename RosArrayT, typename ContractT, typename BoxToContract>
ConversionResult DetectionArrayRosToContract(const RosArrayT & source,
    ContractT & target,
    BoxToContract boxToContract)
{
  target.Clear();
  HeaderToContract(source.header, *target.mutable_header());
  target.mutable_detections()->Reserve(static_cast<int>(source.detections.size()));
  for (std::size_t i = 0; i < source.detections.size(); ++i) {
    const auto & detection = source.detections[i];
    ConversionResult frame = CheckDetectionFrame(detection, source.header, i);
    if (!frame)
      return frame;
    auto * out = target.add_detections();
    out->set_id(detection.id);
    boxToContract(detection.bbox, *out->mutable_bbox());
    for (const auto & result : detection.results)
      ObjectHypothesisToContract(result, *out->add_results());
  }
  return ConversionResult::Success();
}

/**
 * @brief The Contract to ROS counterpart; the caller validates the Contract first.
 *
 * Every detection gets the header of the array, as ROS consumers read a detection's own.
 */
template <typename ContractT, typename RosArrayT, typename BoxToRos>
void DetectionArrayToRos(const ContractT & source, RosArrayT & target, BoxToRos boxToRos)
{
  target = RosArrayT();
  HeaderToRos(source.header(), target.header);
  target.detections.resize(static_cast<std::size_t>(source.detections_size()));
  for (int i = 0; i < source.detections_size(); ++i) {
    const auto & in = source.detections(i);
    auto & out = target.detections[static_cast<std::size_t>(i)];
    out.header = target.header;
    out.id = in.id();
    boxToRos(in.bbox(), out.bbox);
    out.results.resize(static_cast<std::size_t>(in.results_size()));
    for (int j = 0; j < in.results_size(); ++j)
      ObjectHypothesisToRos(in.results(j), out.results[static_cast<std::size_t>(j)]);
  }
}

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_OBJECT_HYPOTHESIS_ROS_CONVERTER_HPP
