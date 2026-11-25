// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_DETECTION_3D_ROS_CONVERTER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_DETECTION_3D_ROS_CONVERTER_HPP

#include <vision_msgs/msg/detection3_d_array.hpp>

#include "qrb/contract/vision/v1/detection_3d_array.pb.h"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_conversion.hpp"

namespace qrb::adapter
{

using Detection3DRosMessage = vision_msgs::msg::Detection3DArray;
using Detection3DRosContract = ::qrb::contract::vision::v1::Detection3DArray;

/**
 * @brief vision_msgs/Detection3DArray <-> qrb.contract.vision.v1.Detection3DArray.
 *
 * The Contract has one header, that of the array: a detection's own header is replaced by it
 * (stamps) or must match it (frame_id, see CheckDetectionFrame).
 */
template <>
struct RosConversion<Detection3DRosMessage, Detection3DRosContract>
{
  using RosType = Detection3DRosMessage;
  using ContractType = Detection3DRosContract;

  static ConversionResult RosToContract(const RosType & source, ContractType & target);
  static ConversionResult ContractToRos(const ContractType & source, RosType & target);
};

using Detection3DRosConversion = RosConversion<Detection3DRosMessage, Detection3DRosContract>;

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_DETECTION_3D_ROS_CONVERTER_HPP
