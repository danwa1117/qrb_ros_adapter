// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_DETECTION_2D_ROS_CONVERTER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_DETECTION_2D_ROS_CONVERTER_HPP

#include <vision_msgs/msg/detection2_d_array.hpp>

#include "qrb/contract/vision/v1/detection_2d_array.pb.h"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_conversion.hpp"

namespace qrb::adapter
{

using Detection2DRosMessage = vision_msgs::msg::Detection2DArray;
using Detection2DRosContract = ::qrb::contract::vision::v1::Detection2DArray;

/**
 * @brief vision_msgs/Detection2DArray <-> qrb.contract.vision.v1.Detection2DArray.
 *
 * The Contract has one header, that of the array: a detection's own header is replaced by it
 * (stamps) or must match it (frame_id, see CheckDetectionFrame).
 */
template <>
struct RosConversion<Detection2DRosMessage, Detection2DRosContract>
{
  using RosType = Detection2DRosMessage;
  using ContractType = Detection2DRosContract;

  static ConversionResult RosToContract(const RosType & source, ContractType & target);
  static ConversionResult ContractToRos(const ContractType & source, RosType & target);
};

using Detection2DRosConversion = RosConversion<Detection2DRosMessage, Detection2DRosContract>;

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_DETECTION_2D_ROS_CONVERTER_HPP
