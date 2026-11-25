// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_IMAGE_ROS_CONVERTER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_IMAGE_ROS_CONVERTER_HPP

#include <cstdint>
#include <lib_mem_dmabuf/dmabuf.hpp>
#include <memory>

#include "qrb/contract/sensor/v1/image_layout.pb.h"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_conversion.hpp"
#include "qrb_ros_transport_image_type/image.hpp"

namespace qrb::adapter
{

using ImageRosMessage = qrb_ros::transport::type::Image;
using ImageRosContract = ::qrb::contract::sensor::v1::ImageLayout;

/**
 * @brief Image ROS/Contract policy.
 *
 * The transport type has no step or plane members: its buffer always uses the
 * qrb_ros_transport aligned layout (CanonicalImageLayout()). ROS to Contract
 * therefore leaves `planes` empty and forwards the producer's buffer. Contract to
 * ROS reads the source with the native planes the Contract describes and repacks
 * the valid rows into host storage in the aligned layout.
 */
template <>
struct RosConversion<ImageRosMessage, ImageRosContract>
{
  using RosType = ImageRosMessage;
  using ContractType = ImageRosContract;

  static ConversionResult RosToContract(const RosType & source,
      ContractType & target,
      RosToContractPayload & payload);

  static ConversionResult ContractToRos(const ContractType & source,
      RosType & target,
      ContractToRosPayload & payload);
};

using ImageRosConversion = RosConversion<ImageRosMessage, ImageRosContract>;

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_IMAGE_ROS_CONVERTER_HPP
