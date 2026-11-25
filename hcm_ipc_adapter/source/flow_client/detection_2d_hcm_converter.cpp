// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/detection_2d_hcm_converter.hpp"

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/object_hypothesis_hcm_converter.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

namespace qrb::adapter
{

namespace
{

namespace vision_proto = ::qrb::contract::vision::v1;

void BoxToHcm(const vision_proto::BoundingBox2D & source, QRB::Memory::BoundingBox2d_t & target)
{
  target.center.position.x = source.center().position().x();
  target.center.position.y = source.center().position().y();
  target.center.theta = source.center().theta();
  target.sizeX = source.size_x();
  target.sizeY = source.size_y();
}

void BoxToContract(const QRB::Memory::BoundingBox2d_t & source,
    vision_proto::BoundingBox2D & target)
{
  target.mutable_center()->mutable_position()->set_x(source.center.position.x);
  target.mutable_center()->mutable_position()->set_y(source.center.position.y);
  target.mutable_center()->set_theta(source.center.theta);
  target.set_size_x(source.sizeX);
  target.set_size_y(source.sizeY);
}

}  // namespace

template <>
ConversionResult ConvertContractToHcm<Detection2DHcmContract, Detection2DHcmDescriptor>(
    const Detection2DHcmContract & detections,
    const DmaBufView & /*payload*/,
    Detection2DHcmDescriptor & descriptor)
{
  ConversionResult validation = ValidateDetection2DArray(detections);
  if (!validation)
    return validation;
  return DetectionArrayToHcm<QRB::Memory::Detection2dRaw_t>(detections, descriptor, BoxToHcm);
}

template <>
ConversionResult ConvertHcmToContract<Detection2DHcmContract, Detection2DHcmDescriptor>(
    const Detection2DHcmDescriptor & descriptor,
    Detection2DHcmContract & detections)
{
  ConversionResult converted = DetectionArrayHcmToContract<QRB::Memory::Detection2dRaw_t>(
      descriptor, detections, BoxToContract);
  if (!converted)
    return converted;
  return ValidateDetection2DArray(detections);
}

}  // namespace qrb::adapter
