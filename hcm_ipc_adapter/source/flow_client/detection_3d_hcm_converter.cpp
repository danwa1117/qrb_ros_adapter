// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/detection_3d_hcm_converter.hpp"

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/object_hypothesis_hcm_converter.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

namespace qrb::adapter
{

namespace
{

namespace vision_proto = ::qrb::contract::vision::v1;

void BoxToHcm(const vision_proto::BoundingBox3D & source, QRB::Memory::BoundingBox3d_t & target)
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

void BoxToContract(const QRB::Memory::BoundingBox3d_t & source,
    vision_proto::BoundingBox3D & target)
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

}  // namespace

template <>
ConversionResult ConvertContractToHcm<Detection3DHcmContract, Detection3DHcmDescriptor>(
    const Detection3DHcmContract & detections,
    const DmaBufView & /*payload*/,
    Detection3DHcmDescriptor & descriptor)
{
  ConversionResult validation = ValidateDetection3DArray(detections);
  if (!validation)
    return validation;
  return DetectionArrayToHcm<QRB::Memory::Detection3dRaw_t>(detections, descriptor, BoxToHcm);
}

template <>
ConversionResult ConvertHcmToContract<Detection3DHcmContract, Detection3DHcmDescriptor>(
    const Detection3DHcmDescriptor & descriptor,
    Detection3DHcmContract & detections)
{
  ConversionResult converted = DetectionArrayHcmToContract<QRB::Memory::Detection3dRaw_t>(
      descriptor, detections, BoxToContract);
  if (!converted)
    return converted;
  return ValidateDetection3DArray(detections);
}

}  // namespace qrb::adapter
