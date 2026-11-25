// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/object_hypothesis_hcm_converter.hpp"

#include <algorithm>

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_dma_buffer.hpp"

namespace qrb::adapter
{

namespace vision_proto = ::qrb::contract::vision::v1;

ConversionResult CheckHypothesisCapacity(const vision_proto::ObjectHypothesisWithPose & source)
{
  if (source.hypothesis().class_id().size() >= QRB_DETECTION_MAX_CLASS_ID_LEN) {
    return ConversionResult::Failure("class_id is longer than the HCM descriptor holds");
  }
  return ConversionResult::Success();
}

void HypothesisToHcm(const vision_proto::ObjectHypothesisWithPose & source,
    QRB::Memory::ObjectHypothesisRaw_t & target)
{
  (void)AssignHcmCharArray(source.hypothesis().class_id(), target.classId);
  target.score = source.hypothesis().score();
  // An absent pose reads as the identity with unknown covariance: what a record starts with.
  if (!source.has_pose())
    return;
  const auto & position = source.pose().pose().position();
  target.pose.pose.position.x = position.x();
  target.pose.pose.position.y = position.y();
  target.pose.pose.position.z = position.z();
  const auto & orientation = source.pose().pose().orientation();
  target.pose.pose.orientation.x = orientation.x();
  target.pose.pose.orientation.y = orientation.y();
  target.pose.pose.orientation.z = orientation.z();
  target.pose.pose.orientation.w = orientation.w();
  target.pose.covariance.fill(0.0);
  for (int i = 0; i < source.pose().covariance_size() && i < 36; ++i)
    target.pose.covariance[static_cast<std::size_t>(i)] = source.pose().covariance(i);
}

ConversionResult HypothesisToContract(const QRB::Memory::ObjectHypothesisRaw_t & source,
    vision_proto::ObjectHypothesisWithPose & target)
{
  std::string classId;
  if (!HcmCharArrayToContract(source.classId, classId))
    return ConversionResult::Failure("classId is not NUL-terminated");
  target.mutable_hypothesis()->set_class_id(std::move(classId));
  target.mutable_hypothesis()->set_score(source.score);

  auto * pose = target.mutable_pose();
  auto * position = pose->mutable_pose()->mutable_position();
  position->set_x(source.pose.pose.position.x);
  position->set_y(source.pose.pose.position.y);
  position->set_z(source.pose.pose.position.z);
  auto * orientation = pose->mutable_pose()->mutable_orientation();
  orientation->set_x(source.pose.pose.orientation.x);
  orientation->set_y(source.pose.pose.orientation.y);
  orientation->set_z(source.pose.pose.orientation.z);
  orientation->set_w(source.pose.pose.orientation.w);
  // A NaN is not zero, so it is kept and rejected by the validation.
  const bool known = std::any_of(source.pose.covariance.begin(), source.pose.covariance.end(),
      [](double covariance) { return covariance != 0.0; });
  if (known) {
    for (double covariance : source.pose.covariance)
      pose->add_covariance(covariance);
  }
  return ConversionResult::Success();
}

ConversionResult CheckDetectionStorage(const QC::Memory::QCBufferDescriptorBase_t & descriptor,
    std::uint64_t count,
    std::size_t recordSize)
{
  if (count == 0)
    return ConversionResult::Success();

  int fd = -1;
  const HcmBufferAccess access = InspectHcmBuffer(descriptor, fd);
  if (access != HcmBufferAccess::kHostMemory && access != HcmBufferAccess::kLocalDmaBuf) {
    return ConversionResult::Failure(
        "HCM detection descriptor has no buffer this process can read or write");
  }
  // InspectHcmBuffer checked offset <= size.
  const auto * buffer = dynamic_cast<const QC::Memory::BufferDescriptor_t *>(&descriptor);
  const std::uint64_t capacity = buffer->size - buffer->offset;
  if (count > capacity / recordSize) {
    return ConversionResult::Failure(
        "detections do not fit the buffer of the HCM detection descriptor");
  }
  return ConversionResult::Success();
}

}  // namespace qrb::adapter
