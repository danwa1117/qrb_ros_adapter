// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_OBJECT_HYPOTHESIS_HCM_CONVERTER_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_OBJECT_HYPOTHESIS_HCM_CONVERTER_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

#include "QRB/Memory/Vision/ObjectDetectionDescriptor.hpp"
#include "qrb/contract/vision/v1/object_hypothesis_with_pose.pb.h"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_conversion.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"

namespace qrb::adapter
{

/**
 * @brief What the 2-D and 3-D detection converters share.
 *
 * A detection descriptor keeps its detections as fixed-size records in its own buffer (the
 * Contract itself is Inline). The converters read and write the records with memcpy, since
 * descriptor.offset need not be aligned for the doubles of a record. A Contract that does not fit
 * the record limits (string length, hypotheses per detection, buffer capacity) is rejected whole
 * before the descriptor is touched, never truncated. The flow client syncs the buffer around the
 * conversion, so the converters do not.
 */

/** @brief Copy a Contract string into a NUL-terminated char array; fails if it does not fit. */
template <std::size_t N>
[[nodiscard]] bool AssignHcmCharArray(const std::string & source, char (&target)[N])
{
  if (source.size() >= N)
    return false;
  std::memset(target, 0, N);
  std::memcpy(target, source.data(), source.size());
  return true;
}

/** @brief Read a char array back as a string; fails if it has no terminating NUL. */
template <std::size_t N>
[[nodiscard]] bool HcmCharArrayToContract(const char (&source)[N], std::string & target)
{
  const void * end = std::memchr(source, '\0', N);
  if (end == nullptr)
    return false;
  target.assign(source, static_cast<std::size_t>(static_cast<const char *>(end) - source));
  return true;
}

/** @brief Fails if the hypothesis does not fit an ObjectHypothesisRaw_t. */
ConversionResult CheckHypothesisCapacity(
    const ::qrb::contract::vision::v1::ObjectHypothesisWithPose & source);

/** @brief Fills @p target from a hypothesis that passed CheckHypothesisCapacity; cannot fail. */
void HypothesisToHcm(const ::qrb::contract::vision::v1::ObjectHypothesisWithPose & source,
    QRB::Memory::ObjectHypothesisRaw_t & target);

/**
 * @brief Appends the hypothesis to the Contract.
 *
 * The covariance is left out when it is all zero (unknown). Fails if the class id is not
 * NUL-terminated.
 */
ConversionResult HypothesisToContract(const QRB::Memory::ObjectHypothesisRaw_t & source,
    ::qrb::contract::vision::v1::ObjectHypothesisWithPose & target);

/**
 * @brief Checks that @p count records of @p recordSize bytes can be read or written at the
 * buffer of @p descriptor, from a process that can reach it: host memory or a local DMA-BUF.
 *
 * Zero records need no buffer: an empty array is a result.
 */
ConversionResult CheckDetectionStorage(const QC::Memory::QCBufferDescriptorBase_t & descriptor,
    std::uint64_t count,
    std::size_t recordSize);

/**
 * @brief Contract to descriptor, for the arrays that differ only in their bounding box.
 *
 * @param boxToHcm  Copies a Contract `bbox` into a record's. Does not validate the Contract; the
 * caller does, before it calls this.
 */
template <typename RecordT, typename ContractT, typename DescriptorT, typename BoxToHcm>
ConversionResult DetectionArrayToHcm(const ContractT & contract,
    DescriptorT & descriptor,
    BoxToHcm boxToHcm)
{
  if (contract.header().frame_id().size() > QRB::Memory::QRB_FRAME_ID_MAX_LEN) {
    return ConversionResult::Failure("frame_id is longer than the HCM detection descriptor holds");
  }
  std::uint64_t timestampNs = 0;
  ConversionResult time = ContractTimeToNanos(contract.header().stamp(), timestampNs);
  if (!time)
    return time;

  const auto count = static_cast<std::uint64_t>(contract.detections_size());
  ConversionResult storage = CheckDetectionStorage(descriptor, count, sizeof(RecordT));
  if (!storage)
    return storage;
  for (int i = 0; i < contract.detections_size(); ++i) {
    const auto & detection = contract.detections(i);
    const std::string where = "detection " + std::to_string(i) + ": ";
    if (detection.id().size() >= QRB_DETECTION_MAX_ID_LEN)
      return ConversionResult::Failure(where + "id is longer than the HCM descriptor holds");
    if (static_cast<std::size_t>(detection.results_size()) > QRB_DETECTION_MAX_RESULTS) {
      return ConversionResult::Failure(where +
                                       "has more hypotheses than the HCM descriptor holds (" +
                                       std::to_string(QRB_DETECTION_MAX_RESULTS) + ")");
    }
    for (const auto & result : detection.results()) {
      ConversionResult capacity = CheckHypothesisCapacity(result);
      if (!capacity)
        return ConversionResult::Failure(where + capacity.Diagnostic());
    }
  }

  // Everything that can fail is checked, so the descriptor changes only for a Contract that fits.
  auto * base = static_cast<std::uint8_t *>(count > 0 ? descriptor.GetDataPtr() : nullptr);
  for (int i = 0; i < contract.detections_size(); ++i) {
    const auto & detection = contract.detections(i);
    RecordT record{};
    (void)AssignHcmCharArray(detection.id(), record.id);
    boxToHcm(detection.bbox(), record.bbox);
    record.resultCount = static_cast<std::uint32_t>(detection.results_size());
    for (int j = 0; j < detection.results_size(); ++j)
      HypothesisToHcm(detection.results(j), record.results[j]);
    std::memcpy(base + static_cast<std::size_t>(i) * sizeof(RecordT), &record, sizeof(RecordT));
  }

  if (!AssignHcmString(contract.header().frame_id(), descriptor.frameId))
    return ConversionResult::Failure("frame_id does not fit the HCM detection descriptor");
  descriptor.timestampNs = timestampNs;
  descriptor.objCount = static_cast<std::uint32_t>(count);
  descriptor.validSize = static_cast<std::size_t>(count) * sizeof(RecordT);
  return ConversionResult::Success();
}

/**
 * @brief Descriptor to Contract, the counterpart of DetectionArrayToHcm.
 *
 * @param boxToContract  Copies a record's `bbox` into the Contract's. The caller validates the
 * filled Contract.
 */
template <typename RecordT, typename DescriptorT, typename ContractT, typename BoxToContract>
ConversionResult DetectionArrayHcmToContract(const DescriptorT & descriptor,
    ContractT & contract,
    BoxToContract boxToContract)
{
  contract.Clear();
  ConversionResult time =
      NanosToContractTime(descriptor.timestampNs, *contract.mutable_header()->mutable_stamp());
  if (!time)
    return time;
  contract.mutable_header()->set_frame_id(HcmStringToContract(descriptor.frameId));

  const std::uint64_t count = descriptor.objCount;
  ConversionResult storage = CheckDetectionStorage(descriptor, count, sizeof(RecordT));
  if (!storage)
    return storage;
  const auto * base =
      static_cast<const std::uint8_t *>(count > 0 ? descriptor.GetDataPtr() : nullptr);

  for (std::uint64_t i = 0; i < count; ++i) {
    RecordT record;
    std::memcpy(&record, base + static_cast<std::size_t>(i) * sizeof(RecordT), sizeof(RecordT));
    const std::string where = "detection " + std::to_string(i) + ": ";
    if (record.resultCount > QRB_DETECTION_MAX_RESULTS)
      return ConversionResult::Failure(where + "resultCount exceeds the descriptor capacity");
    std::string id;
    if (!HcmCharArrayToContract(record.id, id))
      return ConversionResult::Failure(where + "id is not NUL-terminated");

    auto * detection = contract.add_detections();
    detection->set_id(std::move(id));
    boxToContract(record.bbox, *detection->mutable_bbox());
    for (std::uint32_t j = 0; j < record.resultCount; ++j) {
      ConversionResult result = HypothesisToContract(record.results[j], *detection->add_results());
      if (!result)
        return ConversionResult::Failure(where + result.Diagnostic());
    }
  }
  return ConversionResult::Success();
}

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_OBJECT_HYPOTHESIS_HCM_CONVERTER_HPP
