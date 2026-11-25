// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_IMAGE_HCM_CONVERTER_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_IMAGE_HCM_CONVERTER_HPP

#include <cstdint>
#include <memory>

#include "QC/Infras/Memory/CameraBufferDescriptor.hpp"
#include "QC/Infras/Memory/Ifs/QCBufferDescriptorBase.hpp"
#include "qrb/contract/sensor/v1/image_layout.pb.h"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_conversion.hpp"
#include "qrb_ros_adapter/ipc_runtime/conversion_result.hpp"

namespace qrb::adapter
{

/**
 * @brief One paired reading of the two clocks a timestamp crosses.
 *
 * HCM producers stamp frames with the hardware qtimer, the ARM generic timer
 * that counts from boot (`QCarCam` `sofTimestamp.timestamp`, as stored by the
 * QCNode Camera). ROS stamps messages with its default clock, which is the
 * system `CLOCK_REALTIME`. Both values are in nanoseconds.
 */
struct HcmClockSample
{
  std::uint64_t qtimerNs{0};
  std::uint64_t realtimeNs{0};
};

/** @brief A source of paired clock readings. */
using HcmClockSampler = HcmClockSample (*)();

/**
 * @brief Read both clocks now through the installed sampler.
 *
 * The default sampler reads the qtimer on either side of `CLOCK_REALTIME` and
 * pairs the realtime value with their midpoint. A fresh sample is taken for
 * every conversion so that a system time correction applies immediately.
 */
HcmClockSample SampleHcmClocks();

/**
 * @brief Replace the process-wide clock sampler.
 *
 * Intended for tests, which need a fixed offset to assert exact values.
 * Passing nullptr restores the default hardware sampler.
 *
 * @return The sampler that was installed before the call.
 */
HcmClockSampler SetHcmClockSampler(HcmClockSampler sampler);

/**
 * @brief Convert an HCM qtimer timestamp to the ROS time base.
 *
 * `rosNs = qtimerNs + (realtime_now - qtimer_now)`. Fails, leaving `rosNs`
 * untouched, when the result is zero, negative or overflows 64 bits.
 */
ConversionResult HcmTimestampToRos(std::uint64_t qtimerNs, std::uint64_t & rosNs);

/**
 * @brief Convert a ROS timestamp to the HCM qtimer time base.
 *
 * `qtimerNs = rosNs - (realtime_now - qtimer_now)`. Fails, leaving `qtimerNs`
 * untouched, when the ROS time is not after boot, so the qtimer value would be
 * zero or negative, or when the result overflows 64 bits.
 */
ConversionResult RosTimestampToHcm(std::uint64_t rosNs, std::uint64_t & qtimerNs);

/**
 * The Image Contract binds the QCNode Camera frame descriptor.
 *
 * The descriptor `timestamp` is the camera qtimer (QCarCam sofTimestamp), the
 * Contract `header.stamp` is ROS time (CLOCK_REALTIME); the converters convert
 * between them. Camera frames have no frame ID, so HCM to ROS leaves
 * `header.frame_id` empty, and ROS to HCM drops it.
 */
using ImageHcmContract = ::qrb::contract::sensor::v1::ImageLayout;
using ImageHcmDescriptor = QC::Memory::CameraFrameDescriptor_t;

// Declared as in hcm_converters.hpp, which defines it. Converter headers are
// included while contract_mappings.def is being expanded, so they must not
// include hcm_converters.hpp (and through it the Contract registry) themselves.
template <typename DescriptorT>
std::unique_ptr<QC::Memory::QCBufferDescriptorBase_t> CreateHcmDescriptor();

/** @brief A Camera frame descriptor whose Camera fields start at zero. */
template <>
std::unique_ptr<QC::Memory::QCBufferDescriptorBase_t> CreateHcmDescriptor<ImageHcmDescriptor>();

/**
 * @brief Copy a received Image into the host descriptor.
 *
 * A descriptor that describes its own layout (numPlanes > 0, as QCNode allocates
 * it) keeps that layout: the valid rows land at its stride and plane offsets. A
 * descriptor with no layout (numPlanes == 0) adopts the source layout and
 * receives the whole source storage.
 */
template <>
ConversionResult ConvertContractToHcm<ImageHcmContract, ImageHcmDescriptor>(
    const ImageHcmContract & source,
    const DmaBufView & payload,
    ImageHcmDescriptor & target);

/** @brief Describe a host Camera frame, including its native plane layout. */
template <>
ConversionResult ConvertHcmToContract<ImageHcmContract, ImageHcmDescriptor>(
    const ImageHcmDescriptor & source,
    ImageHcmContract & target);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_IMAGE_HCM_CONVERTER_HPP
