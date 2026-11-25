// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_DETECTION_2D_HCM_CONVERTER_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_DETECTION_2D_HCM_CONVERTER_HPP

#include "QRB/Memory/Vision/ObjectDetection2dDescriptor.hpp"
#include "qrb/contract/vision/v1/detection_2d_array.pb.h"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_conversion.hpp"

namespace qrb::adapter
{

using Detection2DHcmContract = ::qrb::contract::vision::v1::Detection2DArray;
using Detection2DHcmDescriptor = QRB::Memory::ObjectDetection2dDescriptor_t;

/**
 * @brief Detection2DArray to the records in the buffer of the descriptor.
 *
 * Sets timestampNs, frameId, objCount and validSize. A Contract that does not fit the
 * descriptor (frame_id, id and class_id longer than the fixed strings, more than
 * QRB_DETECTION_MAX_RESULTS hypotheses, more detections than the buffer holds) is rejected
 * and leaves the descriptor untouched.
 */
template <>
ConversionResult ConvertContractToHcm<Detection2DHcmContract, Detection2DHcmDescriptor>(
    const Detection2DHcmContract & source,
    const DmaBufView & payload,
    Detection2DHcmDescriptor & target);

/** @brief The records in the buffer of the descriptor, objCount of them, to a Detection2DArray. */
template <>
ConversionResult ConvertHcmToContract<Detection2DHcmContract, Detection2DHcmDescriptor>(
    const Detection2DHcmDescriptor & source,
    Detection2DHcmContract & target);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_DETECTION_2D_HCM_CONVERTER_HPP
