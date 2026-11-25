// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_DETECTION_3D_HCM_CONVERTER_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_DETECTION_3D_HCM_CONVERTER_HPP

#include "QRB/Memory/Vision/ObjectDetection3dDescriptor.hpp"
#include "qrb/contract/vision/v1/detection_3d_array.pb.h"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_conversion.hpp"

namespace qrb::adapter
{

using Detection3DHcmContract = ::qrb::contract::vision::v1::Detection3DArray;
using Detection3DHcmDescriptor = QRB::Memory::ObjectDetection3dDescriptor_t;

/**
 * @brief Detection3DArray to the records in the buffer of the descriptor.
 *
 * Sets timestampNs, frameId, objCount and validSize. A Contract that does not fit the
 * descriptor (frame_id, id and class_id longer than the fixed strings, more than
 * QRB_DETECTION_MAX_RESULTS hypotheses, more detections than the buffer holds) is rejected
 * and leaves the descriptor untouched.
 */
template <>
ConversionResult ConvertContractToHcm<Detection3DHcmContract, Detection3DHcmDescriptor>(
    const Detection3DHcmContract & source,
    const DmaBufView & payload,
    Detection3DHcmDescriptor & target);

/** @brief The records in the buffer of the descriptor, objCount of them, to a Detection3DArray. */
template <>
ConversionResult ConvertHcmToContract<Detection3DHcmContract, Detection3DHcmDescriptor>(
    const Detection3DHcmDescriptor & source,
    Detection3DHcmContract & target);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_DETECTION_3D_HCM_CONVERTER_HPP
