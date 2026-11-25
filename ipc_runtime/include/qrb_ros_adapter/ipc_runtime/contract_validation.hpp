// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_VALIDATION_HPP
#define QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_VALIDATION_HPP

#include <cstdint>
#include <string>

#include "qrb/contract/common/v1/header.pb.h"
#include "qrb/contract/geometry/v1/pose_array.pb.h"
#include "qrb/contract/geometry/v1/pose_stamped.pb.h"
#include "qrb/contract/geometry/v1/pose_with_covariance_stamped.pb.h"
#include "qrb/contract/geometry/v1/twist_stamped.pb.h"
#include "qrb/contract/geometry/v1/vector3_stamped.pb.h"
#include "qrb/contract/nav/v1/odometry.pb.h"
#include "qrb/contract/sensor/v1/joint_state.pb.h"
#include "qrb/contract/sensor/v1/point_cloud2.pb.h"
#include "qrb/contract/tf2/v1/tf_message.pb.h"
#include "qrb/contract/vision/v1/detection_2d_array.pb.h"
#include "qrb/contract/vision/v1/detection_3d_array.pb.h"
#include "qrb_ros_adapter/ipc_runtime/contract_limits.hpp"
#include "qrb_ros_adapter/ipc_runtime/conversion_result.hpp"

namespace qrb::adapter
{

ConversionResult ValidateFrameId(const std::string & frameId);

/** @brief Checks `frame_id` and that `stamp.nanosec` is below one second. */
ConversionResult ValidateHeader(const qrb::contract::common::v1::Header & header);

ConversionResult ValidatePoseWithCovarianceStamped(
    const qrb::contract::geometry::v1::PoseWithCovarianceStamped & pose);

/**
 * @brief Checks the header, `child_frame_id` (as a frame_id), the pose as
 * ValidatePoseWithCovarianceStamped() does, and that the twist is finite with an
 * empty or 36-value covariance.
 */
ConversionResult ValidateOdometry(const qrb::contract::nav::v1::Odometry & odometry);

/** @brief Checks the header and that the twist is present and finite. */
ConversionResult ValidateTwistStamped(const qrb::contract::geometry::v1::TwistStamped & twist);

/**
 * @brief Checks the transform count, and for each transform its header,
 * `child_frame_id` (as a frame_id), a finite translation and a finite, non-zero rotation.
 */
ConversionResult ValidateTFMessage(const qrb::contract::tf2::v1::TFMessage & message);

/** @brief Checks the header, joint count/names and finite measurement arrays, each empty or
 * name-sized. */
ConversionResult ValidateJointState(const qrb::contract::sensor::v1::JointState & message);

/**
 * @brief Checks the header and that the pose is present, with a finite position and orientation
 * and an orientation that is not the zero quaternion, as for the pose of
 * ValidatePoseWithCovarianceStamped(). The quaternion may be unnormalized; it is passed on as it
 * is. A missing orientation reads as zero and is rejected.
 */
ConversionResult ValidatePoseStamped(const qrb::contract::geometry::v1::PoseStamped & pose);

/**
 * @brief Checks the pose count (kMaxPoseCount), the header, and each pose as
 * ValidatePoseStamped() does its pose. An empty array is valid: "no pose found" is a result.
 */
ConversionResult ValidatePoseArray(const qrb::contract::geometry::v1::PoseArray & poses);

/** @brief Checks the header and that the vector is present and finite. */
ConversionResult ValidateVector3Stamped(const qrb::contract::geometry::v1::Vector3Stamped & vector);

ConversionResult ValidatePointCloud2(const qrb::contract::sensor::v1::PointCloud2 & cloud,
    std::uint64_t externalPayloadSize);

/**
 * @brief Checks a detection array a peer sent or this side built.
 *
 * Every number must be finite and every box size non-negative; `id` and `class_id` must fit
 * contract_limits::kMaxDetectionStringLength and hold no NUL; a covariance is empty (unknown)
 * or has 36 values. The detector may leave the quaternions of a box or a hypothesis pose
 * unnormalized, or all zero, since many publishers never set them; they are passed on as
 * they are. An empty array is valid: "nothing detected" is a result.
 */
ConversionResult ValidateDetection2DArray(
    const qrb::contract::vision::v1::Detection2DArray & detections);

ConversionResult ValidateDetection3DArray(
    const qrb::contract::vision::v1::Detection3DArray & detections);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_VALIDATION_HPP
