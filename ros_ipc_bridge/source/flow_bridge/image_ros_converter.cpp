// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/image_ros_converter.hpp"

#include <array>
#include <cstddef>
#include <limits>

#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_header.hpp"

namespace qrb::adapter
{

namespace
{

/** @brief Signed ROS dimensions; a negative value must not wrap into uint32. */
ConversionResult
CheckRosDimensions(int width, int height, std::uint32_t & outWidth, std::uint32_t & outHeight)
{
  if (width <= 0 || height <= 0)
    return ConversionResult::Failure("ROS image dimensions must be positive");
  outWidth = static_cast<std::uint32_t>(width);
  outHeight = static_cast<std::uint32_t>(height);
  return ConversionResult::Success();
}

ConversionResult
CheckContractDimensions(std::uint32_t width, std::uint32_t height, int & outWidth, int & outHeight)
{
  constexpr auto kMaxRosDimension = static_cast<std::uint32_t>(std::numeric_limits<int>::max());
  if (width > kMaxRosDimension || height > kMaxRosDimension)
    return ConversionResult::Failure("Contract image dimensions exceed the ROS int range");
  outWidth = static_cast<int>(width);
  outHeight = static_cast<int>(height);
  return ConversionResult::Success();
}

/** @brief Every Image carries a valid, non-zero acquisition time. */
ConversionResult CheckImageStamp(const qrb::contract::common::v1::Time & stamp)
{
  std::uint64_t nanos = 0;
  ConversionResult time = ContractTimeToNanos(stamp, nanos);
  if (!time)
    return time;
  if (nanos == 0)
    return ConversionResult::Failure("Image requires a non-zero source timestamp");
  return ConversionResult::Success();
}

}  // namespace

ConversionResult ImageRosConversion::RosToContract(const ImageRosMessage & source,
    ImageRosContract & target,
    RosToContractPayload & payload)
{
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  ConversionResult dimensions = CheckRosDimensions(source.width, source.height, width, height);
  if (!dimensions)
    return dimensions;

  // The producer's buffer uses the fixed qrb_ros_transport aligned layout.
  ImageLayout layout;
  ConversionResult canonical = CanonicalImageLayout(width, height, source.encoding, layout);
  if (!canonical)
    return canonical;

  target.Clear();
  HeaderToContract(source.header, *target.mutable_header());
  ConversionResult time = CheckImageStamp(target.header().stamp());
  if (!time)
    return time;
  target.set_width(width);
  target.set_height(height);
  target.set_encoding(source.encoding);
  target.set_step(static_cast<std::uint32_t>(layout.step));
  // planes stay empty and batch_size 0: the source is the aligned ROS layout, and
  // data stays empty because the pixels travel in the DMA-BUF.
  ConversionResult validation = ValidateImageLayout(target, layout.storageSize);
  if (!validation)
    return validation;

  // Forwards the producer's own buffer; the flow bridge keeps it alive until HCM
  // confirms its copy.
  return payload.Bind(source.dmabuf, layout.storageSize);
}

ConversionResult ImageRosConversion::ContractToRos(const ImageRosContract & source,
    ImageRosMessage & target,
    ContractToRosPayload & payload)
{
  ConversionResult time = CheckImageStamp(source.header().stamp());
  if (!time)
    return time;
  int width = 0;
  int height = 0;
  ConversionResult dimensions =
      CheckContractDimensions(source.width(), source.height(), width, height);
  if (!dimensions)
    return dimensions;

  // The source layout is whatever the native producer allocated; resolving it
  // also checks it fits the range the IPC payload delivered.
  ImageLayout sourceLayout;
  ConversionResult resolved = ResolveImageLayout(source, payload.AvailableCapacity(), sourceLayout);
  if (!resolved)
    return resolved;

  // The destination is always the layout qrb_ros_transport expects.
  ImageLayout destinationLayout;
  ConversionResult canonical =
      CanonicalImageLayout(source.width(), source.height(), source.encoding(), destinationLayout);
  if (!canonical)
    return canonical;
  if (sourceLayout.planeCount != destinationLayout.planeCount) {
    return ConversionResult::Failure(
        "source and canonical ROS layouts disagree on the plane count");
  }

  std::array<PayloadCopyRegion, kMaxPayloadCopyRegions> regions{};
  for (std::size_t index = 0; index < sourceLayout.planeCount; ++index) {
    const ImagePlaneLayout & from = sourceLayout.planes[index];
    const ImagePlaneLayout & to = destinationLayout.planes[index];
    if (from.rowBytes != to.rowBytes || from.rows != to.rows) {
      return ConversionResult::Failure(
          "source and canonical ROS layouts disagree on the valid pixels");
    }
    regions[index] =
        PayloadCopyRegion{from.offset, from.stride, to.offset, to.stride, to.rowBytes, to.rows};
  }

  HeaderToRos(source.header(), target.header);
  target.width = width;
  target.height = height;
  target.encoding = source.encoding();
  return payload.BindRegions(
      target.dmabuf, destinationLayout.storageSize, regions.data(), sourceLayout.planeCount);
}

}  // namespace qrb::adapter
