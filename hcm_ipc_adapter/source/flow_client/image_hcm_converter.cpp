// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/image_hcm_converter.hpp"

#include <time.h>

#include <atomic>
#include <cstring>
#include <limits>
#include <string>
#include <utility>

#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

namespace qrb::adapter
{

// --- HCM qtimer <-> ROS time -------------------------------------------------

namespace
{

std::uint64_t ReadClockNs(clockid_t clock)
{
  timespec now{};
  (void)::clock_gettime(clock, &now);
  return static_cast<std::uint64_t>(now.tv_sec) * kNanosPerSecond +
         static_cast<std::uint64_t>(now.tv_nsec);
}

#if defined(__aarch64__)
std::uint64_t ReadCounter()
{
  std::uint64_t value;
  asm volatile("mrs %0, cntvct_el0" : "=r"(value));
  return value;
}

std::uint64_t ReadCounterFrequency()
{
  std::uint64_t value;
  asm volatile("mrs %0, cntfrq_el0" : "=r"(value));
  return value;
}
#endif

/** @brief The qtimer in nanoseconds, computed as the camera stack does. */
std::uint64_t ReadQtimerNs()
{
#if defined(__aarch64__)
  const std::uint64_t count = ReadCounter();
  const std::uint64_t frequency = ReadCounterFrequency();
  if (frequency == 0)
    return 0;
  // Split to keep count * 1e9 from overflowing: the remainder is below the
  // frequency, so remainder * 1e9 fits in 64 bits for any real timer.
  return (count / frequency) * kNanosPerSecond + (count % frequency) * kNanosPerSecond / frequency;
#else
  // Host builds have no qtimer. CLOCK_MONOTONIC also counts from boot and
  // stands in for it so the Adapter and its tests still build and run.
  return ReadClockNs(CLOCK_MONOTONIC);
#endif
}

HcmClockSample SampleHardwareClocks()
{
  const std::uint64_t before = ReadQtimerNs();
  const std::uint64_t realtime = ReadClockNs(CLOCK_REALTIME);
  const std::uint64_t after = ReadQtimerNs();
  // The midpoint pairs the realtime read with the qtimer at the same instant.
  return HcmClockSample{before + (after - before) / 2, realtime};
}

std::atomic<HcmClockSampler> g_sampler{&SampleHardwareClocks};

}  // namespace

HcmClockSample SampleHcmClocks()
{
  return g_sampler.load(std::memory_order_acquire)();
}

HcmClockSampler SetHcmClockSampler(HcmClockSampler sampler)
{
  return g_sampler.exchange(
      sampler == nullptr ? &SampleHardwareClocks : sampler, std::memory_order_acq_rel);
}

ConversionResult HcmTimestampToRos(std::uint64_t qtimerNs, std::uint64_t & rosNs)
{
  const HcmClockSample now = SampleHcmClocks();
  std::uint64_t result = 0;
  if (now.realtimeNs >= now.qtimerNs) {
    const std::uint64_t offset = now.realtimeNs - now.qtimerNs;
    if (qtimerNs > std::numeric_limits<std::uint64_t>::max() - offset)
      return ConversionResult::Failure("HCM timestamp overflows the ROS time base");
    result = qtimerNs + offset;
  } else {
    // The system time is earlier than the time since boot: it was never set.
    const std::uint64_t offset = now.qtimerNs - now.realtimeNs;
    if (qtimerNs <= offset)
      return ConversionResult::Failure("HCM timestamp is before the ROS time origin");
    result = qtimerNs - offset;
  }
  if (result == 0)
    return ConversionResult::Failure("HCM timestamp maps to a zero ROS time");
  rosNs = result;
  return ConversionResult::Success();
}

ConversionResult RosTimestampToHcm(std::uint64_t rosNs, std::uint64_t & qtimerNs)
{
  const HcmClockSample now = SampleHcmClocks();
  std::uint64_t result = 0;
  if (now.realtimeNs >= now.qtimerNs) {
    const std::uint64_t offset = now.realtimeNs - now.qtimerNs;
    if (rosNs <= offset)
      return ConversionResult::Failure("ROS timestamp is not after the HCM clock origin");
    result = rosNs - offset;
  } else {
    const std::uint64_t offset = now.qtimerNs - now.realtimeNs;
    if (rosNs > std::numeric_limits<std::uint64_t>::max() - offset)
      return ConversionResult::Failure("ROS timestamp overflows the HCM time base");
    result = rosNs + offset;
  }
  qtimerNs = result;
  return ConversionResult::Success();
}

// --- Image Contract <-> Camera frame descriptor -------------------------------

namespace
{

QC::QCImageFormat_e ImageFormat(const std::string & encoding)
{
  if (encoding == "rgb8")
    return QC::QC_IMAGE_FORMAT_RGB888;
  if (encoding == "bgr8")
    return QC::QC_IMAGE_FORMAT_BGR888;
  if (encoding == "nv12")
    return QC::QC_IMAGE_FORMAT_NV12;
  return QC::QC_IMAGE_FORMAT_MAX;
}

const char * ImageEncoding(QC::QCImageFormat_e format)
{
  switch (format) {
    case QC::QC_IMAGE_FORMAT_RGB888:
      return "rgb8";
    case QC::QC_IMAGE_FORMAT_BGR888:
      return "bgr8";
    case QC::QC_IMAGE_FORMAT_NV12:
      return "nv12";
    default:
      return "";
  }
}

/**
 * @brief The host's own layout of `descriptor` for an image shaped like `source`.
 *
 * A QCNode-allocated Image descriptor already describes its storage: planes of
 * `stride` x `actualHeight` rows, each starting right after the previous plane's
 * `planeBufSize` (stride * actualHeight when zero). It must hold the Contract's
 * format, size and plane count, and fit in the buffer past `offset`.
 */
ConversionResult ResolveNativeLayout(const ImageHcmDescriptor & descriptor,
    const ImageHcmContract & image,
    const ImageLayout & source,
    ImageLayout & native)
{
  if (descriptor.format != ImageFormat(image.encoding()) || descriptor.width != image.width() ||
      descriptor.height != image.height()) {
    return ConversionResult::Failure(
        "HCM Image descriptor format or size does not match the Contract");
  }
  if (descriptor.numPlanes != source.planeCount || descriptor.batchSize > 1) {
    return ConversionResult::Failure(
        "HCM Image descriptor must hold one frame with the Contract's plane count");
  }

  ImageLayout value = source;
  value.storageSize = 0;
  for (std::size_t i = 0; i < value.planeCount; ++i) {
    ImagePlaneLayout & plane = value.planes[i];
    const std::uint64_t stride = descriptor.stride[i];
    const std::uint64_t actualHeight = descriptor.actualHeight[i];
    const std::uint64_t minimumSize = stride * actualHeight;
    const std::uint64_t bufferSize =
        descriptor.planeBufSize[i] != 0 ? descriptor.planeBufSize[i] : minimumSize;
    if (stride < plane.rowBytes || actualHeight < plane.rows || bufferSize < minimumSize) {
      return ConversionResult::Failure(
          "HCM Image descriptor plane cannot hold the Contract's rows");
    }
    plane.offset = value.storageSize;
    plane.stride = stride;
    plane.actualHeight = actualHeight;
    plane.bufferSize = bufferSize;
    value.storageSize += bufferSize;
  }
  if (descriptor.offset > descriptor.size ||
      value.storageSize > descriptor.size - descriptor.offset) {
    return ConversionResult::Failure("HCM Image descriptor buffer is too small for its layout");
  }
  native = value;
  return ConversionResult::Success();
}

}  // namespace

template <>
std::unique_ptr<QC::Memory::QCBufferDescriptorBase_t> CreateHcmDescriptor<ImageHcmDescriptor>()
{
  auto descriptor = std::make_unique<ImageHcmDescriptor>();
  descriptor->timestamp = 0;
  descriptor->timestampQGPTP = 0;
  descriptor->frameIdx = 0;
  descriptor->flags = 0;
  descriptor->streamId = 0;
  return descriptor;
}

template <>
ConversionResult ConvertContractToHcm<ImageHcmContract, ImageHcmDescriptor>(
    const ImageHcmContract & image,
    const DmaBufView & payload,
    ImageHcmDescriptor & descriptor)
{
  ImageLayout layout;
  ConversionResult validation = ResolveImageLayout(image, payload.size, layout);
  if (!validation)
    return validation;

  // The Contract header.stamp is ROS time; the HCM descriptor carries the qtimer.
  std::uint64_t rosNs = 0;
  ConversionResult time = ContractTimeToNanos(image.header().stamp(), rosNs);
  if (!time)
    return time;
  if (rosNs == 0)
    return ConversionResult::Failure("Image requires a non-zero source timestamp");
  std::uint64_t qtimerNs = 0;
  time = RosTimestampToHcm(rosNs, qtimerNs);
  if (!time)
    return time;

  if (payload.data == nullptr)
    return ConversionResult::Failure("source Image DMA-BUF is missing");
  if (descriptor.pBuf == nullptr)
    return ConversionResult::Failure("HCM Image descriptor buffer is missing");

  // A descriptor that describes no layout adopts the source layout and receives
  // the whole source storage.
  if (descriptor.numPlanes == 0) {
    if (descriptor.offset > descriptor.size ||
        layout.storageSize > descriptor.size - descriptor.offset) {
      return ConversionResult::Failure("HCM Image descriptor buffer is too small");
    }
    descriptor.format = ImageFormat(image.encoding());
    descriptor.batchSize = 1;
    descriptor.width = image.width();
    descriptor.height = image.height();
    descriptor.numPlanes = static_cast<std::uint32_t>(layout.planeCount);
    for (std::size_t i = 0; i < QC_NUM_IMAGE_PLANES; ++i) {
      const bool used = i < layout.planeCount;
      descriptor.stride[i] = used ? static_cast<std::uint32_t>(layout.planes[i].stride) : 0;
      descriptor.actualHeight[i] =
          used ? static_cast<std::uint32_t>(layout.planes[i].actualHeight) : 0;
      descriptor.planeBufSize[i] =
          used ? static_cast<std::uint32_t>(layout.planes[i].bufferSize) : 0;
    }
    descriptor.timestamp = qtimerNs;
    std::memcpy(
        descriptor.GetDataPtr(), payload.data, static_cast<std::size_t>(layout.storageSize));
    descriptor.validSize = static_cast<std::size_t>(layout.storageSize);
    return ConversionResult::Success();
  }

  // A described descriptor keeps the host's layout: the valid rows of every plane
  // are copied into it at the host's own stride and plane offsets.
  ImageLayout native;
  validation = ResolveNativeLayout(descriptor, image, layout, native);
  if (!validation)
    return validation;
  const auto * from = static_cast<const std::uint8_t *>(payload.data);
  auto * to = static_cast<std::uint8_t *>(descriptor.GetDataPtr());
  for (std::size_t i = 0; i < layout.planeCount; ++i) {
    const ImagePlaneLayout & src = layout.planes[i];
    const ImagePlaneLayout & dst = native.planes[i];
    for (std::uint64_t row = 0; row < src.rows; ++row) {
      std::memcpy(to + dst.offset + row * dst.stride, from + src.offset + row * src.stride,
          static_cast<std::size_t>(src.rowBytes));
    }
  }
  descriptor.batchSize = 1;
  descriptor.timestamp = qtimerNs;
  descriptor.validSize = static_cast<std::size_t>(native.storageSize);
  return ConversionResult::Success();
}

template <>
ConversionResult ConvertHcmToContract<ImageHcmContract, ImageHcmDescriptor>(
    const ImageHcmDescriptor & descriptor,
    ImageHcmContract & image)
{
  if (descriptor.timestamp == 0)
    return ConversionResult::Failure("Image requires a non-zero source timestamp");
  std::uint64_t rosNs = 0;
  ConversionResult time = HcmTimestampToRos(descriptor.timestamp, rosNs);
  if (!time)
    return time;
  if (descriptor.batchSize != 1 || descriptor.numPlanes == 0 || descriptor.numPlanes > 2)
    return ConversionResult::Failure("HCM Image requires one frame and one or two planes");
  if (descriptor.pBuf == nullptr || descriptor.offset > descriptor.size ||
      descriptor.validSize > descriptor.size - descriptor.offset) {
    return ConversionResult::Failure("HCM Image payload storage is inconsistent");
  }

  ImageHcmContract value;
  value.set_width(descriptor.width);
  value.set_height(descriptor.height);
  value.set_encoding(ImageEncoding(descriptor.format));
  ImageLayout canonical;
  ConversionResult supported =
      CanonicalImageLayout(descriptor.width, descriptor.height, value.encoding(), canonical);
  if (!supported)
    return supported;
  value.set_step(static_cast<std::uint32_t>(canonical.step));
  value.set_batch_size(1);
  for (std::size_t i = 0; i < descriptor.numPlanes; ++i) {
    auto * plane = value.add_planes();
    plane->set_stride(descriptor.stride[i]);
    plane->set_actual_height(descriptor.actualHeight[i]);
    plane->set_buffer_size(descriptor.planeBufSize[i]);
  }
  time = NanosToContractTime(rosNs, *value.mutable_header()->mutable_stamp());
  if (!time)
    return time;
  ConversionResult validation = ValidateImageLayout(value, descriptor.validSize);
  if (!validation)
    return validation;
  image = std::move(value);
  return ConversionResult::Success();
}

}  // namespace qrb::adapter
