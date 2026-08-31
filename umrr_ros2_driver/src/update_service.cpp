// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// The initial version of the code was developed by Apex.AI and
// was thereafter adapted and extended by smartmicro.

#include "umrr_ros2_driver/update_service.hpp"

#include <chrono>
#include <fstream>
#include <string>
#include <thread>

using com::master::CommunicationServicesIface;

UpdateService::UpdateService() {}

UpdateResult UpdateService::StartSoftwareUpdate(
  com::types::ClientId client_id,
  std::string & update_image)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (update_in_progress_) {
      RCLCPP_WARN(
        rclcpp::get_logger("FirmwareUpdater"),
        "Firmware update request rejected: another update is already in progress.");
      return UpdateResult::kBusy;
    }
    update_in_progress_ = true;
  }

  std::ifstream fileStream(update_image, std::ios::binary | std::ios::ate);
  if (!fileStream.is_open()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("FirmwareUpdater"), "Couldn't open file: %s", update_image.c_str());
    std::lock_guard<std::mutex> lock(mutex_);
    update_in_progress_ = false;
    return UpdateResult::kFileOpenError;
  }

  const auto file_end_pos = fileStream.tellg();
  if (file_end_pos < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("FirmwareUpdater"), "Failed to determine file size for: %s",
      update_image.c_str());
    std::lock_guard<std::mutex> lock(mutex_);
    update_in_progress_ = false;
    return UpdateResult::kFileSizeError;
  }
  const uint64_t totalSize = static_cast<uint64_t>(file_end_pos);

  auto comServicesPtr = CommunicationServicesIface::Get();
  auto updateService = comServicesPtr->GetUpdateService();
  if (!updateService) {
    RCLCPP_ERROR(rclcpp::get_logger("FirmwareUpdater"), "Update service is not available");
    std::lock_guard<std::mutex> lock(mutex_);
    update_in_progress_ = false;
    return UpdateResult::kServiceUnavailable;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    updateInfo_.SetUpdateStatus(com::types::RUNNING);
    updateInfo_.SetCurrentDownloadedBytes(0);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("UpdateService"), "Starting firmware download of %lu bytes...", totalSize);

  if (updateService->SoftwareUpdate(
      update_image, client_id, [this](com::types::SWUpdateInfo & info) {
        this->UpdateCallback(info);
      }) != com::types::ERROR_CODE_OK)
  {
    RCLCPP_ERROR(rclcpp::get_logger("FirmwareUpdater"), "Start of software download failed");
    std::lock_guard<std::mutex> lock(mutex_);
    update_in_progress_ = false;
    return UpdateResult::kStartFailed;
  }

  constexpr auto kUpdateTimeout = std::chrono::minutes(5);
  bool finished = false;
  {
    std::unique_lock<std::mutex> lock(
      mutex_);
    finished = cv_.wait_for(
      lock,
      kUpdateTimeout,
      [this,
      totalSize] {
        return updateInfo_.GetUpdateStatus() != com::types::RUNNING ||
        updateInfo_.GetCurrentDownloadedBytes() >= totalSize;
      });
  }

  if (!finished) {
    RCLCPP_ERROR(rclcpp::get_logger("FirmwareUpdater"), "Firmware download timed out. Aborting.");
    updateService->AbortSoftwareUpdate();
    std::lock_guard<std::mutex> lock(mutex_);
    updateInfo_.SetUpdateStatus(com::types::STOPPED_BY_ERROR_TIMEOUT);
    update_in_progress_ = false;
    return UpdateResult::kTimeout;
  }

  const auto result = HandleResult();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    update_in_progress_ = false;
  }

  return result;
}

void UpdateService::UpdateCallback(com::types::SWUpdateInfo & info)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    updateInfo_ = info;
    RCLCPP_INFO(
      rclcpp::get_logger("FirmwareUpdater"), "Downloaded %lu bytes...",
      info.GetCurrentDownloadedBytes());
  }
  cv_.notify_one();
}

UpdateResult UpdateService::HandleResult()
{
  std::lock_guard<std::mutex> lock(mutex_);
  switch (updateInfo_.GetUpdateStatus()) {
    case com::types::READY_SUCCESS:
      RCLCPP_INFO(
        rclcpp::get_logger("FirmwareUpdater"), "Firmware download completed successfully.");
      return UpdateResult::kSuccess;
    case com::types::STOPPED_BY_MASTER:
      RCLCPP_WARN(rclcpp::get_logger("FirmwareUpdater"), "Download stopped by master.");
      return UpdateResult::kStoppedByMaster;
    case com::types::STOPPED_BY_SLAVE:
      RCLCPP_ERROR(rclcpp::get_logger("FirmwareUpdater"), "Download stopped by slave.");
      return UpdateResult::kStoppedBySlave;
    case com::types::STOPPED_BY_ERROR_TIMEOUT:
      RCLCPP_ERROR(rclcpp::get_logger("FirmwareUpdater"), "Download failed: timeout.");
      return UpdateResult::kTimeout;
    case com::types::STOPPED_BY_ERROR_BLOCK_REPEAT:
      RCLCPP_ERROR(rclcpp::get_logger("FirmwareUpdater"), "Download failed: block repeat error.");
      return UpdateResult::kBlockRepeatError;
    case com::types::STOPPED_BY_ERROR_IMAGE_INVALID:
      RCLCPP_ERROR(rclcpp::get_logger("FirmwareUpdater"), "Download failed: invalid image.");
      return UpdateResult::kImageInvalid;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("FirmwareUpdater"), "Download failed: unknown error.");
      return UpdateResult::kUnknownError;
  }
}
