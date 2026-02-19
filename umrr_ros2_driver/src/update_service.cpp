#include "umrr_ros2_driver/update_service.hpp"

#include <fstream>
#include <thread>

using namespace com::types;
using namespace com::master;

UpdateService::UpdateService() {}

void UpdateService::StartSoftwareUpdate(ClientId client_id, std::string & update_image)
{
  std::ifstream fileStream(update_image, std::ios::binary | std::ios::ate);
  if (!fileStream.is_open()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("FirmwareUpdater"), "Couldn't open file: %s", update_image.c_str());
    return;
  }

  const uint64_t totalSize = fileStream.tellg();

  auto comServicesPtr = CommunicationServicesIface::Get();
  auto updateService = comServicesPtr->GetUpdateService();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    updateInfo_.SetUpdateStatus(RUNNING);
    updateInfo_.SetCurrentDownloadedBytes(0);
  }  // mutex(unlocked)

  RCLCPP_INFO(
    rclcpp::get_logger("UpdateService"), "Starting firmware download of %lu bytes...", totalSize);

  if (updateService->SoftwareUpdate(update_image, client_id, [this](SWUpdateInfo & info) {
        this->UpdateCallback(info);
      }) != ERROR_CODE_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("FirmwareUpdater"), "Start of software download failed");
    return;
  }

  {
    std::unique_lock<std::mutex> lock(
      mutex_);
    cv_.wait(
      lock,
      [this,
       totalSize] {
        return updateInfo_.GetUpdateStatus() != RUNNING ||
               updateInfo_.GetCurrentDownloadedBytes() >= totalSize;
      });
  }

  HandleResult();
}

void UpdateService::UpdateCallback(SWUpdateInfo & info)
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

void UpdateService::HandleResult()
{
  std::lock_guard<std::mutex> lock(mutex_);
  switch (updateInfo_.GetUpdateStatus()) {
    case READY_SUCCESS:
      RCLCPP_INFO(
        rclcpp::get_logger("FirmwareUpdater"), "Firmware download completed successfully.");
      break;
    case STOPPED_BY_MASTER:
      RCLCPP_WARN(rclcpp::get_logger("FirmwareUpdater"), "Download stopped by master.");
      break;
    case STOPPED_BY_SLAVE:
      RCLCPP_ERROR(rclcpp::get_logger("FirmwareUpdater"), "Download stopped by slave.");
      break;
    case STOPPED_BY_ERROR_TIMEOUT:
      RCLCPP_ERROR(rclcpp::get_logger("FirmwareUpdater"), "Download failed: timeout.");
      break;
    case STOPPED_BY_ERROR_BLOCK_REPEAT:
      RCLCPP_ERROR(rclcpp::get_logger("FirmwareUpdater"), "Download failed: block repeat error.");
      break;
    case STOPPED_BY_ERROR_IMAGE_INVALID:
      RCLCPP_ERROR(rclcpp::get_logger("FirmwareUpdater"), "Download failed: invalid image.");
      break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("FirmwareUpdater"), "Download failed: unknown error.");
      break;
  }
}
