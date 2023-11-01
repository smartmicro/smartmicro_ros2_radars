#include <chrono>
#include <execinfo.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include "umrr_ros2_driver/UpdateService.hpp"
#include <rclcpp/rclcpp.hpp>

#include <CommunicationServicesIface.h>
#include <DeviceMonitorServiceIface.h>
#include <Instruction.h>
#include <InstructionBatch.h>
#include <InstructionServiceIface.h>
#include <UpdateServiceIface.h>

using namespace com::common;
using namespace com::types;
using namespace com::master;

static SWUpdateInfo UpdateInfoStatic;

void MyUpdateServiceInfoHandler(SWUpdateInfo &updateInfoLocal) {
  UpdateInfoStatic = updateInfoLocal;
  uint64_t downloadedBytes = updateInfoLocal.GetCurrentDownloadedBytes();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Downloaded bytes: %lu", downloadedBytes);
}

void StartSoftwareUpdate(ClientId client_id, std::string update_image) {
  std::ifstream fileStream(update_image, std::ios::binary | std::ios::ate);
  if (!fileStream.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Couldn't open File");
    return;
  }
  const uint64_t size = fileStream.tellg();
  fileStream.close();

  auto comServicesPtr = com::master::CommunicationServicesIface::Get();
  auto UpdateServicePtr = comServicesPtr->GetUpdateService();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start Download of the firmware");

  UpdateInfoStatic.SetUpdateStatus(RUNNING);
  UpdateInfoStatic.SetCurrentDownloadedBytes(0);
  bool readyFlag = false;

  if (UpdateServicePtr->SoftwareUpdate(update_image, client_id,
                                       MyUpdateServiceInfoHandler) !=
      ERROR_CODE_OK) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start of SW-Download failed");
  }

  if (UpdateInfoStatic.GetCurrentDownloadedBytes() == size) {
    std::this_thread::sleep_for(std::chrono::seconds(3));
    readyFlag = true;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  while (UpdateInfoStatic.GetUpdateStatus() == RUNNING && !readyFlag) {
    // Waiting for the update to finish
  }

  switch (UpdateInfoStatic.GetUpdateStatus()) {
  case READY_SUCCESS:
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successful download of firmware.");
    //std::system("clear");
    break;
  case STOPPED_BY_MASTER:
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Download is stopped by Master.");
    break;
  case STOPPED_BY_SLAVE:
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Download is stopped by Slave.");
    break;
  case STOPPED_BY_ERROR_TIMEOUT:
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                "Download-Error caused by timeout.");
    break;
  case STOPPED_BY_ERROR_BLOCK_REPEAT:
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                "Download-Error caused by Block-Repeat problem.");
    break;
  case STOPPED_BY_ERROR_IMAGE_INVALID:
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                "Download-Error caused by Invalid-Image problem.");
    break;
  default:
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                "Download-Error caused by Unknown-Reason.");
    break;
  }
}
