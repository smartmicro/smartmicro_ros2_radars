#ifndef UPDATE_SERVICE_HPP
#define UPDATE_SERVICE_HPP

#include <condition_variable>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <CommunicationServicesIface.h>
#include <DeviceMonitorServiceIface.h>
#include <Instruction.h>
#include <InstructionBatch.h>
#include <InstructionServiceIface.h>
#include <UpdateServiceIface.h>

enum class UpdateResult {
    kSuccess,
    kBusy,
    kFileOpenError,
    kFileSizeError,
    kServiceUnavailable,
    kStartFailed,
    kTimeout,
    kStoppedByMaster,
    kStoppedBySlave,
    kBlockRepeatError,
    kImageInvalid,
    kUnknownError
};

class UpdateService {
    public:
        UpdateService();
        ~UpdateService() = default;

                UpdateResult StartSoftwareUpdate(com::types::ClientId client_id, std::string &update_image);
    
    private:
        void UpdateCallback(com::types::SWUpdateInfo &info);
        UpdateResult HandleResult();

        com::types::SWUpdateInfo updateInfo_;
        std::mutex mutex_;
        std::condition_variable cv_;
        bool update_in_progress_{false};
};

#endif // UPDATE_SERVICE_HPP
