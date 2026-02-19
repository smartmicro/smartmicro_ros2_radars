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

class UpdateService {
    public:
        UpdateService();
        ~UpdateService() = default;

        void StartSoftwareUpdate(com::types::ClientId client_id, std::string &update_image);
    
    private:
        void UpdateCallback(com::types::SWUpdateInfo &info);
        void HandleResult();

        com::types::SWUpdateInfo updateInfo_;
        std::mutex mutex_;
        std::condition_variable cv_;
};

#endif // UPDATE_SERVICE_HPP
