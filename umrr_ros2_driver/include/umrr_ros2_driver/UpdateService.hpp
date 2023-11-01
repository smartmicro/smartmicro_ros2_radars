#pragma once

#include <CommunicationServicesIface.h>
#include <DeviceMonitorServiceIface.h>
#include <Instruction.h>
#include <InstructionBatch.h>
#include <InstructionServiceIface.h>
#include <UpdateServiceIface.h>

using namespace com::common;
using namespace com::types;
using namespace com::master;

void MyUpdateServiceInfoHandler(SWUpdateInfo &updateInfoLocal);
void StartSoftwareUpdate(ClientId client_id, std::string update_image);
