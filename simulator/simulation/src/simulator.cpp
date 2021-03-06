#include <CommunicationServicesIface.h>
#include <DataServicesIface.h>
#include <DeviceMonitorServiceIface.h>
#include <Instruction.h>
#include <InstructionBatch.h>
#include <InstructionBuffer.h>
#include <InstructionServiceIface.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>

using namespace com::common;
using namespace com::master;
using namespace com::types;

uint64_t identifier;
uint64_t majorVersion;
uint64_t minorVersion;

std::shared_ptr<com::common::DataServicesIface> dataServices =
    com::common::DataServicesIface::Get();

void slave_callback(ClientId clientId, PortId, BufferDescriptor buffer) {
  InstructionBuffer *receive =
      reinterpret_cast<InstructionBuffer *>(buffer.GetBufferPtr());
  int sizeIncomingBuf = buffer.GetSize();
  uint32_t instnumber = receive->GetNumOfInstructions();
  auto instructions = receive->GetInstructions();

  for (auto instruction : instructions) {
    if (instruction->GetSectionId() == 3042 && instruction->GetId() == 20) {
      std::cout << "Base user interface major version set!" << std::endl;
      instruction->SetResponse(COM_INSTR_PORT_SUCCESS);
      instruction->SetValue(1);
    } else if (instruction->GetSectionId() == 3042 &&
               instruction->GetId() == 21) {
      std::cout << "Base user interface minor version set!" << std::endl;
      instruction->SetResponse(COM_INSTR_PORT_SUCCESS);
      instruction->SetValue(0);
    } else if (instruction->GetSectionId() == 3042 &&
               instruction->GetId() == 22) {
      std::cout << "User interface identifier set!" << std::endl;
      instruction->SetResponse(COM_INSTR_PORT_SUCCESS);
      instruction->SetValue(identifier);
    } else if (instruction->GetSectionId() == 3042 &&
               instruction->GetId() == 23) {
      std::cout << "User interface major version set!" << std::endl;
      instruction->SetResponse(COM_INSTR_PORT_SUCCESS);
      instruction->SetValue(majorVersion);
    } else if (instruction->GetSectionId() == 3042 &&
               instruction->GetId() == 24) {
      std::cout << "User interface minor version set!" << std::endl;
      instruction->SetResponse(COM_INSTR_PORT_SUCCESS);
      instruction->SetValue(minorVersion);
    } else if (instruction->GetSectionId() == 2010 &&
               instruction->GetId() == 0) {
      std::cout << "Instruction for mode change tx_antenna answered!"
                << std::endl;
      instruction->SetResponse(COM_INSTR_PORT_SUCCESS);
    } else if (instruction->GetSectionId() == 2010 &&
               instruction->GetId() == 2) {
      std::cout << "Instruction for mode change sweep_idx answered!"
                << std::endl;
      instruction->SetResponse(COM_INSTR_PORT_SUCCESS);
    } else {
      std::cout << "Unknown instruction received from ROS driver!" << std::endl;
    }
  }

  dataServices->SetInstructionBuffer(clientId, *receive, nullptr);
}

int main(int argc, char *argv[]) {
  if (argc != 4) {
    std::cout << "Specifiy User Interface for the sensor" << std::endl;
    return 1;
  }

  identifier = strtoll(argv[1], nullptr, 10);
  majorVersion = strtoll(argv[2], nullptr, 10);
  minorVersion = strtoll(argv[3], nullptr, 10);

  if (!dataServices->Init()) {
    throw std::runtime_error("Data services have not been initialized!");
  }

  dataServices->RegisterInstRecvCallback(slave_callback);
  auto Start = std::chrono::steady_clock::now();

  while (1) {
    ClientId masterId = 1;
    PortId portTargetListId = 66;
    std::string portFile = "/code/simulator/targetlist_port.bin";
    std::ifstream ifs(portFile, std::ifstream::binary | std::ios::binary);
    std::filebuf *pbuf = ifs.rdbuf();
    int size = pbuf->pubseekoff(0, ifs.end, ifs.in);
    pbuf->pubseekpos(0, ifs.in);
    char *filebuffer = new (std::nothrow) char[size];

    if (filebuffer == nullptr) {
      std::cout << "error assigning memory!" << std::endl;
    }

    pbuf->sgetn(filebuffer, size);
    BufferDescriptor bufferdesc((uint8_t *)filebuffer, size);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (ERROR_CODE_OK !=
        dataServices->StreamDataPort(masterId, portTargetListId, bufferdesc)) {
      return -1;
    }

    std::cout << "sensor is transmitting data! " << std::endl;
    ifs.close();
    delete[] filebuffer;

    if (std::chrono::steady_clock::now() - Start > std::chrono::seconds(40))
      break;
  }
  return 0;
}
