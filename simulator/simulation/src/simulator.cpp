// Copyright (c) 2021, s.m.s, smart microwave sensors GmbH, Brunswick, Germany.
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

#include <CommunicationServicesIface.h>
#include <DataServicesIface.h>
#include <DeviceMonitorServiceIface.h>
#include <Instruction.h>
#include <InstructionBatch.h>
#include <InstructionBuffer.h>
#include <InstructionServiceIface.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace com::common;
using namespace com::master;
using namespace com::types;

namespace
{
uint64_t identifier;
uint64_t majorVersion;
uint64_t minorVersion;
std::string port;
std::string portFile;
}  // namespace

std::shared_ptr<com::common::DataServicesIface> dataServices =
  com::common::DataServicesIface::Get();

void slave_callback(ClientId clientId, PortId, BufferDescriptor buffer)
{
  if (buffer.GetBufferPtr() == nullptr || buffer.GetSize() == 0U) {
    std::cout << "Invalid instruction buffer received!" << std::endl;
    return;
  }

  InstructionBuffer * receive = reinterpret_cast<InstructionBuffer *>(buffer.GetBufferPtr());
  uint32_t instnumber = receive->GetNumOfInstructions();
  auto instructions = receive->GetInstructions();

  for (auto instruction : instructions) {
    if (instruction->GetSectionId() == 2010 && instruction->GetId() == 2) {
        std::cout << "UMRR96 mode frequency_sweep set!" << std::endl;
        instruction->SetResponse(COM_INSTR_PORT_SUCCESS);
    } else if (instruction->GetSectionId() == 2010 && instruction->GetId() == 5) {
        std::cout << "UMRR9F mode range_toggle_mode get!" << std::endl;
        instruction->SetResponse(COM_INSTR_PORT_SUCCESS);
    } else if (instruction->GetSectionId() == 2010 && instruction->GetId() == 4) {
        std::cout << "UMRR11 mode angular_separation set!" << std::endl;
        instruction->SetResponse(COM_INSTR_PORT_SUCCESS);
    } else if (instruction->GetSectionId() == 2012 && instruction->GetId() == 3) {
        std::cout << "Software major version read!" << std::endl;
        instruction->SetResponse(COM_INSTR_PORT_SUCCESS);
    } else if (instruction->GetSectionId() == 2012 && instruction->GetId() == 4) {
        std::cout << "Software minorr version read!" << std::endl;
        instruction->SetResponse(COM_INSTR_PORT_SUCCESS);
    } else {
        std::cout << "Unknown instruction received from ROS driver!" << std::endl;
    }
  }

  dataServices->SetInstructionBuffer(clientId, *receive, nullptr);
}

void stream_port(std::string portFile)
{
  dataServices->RegisterInstRecvCallback(slave_callback);

  ClientId masterId = 1;
  PortId portTargetListId = 66;
  std::ifstream ifs(portFile, std::ios::binary | std::ios::ate);
  if (!ifs.is_open()) {
    std::cout << "could not open port file: " << portFile << std::endl;
    return;
  }

  const auto file_end_pos = ifs.tellg();
  if (file_end_pos <= 0) {
    std::cout << "invalid or empty port file: " << portFile << std::endl;
    return;
  }

  const auto size = static_cast<size_t>(file_end_pos);
  std::vector<uint8_t> filebuffer(size);

  ifs.seekg(0, std::ios::beg);
  if (!ifs.read(reinterpret_cast<char *>(filebuffer.data()), static_cast<std::streamsize>(size))) {
    std::cout << "failed reading port file: " << portFile << std::endl;
    return;
  }

  BufferDescriptor bufferdesc(filebuffer.data(), size);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  if (ERROR_CODE_OK != dataServices->StreamDataPort(masterId, portTargetListId, bufferdesc)) {
    return;
  }

  std::cout << "sensor is transmitting data! " << std::endl;
}

int main(int argc, char * argv[])
{
  if (argc != 5) {
    std::cout << "Specifiy User Interface for the sensor" << std::endl;
    return 1;
  }

  identifier = strtoll(argv[1], nullptr, 10);
  majorVersion = strtoll(argv[2], nullptr, 10);
  minorVersion = strtoll(argv[3], nullptr, 10);
  port = argv[4];

  if (!dataServices->Init()) {
    throw std::runtime_error("Data services have not been initialized!");
  }

  auto Start = std::chrono::steady_clock::now();

  while (1) {
    if (port == "A") {
      std::string portFile = "/code/simulator/targetlist_port_v2_1_0.bin";
      stream_port(portFile);
    } else if (port == "B") {
      std::string portFile = "/code/simulator/targetlist_port_v3_0_0.bin";
      stream_port(portFile);
    } else if (port == "C") {
      std::string portFile = "/code/simulator/targetlist_port_v4_0_0.bin";
      stream_port(portFile);
    } else if (port == "D") {
      std::string portFile = "/code/simulator/targetlist_port_v4_1_0.bin";
      stream_port(portFile);
    } else {
      std::cout << "Invalid input!" << std::endl;
    }
    if (std::chrono::steady_clock::now() - Start > std::chrono::seconds(15)) {
      break;
    }
  }
  return 0;
}
