#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <filesystem> 
#include <Instruction.h>
#include <InstructionBatch.h>
#include <InstructionBuffer.h>
#include <InstructionServiceIface.h>
#include <CommunicationServicesIface.h>
#include <DeviceMonitorServiceIface.h>
#include <DataServicesIface.h>

using namespace com::common;
using namespace com::master;
using namespace com::types;

std::shared_ptr<com::common::DataServicesIface> dataServices = com::common::DataServicesIface::Get();

void slave_callback(ClientId clientId, PortId portId, BufferDescriptor buffer)
{   
    InstructionBuffer* receive = reinterpret_cast<InstructionBuffer*>(buffer.GetBufferPtr());
    int sizeIncomingBuf = buffer.GetSize();
    uint32_t instnumber = receive->GetNumOfInstructions();
    auto instructions = receive->GetInstructions();
    
    for (auto instruction: instructions)
        if (instruction->GetSectionId() == 3042 && instruction->GetId() == 20)
        {
            std::cout << "Base user interface major version for UMRR96 set!"  << std::endl;
            instruction->SetResponse(COM_INSTR_PORT_SUCCESS);
            instruction->SetValue(1);
        }
        else if (instruction->GetSectionId() == 3042 && instruction->GetId() == 21)
        {
            std::cout << "Base user interface minor version for UMRR96 set!"  << std::endl;
            instruction->SetResponse(COM_INSTR_PORT_SUCCESS);
            instruction->SetValue(0);
        }
        else if (instruction->GetSectionId() == 3042 && instruction->GetId() == 22)
        {
            std::cout << "User interface identifier for UMRR96 set!" << std::endl;
            instruction->SetResponse(COM_INSTR_PORT_SUCCESS);
            instruction->SetValue(5);
        }
        else if (instruction->GetSectionId() == 3042 && instruction->GetId() == 23)
        {
            std::cout << "User interface major version for UMRR96 set!" << std::endl;
            instruction->SetResponse(COM_INSTR_PORT_SUCCESS);
            instruction->SetValue(1);
        }
        else if (instruction->GetSectionId() == 3042 && instruction->GetId() == 24)
        {
            std::cout << "User interface minor version for UMRR96 set!" << std::endl;
            instruction->SetResponse(COM_INSTR_PORT_SUCCESS);
            instruction->SetValue(2);
        }
        else
        {
            std::cout << "Unknown instruction received from ROS driver!" << std::endl;
        }
     
    dataServices->SetInstructionBuffer(clientId ,*receive, nullptr);
}


int main()
{
    if(!dataServices->Init())
    {
        throw std::runtime_error("Data services have not been initialized!");
    }
    
    dataServices->RegisterInstRecvCallback(slave_callback);
    
    while(1)
    {   
        ClientId masterId = 1;
        PortId portTargetListId = 66;
        std::string portFile = "/code/umrr96_simulator/port_radar_targets.bin";
        std::ifstream ifs (portFile, std::ifstream::binary | std::ios::binary);
        std::filebuf* pbuf = ifs.rdbuf();
        int size = pbuf->pubseekoff (0, ifs.end, ifs.in);
        pbuf->pubseekpos (0, ifs.in);
        char* filebuffer=new (std::nothrow) char[size];
        if (filebuffer == nullptr) {
            std::cout << "error assigning memory!" << std::endl;    
        }
        pbuf->sgetn (filebuffer, size);
        BufferDescriptor bufferdesc((uint8_t*)filebuffer, size);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if(ERROR_CODE_OK !=  dataServices->StreamDataPort(masterId, portTargetListId, bufferdesc))
        {
            return -1;
        }

        std::cout << "sensor UMRR96 is transmitting data! " << std::endl;
        ifs.close();
        delete[] filebuffer;

    }
    return 0;
}
