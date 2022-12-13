/************************************************************************
 *
 * File: DataServicesIface.h
 *
 * Copyright
 *
 * s.m.s. smart microwave sensors GmbH
 * In den Waashainen 1
 * 38108 Braunschweig - Germany
 *
 * Register Court: Amtsgericht Braunschweig / Register Number: HRB 4525
 * VAT ID: DE 193324798 / Tax Number: 13/207/02357
 ************************************************************************/

#ifndef COM_COMMON_DATA_SERVICES_IF_H
#define COM_COMMON_DATA_SERVICES_IF_H


#include <Types.h>
#include <VersionedIface.h>
#include <ExternalTypes.h>
#include <InstructionBuffer.h>

#include <functional>

using namespace com::types;

namespace com {
namespace common {


typedef std::function<void(ClientId clientId,PortId,BufferDescriptor)> DataReceiverCallback;
typedef std::function<void(ClientId clientId,InstructionBuffer&)> ResponseCallback;
typedef std::function<void(const std::string&)> UpdateCallback;
typedef std::function<bool(uint64_t&)> GetLocalTimeCallback;
typedef std::function<bool(int64_t)> SetTimeOffsetCallback;
typedef std::function<void(SWUpdateInfo&)> OnUpdateInfoCallback;
typedef std::function<void(ClientId id, bool clientStatus)> ConnectedInfoCallback;


class EXPORT_COM_LIB DataServicesIface : public com::VersionedIface {

public:

    DataServicesIface() {}
    virtual ~DataServicesIface() {}

    /*  
     * Function: Get()
     * Arguments: none
     * Return: std::shared_ptr<DataServicesIface>
     * Description: Returns a singletone instance of a this class, since this is only
     *    an abstract class, the instantiation should be done by a deriving class.
     */
    static std::shared_ptr<DataServicesIface> Get();

    /*  
     * Function: SetInstructionBuffer 
     * Arguments: IN ClientId clientId - destination client id.
     *            IN const InstructionBuffer& instBuffer - instruction port object.
     *            IN ResponseCallback callback - callback function which should be
     *                                           called upon a response.
     *
     * Return: ErrorCode
     * Description: This function sends an instruction port to the input client. It is used
     *     to send requests and responses. In the case of a response, no callback is required.
     */
    virtual ErrorCode SetInstructionBuffer(IN ClientId clientId,
                                           IN InstructionBuffer& instBuffer,
                                           IN ResponseCallback callback) = 0;
    /*  
     * Function: RegisterDataRecvCallback
     * Arguments: IN ClientId clientId - client id of the sending client
     *            IN PortId portId - port id of the required port 
     *            IN DataReceiverCallback callback - callback function which should be 
     *                                               called upon a reception.
     * Return: ErrorCode
     * Description: Registers a receiver callback function , which should be called ,
     *    when a new buffer arrives.
     */
    virtual ErrorCode RegisterDataRecvCallback(IN ClientId clientId,
                                               IN PortId portId,
                                               IN DataReceiverCallback callback) = 0;
    /*  
     * Function: RegisterInstRecvCallback
     * Arguments: IN DataReceiverCallback callback - callback function will be called upon 
                                                    a reception of new instruction port.
     * Return: ErrorCode
     * Description: Registers a receiver callback function, which should be called,
           when new instructions arrive.
     */
    virtual ErrorCode RegisterInstRecvCallback(IN DataReceiverCallback callback) = 0;

    /*  
     * Function: StreamDataPort
     * Arguments: IN ClientId clientId - destination client id.
     *            IN PortId   portId - id of the streamed port.
     *            IN BufferDescriptor& buffer - data buffer
     * Return: ErrorCode
     * Description: Streams a data port to input client id
     */
    virtual ErrorCode StreamDataPort(IN ClientId clientId, 
                                      IN PortId   portId,
                                      IN BufferDescriptor& buffer) = 0;

    /*  
     * Function: StreamInternalDataPort
     * Arguments: ClientId clientId - destination client id.
     *            PortId   portId - id of the streamed port.
     *            BufferDescriptor& buffer - data buffer
     * Return: ErrorCode
     * Description: Streams a internal data port to input client id.
     *              This will return with an error code if the port is
     *              content of client user interface
     */
    virtual ErrorCode StreamInternalDataPort(
       ClientId clientId, PortId portId, BufferDescriptor& buffer) = 0;

    /*  
     * Function: Init
     * Arguments: none
     * Return: bool
     * Description: Initializes the data services layer.
     */
    virtual bool Init() = 0;

    /*
     * Function:    RegisterUpdCallback 
     * Arguments:   UpdateCallback
     * Return:      ErrorCode
     * Description: Register an update callback, will be called upon
     *              receiving a new download package
     */
    virtual ErrorCode RegisterUpdCallback(IN UpdateCallback callback) = 0;

    /*
     * Function:    RegisterConnectedClientCallback
     * Arguments:   IN ConnectedInfoCallback callback
     *              Callback-Function which shall be called to inform about 
     *              change in 'connected client list'
     * Return:      ErrorCode
     * Description: Registers the ConnectedClientCallback 
     */
    virtual ErrorCode RegisterConnectedClientCallback(IN ConnectedInfoCallback callback) = 0;

    /*  
     * Function:    RegisterGetLocalTimeCallback
     * Arguments:   GetLocalTimeCallback
     * Return:      ErrorCode
     * Description: Register a callback to receive the local time for the time sync feature.
     *              Valid for time sync master and slave 
     */
    virtual ErrorCode RegisterGetTimeCallback(IN GetLocalTimeCallback callback) = 0;

    /*  
     * Function: RegisterSetTimeOffsetCallback
     * Arguments: SetTimeOffsetCallback
     * Return: ErrorCode
     * Description: Register a callback to set adjust the local time with the offset
     * Valid for time sync slave.
     */
    virtual ErrorCode RegisterSetTimeOffsetCallback(IN SetTimeOffsetCallback callback) = 0;

    /*  
     * Function: AddTimeSyncSlave
     * Arguments: uint8_t slaveDevId slave device id
     * Return: ErrorCode
     * Description: adds a time sync slave to be synchronized 
     * Valid for time sync master
     */
    virtual ErrorCode AddTimeSyncSlave(IN uint8_t slaveDevId) = 0;

    /*  
     * Function: RemoveTimeSyncSlave
     * Arguments: uint8_t slaveDevId slave device id
     * Return: ErrorCode
     * Description: removes a time sync slave. 
     * Valid for time sync master
     */
    virtual ErrorCode RemoveTimeSyncSlave(IN uint8_t slaveDevId) = 0;
    /*  
     * Function: SetTimeSyncSlaveId
     * Arguments: uint8_t slaveDevId slave device id
     * Return: ErrorCode
     * Description: set time sync slave id.
     * Valid for time sync slave
     */
    virtual ErrorCode SetTimeSyncSlaveId(IN uint8_t slaveDevId) = 0;
    /*  
     * Function: ActivateTimeSync
     * Arguments: bool activate - true activates , false diactivates
     * Return: ErrorCode
     * Description: activates/diactivates the time sync protocol
     * Valid for time sync slave and master
     */
    virtual ErrorCode ActivateTimeSync(IN bool activate) = 0;

    /*
     * Function:     StartSWUpdate
     * Arguments:    IN std::map<uint32_t, SWUpdateConfigData> updateList
     *               IN ClientId clientID
     *               IN OnUpdateInfoCallback callback
     * Return :      ErrorCode
     * Description : Function for trigger a new Software-Update 
     *               in Data-Service-Layer
     */
    virtual ErrorCode StartSWUpdate(IN std::map<uint32_t, SWUpdateConfigData>& updateList,
                                    IN ClientId clientID,
                                    IN OnUpdateInfoCallback callback) = 0;
    /*  
     * Function:     AbortSWUpdate
     * Arguments:    None
     * Return:       void
     * Description:  Function for aborting a currently running Update-Service
     */
    virtual void AbortSWUpdate() = 0;

    /*
     * Function: GetConnectedClients
     * Arguments: OUT std::map<ClientId,std::shared_ptr<ClientDescriptor>>& clients
     * Return: bool 
     * Description: Returns a map of connected client descriptions.
     */
    virtual bool GetConnectedClients(
                   OUT std::map<ClientId,std::shared_ptr<ClientDescriptor>>& clients) = 0;
    /*
     * Function: SetClientUserInterfaceInfo
     * Arguments: IN ClientId clientId client id
                  IN const std::string& userIfName user interface name
     *            IN uint32_t majorVer user interface major version
     *            IN uint32_t minorVer user interface minor version
     *            IN uint32_t patchVer user interface patch version
     * Return: bool 
     * Description: set user interface information of a certain client
     */
    virtual bool SetClientUserInterfaceInfo(IN ClientId clientId,
                                            IN const std::string& userIfName,
                                            IN uint32_t majorVer,
                                            IN uint32_t minorVer,
                                            IN uint32_t patchVer) = 0;

    virtual bool GetConfig(OUT ComLibConfig& config) = 0;

};

} // common
} // com

#endif //COM_COMMON_DATA_SERVICES_IF_H
