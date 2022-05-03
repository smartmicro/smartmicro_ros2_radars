/************************************************************************
 * Copyright
 *
 * s.m.s. smart microwave sensors GmbH
 * In den Waashainen 1
 * 38108 Braunschweig - Germany
 *
 * Register Court: Amtsgericht Braunschweig / Register Number: HRB 4525
 * VAT ID: DE 193324798 / Tax Number: 13/207/02357
 ************************************************************************/

#ifndef COM_TYPES_H
#define COM_TYPES_H

#include <stdint.h>
#include <stdlib.h>
#include <map>
#include <set>
#include <ExternalTypes.h>

namespace com {

namespace types {

const uint8_t MAX_NUM_OF_INST = 255;

#ifdef _WIN32
    #define USER_IF_LIB_PREFIX  ""
    #define USER_IF_LIB_EXT  "_user_interface.dll"
#else
    #define USER_IF_LIB_PREFIX  "lib"
    #define USER_IF_LIB_EXT  "_user_interface.so"
#endif
#define CAN_MAX_DATA_BYTES   8

enum SerializationType {
    SERIALIZATION_TYPE_CAN_SPEC = 0,
    SERIALIZATION_TYPE_PORT_BASED,
    SERIALIZATION_TYPE_UNKNOWN
};

typedef struct
{
   uint16_t u16_identifier;
   uint8_t  u8_dlc;
   uint8_t  au8_data[CAN_MAX_DATA_BYTES];
}CanFormat;

enum LinkType {
    LINK_TYPE_UDP = 0,
    LINK_TYPE_UDP_DISCOVERY,
    LINK_TYPE_CAN,
    LINK_TYPE_CAN_DISCOVERY,
    LINK_TYPE_RS485,
    LIMK_TYPE_UNKNOWN
};

enum ProtocolType {
    PROTOCOL_TYPE_UNKNOWN = 0,
    PROTOCOL_TYPE_SMS_CAN_BASE_DATA_V1,
    PROTOCOL_TYPE_ATXMEGA_SDLC,
    PROTOCOL_TYPE_STG,
    PROTOCOL_TYPE_SMS_CAN_BASE_DATA_V2,
    PROTOCOL_TYPE_DEBUG,
    PROTOCOL_TYPE_LOG_MSG,
    PROTOCOL_TYPE_ALIVE,
    PROTOCOL_TYPE_PORT,
    PROTOCOL_TYPE_INTERVIEW,
    PROTOCOL_TYPE_DOWNLOAD,
    PROTOCOL_TYPE_TIME_SYNC,
    PROTOCOL_TYPE_DATA_STREAM,
    PROTOCOL_TYPE_INSTRUCTION
};

enum LibraryRole {
    LIBRARY_ROLE_MASTER,
    LIBRARY_ROLE_SLAVE,
    LIBRARY_ROLE_UNKNOWN
};

enum TimeSyncRole {
    TIME_SYNC_ROLE_MASTER,
    TIME_SYNC_ROLE_SLAVE,
    TIME_SYNC_ROLE_UNKNOWN
};


typedef uint16_t        LinkId;
typedef uint32_t        SequenceNumber;
typedef uint8_t         CanNetId;
typedef uint16_t        CanId;
typedef uint16_t        UdtId;
typedef uint8_t         PhyDeviceId;

const PortId INSTRUCTION_PORT_ID = 46;


class SharedLibDescriptor {
public:
    SharedLibDescriptor(const std::string& libName);
    virtual ~SharedLibDescriptor();

    bool Link();

    inline void* GetHandle() 
    {
        return _handle;
    }

protected:
    std::string      _libName;
    void*            _handle;
};


class ReceiverKey {
public:
    ReceiverKey(ClientId clientId,PortId portId);
    ReceiverKey();
    ~ReceiverKey();

    ClientId GetClientId() const;
    void SetClientId(const ClientId& clientId);

    PortId GetPortId() const;
    void SetPortId(const PortId& portId); 

  
private:

    ClientId                 _clientId;
    PortId                   _portId;

};

inline bool operator<(const ReceiverKey& lhs,const ReceiverKey& rhs)
{
    if((lhs.GetClientId() < rhs.GetClientId()) ||
       ((lhs.GetClientId() == rhs.GetClientId()) && (lhs.GetPortId() < rhs.GetPortId())))
    {
        return true;
    }
    return false;
}

class ResponseKey {
public:
    ResponseKey(ClientId clientId,SequenceNumber seqNum);
    ResponseKey();
    ~ResponseKey();

    ClientId GetClientId() const;
    void SetClientId(IN const ClientId& clientId);

    SequenceNumber GetSequenceNumber() const;
    void SetSequenceNumber(IN const SequenceNumber& seqNum); 

  
private:

    ClientId                 _clientId;
    SequenceNumber           _seqNum;

};

inline bool operator<(IN const ResponseKey& lhs,IN const ResponseKey& rhs)
{
    if((lhs.GetClientId() < rhs.GetClientId()) ||
       ((lhs.GetClientId() == rhs.GetClientId()) && (lhs.GetSequenceNumber() < rhs.GetSequenceNumber())))
    {
        return true;
    }
    return false;
}


class EthNetDescriptor {
public:
    EthNetDescriptor(const std::string& ip,
                           uint32_t port,
                           EthTransportType type);

    EthNetDescriptor(){} 
    ~EthNetDescriptor(){}
    // ACCESS
    void GetIp(OUT std::string& ip) const;
    void SetIp(IN const std::string& ip);

    uint32_t GetPort() const;
    void SetPort(IN uint32_t port);

    EthTransportType GetType() const;
    void SetType(IN EthTransportType type);


private:

    std::string       _ip;
    uint32_t          _port;
    EthTransportType  _type;

};


enum ClientPhyType {
    CLIENT_PHY_CAN = 0,
    CLIENT_PHY_ETH,
    CLIENT_PHY_RS485,
    CLIENT_PHY_UNKNOWN
};


class ClientDescriptor {
public:  
    ClientDescriptor(ClientId clientId,
                     ClientPhyType phyType,
                     SerializationType instSerialType,
                     SerializationType dataSerialType,
                     uint8_t phyDevId);

    virtual ~ClientDescriptor() {}

    inline ClientPhyType GetPhyType() const
    {
        return _phyType;
    }
    inline void SetPhyType(IN ClientPhyType phyType)
    {
        _phyType = phyType; 
    }

    inline ClientId GetId() const
    {
        return _clientId;    
    }

    inline void SetId(IN ClientId clientId)
    {
        _clientId = clientId;    
    }

    inline void SetPhyDevId(IN PhyDeviceId phyDevId)
    {
        _phyDevId = phyDevId;    
    }

    inline SerializationType GetInstSerialType() const
    {
        return _instSerialType;
    }

    inline void SetInstSerialType(IN SerializationType type)
    {
        _instSerialType = type;
    }
   
    inline SerializationType GetDataSerialType() const
    {
        return _dataSerialType;
    }

    inline void SetDataSerialType(IN SerializationType type)
    {
        _dataSerialType = type;
    }

    inline PhyDeviceId GetPhyDevId() const
    {
        return _phyDevId;    
    }

    inline void SetUserIfName(IN const std::string userIfName) 
    {
        _userIfName = userIfName;    
    }

    inline std::string GetUserIfName() const
    {
        return _userIfName;    
    }

    inline void SetUserIfMajorVer(IN uint32_t majorVer)
    {
        _userIfMajorVer = majorVer;
    }

    inline uint32_t GetUserIfMajorVer() const
    {
        return _userIfMajorVer;    
    }

    inline void SetUserIfMinorVer(IN uint32_t minorVer)
    {
        _userIfMinorVer = minorVer;    
    }

    inline uint32_t GetUserIfMinorVer() const
    {
        return _userIfMinorVer;    
    }

    inline void SetUserIfPatchVer(IN uint32_t patchVer)
    {
        _userIfPatchVer = patchVer;    
    }

    inline uint32_t GetUserIfPatchVer() const
    {
        return _userIfPatchVer;    
    }

private:
 
    ClientId                _clientId; 
    ClientPhyType           _phyType;
    SerializationType       _instSerialType;
    SerializationType       _dataSerialType;
    PhyDeviceId             _phyDevId;
    std::string             _userIfName;
    uint32_t                _userIfMajorVer; 
    uint32_t                _userIfMinorVer;
    uint32_t                _userIfPatchVer;

};


class CanClientDescriptor : public ClientDescriptor{
public:  
    CanClientDescriptor(ClientId clientId, CanNetId canNetId, PhyDeviceId phyDevId);
    virtual ~CanClientDescriptor() {}

    inline CanNetId GetCanNetId() const
    {
        return _canNetId;    
    }

    inline void SetCanNetId(IN CanNetId canNetId)
    {
        _canNetId = canNetId;    
    }
   
private:
    CanNetId                _canNetId;
};

class Rs485ClientDescriptor : public ClientDescriptor{
public:  
    Rs485ClientDescriptor(ClientId clientId, 
                          SerializationType instSerialType,
                          SerializationType dataSerialType,
                          PhyDeviceId phyDevId);
    virtual ~Rs485ClientDescriptor() {}

};


class EthClientDescriptor : public ClientDescriptor{
public:  
    EthClientDescriptor(ClientId clientId, 
                        SerializationType instSerialType,
                        SerializationType dataSerialType);
    virtual ~EthClientDescriptor() {}

};





class SerializerConfig {
public:
    SerializerConfig(ClientId clientId, SerializationType serialType)
    : _clientId(clientId),
      _serialType(serialType)
    {}

    virtual ~SerializerConfig() {}

    inline ClientId GetClientId() const
    {
        return _clientId;    
    }

    inline SerializationType GetType() const
    {
        return  _serialType;   
    }

private:

    ClientId           _clientId;
    SerializationType  _serialType;
};



class CanSerializerConfig : public SerializerConfig {
public:
    CanSerializerConfig(ClientId clientId,
                         LibraryRole role, 
                         CanNetId canNetId)
    : SerializerConfig(clientId,SERIALIZATION_TYPE_CAN_SPEC),
      _role(role),
      _canNetId(canNetId)
     {}

     virtual ~CanSerializerConfig() {}
 
     inline CanNetId GetCanNetId() const
     {
        return _canNetId;    
     }

     inline LibraryRole  GetLibraryRole() const
     {
         return _role;    
     }

private:
     CanNetId      _canNetId;
     LibraryRole   _role;
     
};

class PortBasedSerializerConfig : public SerializerConfig {
public:
    PortBasedSerializerConfig(ClientId clientId,
                         LibraryRole role) 
     : SerializerConfig(clientId,SERIALIZATION_TYPE_PORT_BASED),
     _role(role)
    {}
    virtual ~PortBasedSerializerConfig() {}
 
    inline LibraryRole GetLibraryRole() const
    {
        return _role;     
    }

private:
     
     LibraryRole          _role;
     
};


class InterfaceConfig {
public:
    InterfaceConfig(LinkType linkType);
    virtual ~InterfaceConfig() {}
    
    LinkType GetLinkType() const;

private:

    LinkType    _linkType;
    
};


class CanInterfaceConfig : public InterfaceConfig {
public:
    CanInterfaceConfig(CanNetId canNetId, CanDevId canDevId);
    virtual ~CanInterfaceConfig() {}

    CanNetId GetNetId() const;
    CanDevId GetDevId() const;

    inline void SetUserIfName(IN const std::string& userIfName)
    {
        _userIfName = userIfName;    
    }

    inline std::string GetUserIfName() const
    {
        return _userIfName; 
    }

    inline void SetUserIfMajorVer(IN uint32_t majorVer)
    {
        _userIfMajorVer = majorVer;
    }

    inline uint32_t GetUserIfMajorVer() const
    {
        return _userIfMajorVer;    
    }

    inline void SetUserIfMinorVer(IN uint32_t minorVer)
    {
        _userIfMinorVer = minorVer;    
    }

    inline uint32_t GetUserIfMinorVer() const
    {
        return _userIfMinorVer;    
    }

    inline void SetUserIfPatchVer(IN uint32_t patchVer)
    {
        _userIfPatchVer = patchVer;    
    }

    inline uint32_t GetUserIfPatchVer() const
    {
        return _userIfPatchVer;    
    }


private:
    CanNetId     _canNetId; 
    CanDevId     _canDevId;  
    std::string  _userIfName;
    uint32_t     _userIfMajorVer;
    uint32_t     _userIfMinorVer;
    uint32_t     _userIfPatchVer;
};

class Rs485InterfaceConfig : public InterfaceConfig {
public:
    Rs485InterfaceConfig(SerialDevId devId);
    virtual ~Rs485InterfaceConfig() {}

    SerialDevId GetDevId() const;

private:

    SerialDevId   _devId;  
};


class IpConfig : public InterfaceConfig {
public:
    IpConfig(const std::string& ip, uint16_t port, LinkType type)
    : InterfaceConfig(type),
      _ip(ip),
      _port(port)
    {}
    virtual ~IpConfig() {}

    inline std::string GetIp() const
    {
        return _ip;    
    }

    inline uint16_t GetPort() const
    {
        return _port;    
    }
private:

    std::string             _ip;
    uint16_t                _port;    
};


class UdpConfig : public IpConfig
{
public: 
    UdpConfig(const std::string& ip, uint16_t port)
    : IpConfig(ip,port,LINK_TYPE_UDP)
    {}
    virtual ~UdpConfig() 
    {}
 };

class MulticastConfig : public IpConfig
{
public: 
    MulticastConfig(const std::string& ip, uint16_t port)
    : IpConfig(ip,port,LINK_TYPE_UDP_DISCOVERY)
    {}
    virtual ~MulticastConfig() 
    {}
 };


class ComLibConfig {
public:
    ComLibConfig();
   
    ComLibConfig(uint16_t majorVer, uint16_t minorVer, uint16_t  patchVer);

    ~ComLibConfig();
    //ACCESS

    LibraryRole GetRole()  const;
    ClientId GetClientId() const;
    const std::string& GetConfigPath() const;
    const std::string& GetSerialLibPath() const;
    const std::string& GetUserInterfaceName() const;
    uint16_t GetUserInterfaceMajorVersion() const;
    uint16_t GetUserInterfaceMinorVersion() const;
    uint16_t GetUserInterfacePatchVersion() const;
    const std::string& GetDownloadPath() const;
    SerializationType GetInstSerializationType() const;
    SerializationType GetDataSerializationType() const;
    ClientPhyType GetTimeSyncHwIfaceType() const;
    uint8_t GetTimeSyncHwDeviceId() const;
    bool IsTimeSyncSupported() const;
    bool IsAliveSupported() const;
    TimeSyncRole GetTimeSyncRole() const;
    uint8_t GetTimeSyncNumOfIter() const;
    uint16_t GetMajorVersion() const;
    uint16_t GetMinorVersion() const;
    uint16_t GetPatchVersion() const;


    void SetRole(IN LibraryRole role);
    void SetClientId(IN ClientId clientId);
    void SetConfigPath(IN const std::string& path);
    void SetSerialLibPath(IN const std::string& path);
    void SetUserInterfaceName(IN const std::string& name);
    void SetDownloadPath(IN const std::string& path);
    void SetDataSerializationType(IN SerializationType type);
    void SetInstSerializationType(IN SerializationType type);
    void SetTimeSyncHwIfaceType(IN ClientPhyType hwIfType);
    void SetTimeSyncHwDeviceId(IN uint8_t deviceId);
    void SetTimeSyncSupport(IN bool isSupported);
    void SetTimeSyncRole(IN TimeSyncRole role);
    void SetTimeSyncNumOfIter(IN uint8_t numOfIter);
    void SetAliveSupport(IN bool support);
    void SetUserInterfaceMajorVersion(uint16_t majorVer);
    void SetUserInterfaceMinorVersion(uint16_t minorVer);
    void SetUserInterfacePatchVersion(uint16_t patchVer);
    void SetComLibConfigMajorVersion(uint16_t majorVer);
    void SetComLibConfigMinorVersion(uint16_t minorVer);
    void SetComLibConfigPatchVersion(uint16_t patchVer);

private:
    uint16_t                    _majorVer;
    uint16_t                    _minorVer;
    uint16_t                    _patchVer;
    std::string                 _serialLibPath;
    std::string                 _configFilesPath;
    ClientId                    _clientId;
    LibraryRole                 _role;
    std::string                 _userIfName;
    std::string                 _downloadPath;
    SerializationType           _instSerialization;
    SerializationType           _dataSerialization;
    ClientPhyType       _timeSyncHwIfaceType;
    uint8_t             _timeSyncHwDevId;
    bool                _isTimeSyncSupported;
    TimeSyncRole        _timeSyncRole;
    uint8_t             _numOfIter;
    bool                _isAliveSupported;
    uint16_t            _userIfMajorVer;
    uint16_t            _userIfMinorVer;
    uint16_t            _userIfPatchVer;

};

class SWUpdateConfigData 
{
public:
   SWUpdateConfigData()
   : 
   _pathAvailableFlag(false),
   _dataPtrValidFlag(false),
   _segmentBufferSize(0),
   _segmentDataPtr((uint8_t *)0)
   {
   }
   virtual ~SWUpdateConfigData()
   {
   }   

   inline void SetData(IN bool pathFlag, IN std::string pathStr, 
                       IN bool dataFlag, IN int32_t dataSize, 
                       IN uint8_t dataEncrypt, 
                       IN uint8_t *dataPtr)
   {
      _pathAvailableFlag = pathFlag;
      _segmentPath       = pathStr;
      _dataPtrValidFlag  = dataFlag;
      _segmentBufferSize = dataSize;
      _segmentEncryption = dataEncrypt;
      _segmentDataPtr    = dataPtr;
   }

   void cleanData();

   inline bool GetPathAvailableFlag() const
   {
      return _pathAvailableFlag;  
   }

   inline std::string GetSegmentFilePath() const
   {
      return  _segmentPath;
   }

   inline bool GetDataPtrValidFlag() const
   {
      return _dataPtrValidFlag;  
   }

   inline uint8_t GetSegmentEncryption() const
   {
      return _segmentEncryption;
   }

   inline int32_t GetSegmentBufferSize() const
   {
      return _segmentBufferSize;
   }

   inline uint8_t *GetSegmentDataPtr()
   {
      return _segmentDataPtr;
   } 

private:
   bool           _pathAvailableFlag;
   std::string    _segmentPath;
   bool           _dataPtrValidFlag;
   int32_t        _segmentBufferSize;
   uint8_t        _segmentEncryption;
   uint8_t       *_segmentDataPtr;
};

} // types
} // com

#endif //COM_TYPES_H
