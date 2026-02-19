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

#include <ExternalTypes.h>
#include <stdint.h>
#include <stdlib.h>

#include <map>
#include <mutex>
#include <string>
#include <utility>

namespace com {

namespace types {

const uint8_t MAX_NUM_OF_INST = 255;

#ifdef _WIN32
#define USER_IF_LIB_PREFIX ""
#define USER_IF_LIB_EXT "_user_interface.dll"
#else
#define USER_IF_LIB_PREFIX "lib"
#define USER_IF_LIB_EXT "_user_interface.so"
#endif
#define CAN_MAX_DATA_BYTES 8

enum SerializationType {  // NOLINT(*enum-size)
  SERIALIZATION_TYPE_CAN_SPEC = 0,
  SERIALIZATION_TYPE_PORT_BASED,
  SERIALIZATION_TYPE_UNKNOWN
};

struct CanFormat {
  uint16_t u16_identifier;
  uint8_t u8_dlc;
  uint8_t au8_data[CAN_MAX_DATA_BYTES];  // NOLINT(*arrays)
};

enum LinkType {  // NOLINT(*enum-size)
  LINK_TYPE_UDP = 0,
  LINK_TYPE_UDP_DISCOVERY,
  LINK_TYPE_CAN,
  LINK_TYPE_CAN_DISCOVERY,
  LINK_TYPE_RS485,
  LIMK_TYPE_UNKNOWN,
  LINK_TYPE_UNKNOWN = LIMK_TYPE_UNKNOWN
};

enum ProtocolType {  // NOLINT(*enum-size)
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

enum LibraryRole {  // NOLINT(*enum-size)
  LIBRARY_ROLE_MASTER,
  LIBRARY_ROLE_SLAVE,
  LIBRARY_ROLE_UNKNOWN
};

enum TimeSyncRole {  // NOLINT(*enum-size)
  TIME_SYNC_ROLE_MASTER,
  TIME_SYNC_ROLE_SLAVE,
  TIME_SYNC_ROLE_UNKNOWN
};

using LinkId = uint16_t;
using SequenceNumber = uint32_t;
using CanNetId = uint8_t;
using CanId = uint16_t;
using UdtId = uint16_t;
using PhyDeviceId = uint8_t;

struct TimeOutDetails {
  uint16_t timeOutCount;
  uint64_t latestOffsetValue;
};
using TimeOutDeatils = TimeOutDetails;  // TODO(RKO): Remove in next release
using TimeOutMap = std::map<uint8_t, TimeOutDetails>;
const PortId INSTRUCTION_PORT_ID = 46;

class SharedLibDescriptor {
 public:
  // NOLINTNEXTLINE(*explicit*)
  SharedLibDescriptor(const std::string& libName);
  SharedLibDescriptor(const SharedLibDescriptor&) = delete;
  SharedLibDescriptor(SharedLibDescriptor&&) = default;
  SharedLibDescriptor& operator=(const SharedLibDescriptor&) = delete;
  SharedLibDescriptor& operator=(SharedLibDescriptor&&) = default;
  virtual ~SharedLibDescriptor();

  bool Link();

  void* GetHandle() { return _handle; }

 protected:
  // NOLINTBEGIN(*non-private-member-variables-in-classes)
  std::string _libName;
  void* _handle{nullptr};
  // NOLINTEND(*non-private-member-variables-in-classes)

 private:
  static std::mutex mutex_;
};

class ReceiverKey {
 public:
  ReceiverKey(ClientId clientId, PortId portId);
  ReceiverKey() = default;
  ReceiverKey(const ReceiverKey&) = default;
  ReceiverKey(ReceiverKey&&) = delete;
  ReceiverKey& operator=(const ReceiverKey&) = default;
  ReceiverKey& operator=(ReceiverKey&&) = delete;
  ~ReceiverKey() = default;

  ClientId GetClientId() const;
  void SetClientId(const ClientId& clientId);

  PortId GetPortId() const;
  void SetPortId(const PortId& portId);

 private:
  ClientId _clientId{0};
  PortId _portId{0};
};

inline bool operator<(const ReceiverKey& lhs, const ReceiverKey& rhs) {
  return (lhs.GetClientId() < rhs.GetClientId()) ||
         ((lhs.GetClientId() == rhs.GetClientId()) &&
          (lhs.GetPortId() < rhs.GetPortId()));
}

class ResponseKey {
 public:
  ResponseKey(ClientId clientId, SequenceNumber seqNum);
  ResponseKey() = default;
  ResponseKey(const ResponseKey&) = default;
  ResponseKey(ResponseKey&&) = delete;
  ResponseKey& operator=(const ResponseKey&) = default;
  ResponseKey& operator=(ResponseKey&&) = delete;
  ~ResponseKey() = default;

  ClientId GetClientId() const;
  void SetClientId(IN const ClientId& clientId);

  SequenceNumber GetSequenceNumber() const;
  void SetSequenceNumber(IN const SequenceNumber& seqNum);

 private:
  ClientId _clientId{0};
  SequenceNumber _seqNum{0};
};

inline bool operator<(IN const ResponseKey& lhs, IN const ResponseKey& rhs) {
  return (lhs.GetClientId() < rhs.GetClientId()) ||
         ((lhs.GetClientId() == rhs.GetClientId()) &&
          (lhs.GetSequenceNumber() < rhs.GetSequenceNumber()));
}

class EthNetDescriptor {
 public:
  EthNetDescriptor(const std::string& ip, uint32_t port, EthTransportType type);
  EthNetDescriptor() = default;
  EthNetDescriptor(const EthNetDescriptor&) = default;
  EthNetDescriptor(EthNetDescriptor&&) = delete;
  EthNetDescriptor& operator=(const EthNetDescriptor&) = default;
  EthNetDescriptor& operator=(EthNetDescriptor&&) = delete;
  ~EthNetDescriptor() = default;

  void GetIp(OUT std::string& ip) const;
  void SetIp(IN const std::string& ip);

  uint32_t GetPort() const;
  void SetPort(IN uint32_t port);

  EthTransportType GetType() const;
  void SetType(IN EthTransportType type);

 private:
  std::string _ip;
  uint32_t _port{0};
  EthTransportType _type{ETH_TRANSPORT_TYPE_TCP};
};

enum ClientPhyType {  // NOLINT(*enum-size)
  CLIENT_PHY_CAN = 0,
  CLIENT_PHY_ETH,
  CLIENT_PHY_RS485,
  CLIENT_PHY_UNKNOWN
};

class ClientDescriptor {
 public:
  ClientDescriptor(ClientId clientId, ClientPhyType phyType,
                   SerializationType instSerialType,
                   SerializationType dataSerialType, uint8_t phyDevId);
  ClientDescriptor(const ClientDescriptor&) = default;
  ClientDescriptor(ClientDescriptor&&) = delete;
  ClientDescriptor& operator=(const ClientDescriptor&) = default;
  ClientDescriptor& operator=(ClientDescriptor&&) = delete;
  virtual ~ClientDescriptor() = default;

  ClientPhyType GetPhyType() const;
  void SetPhyType(IN ClientPhyType phyType);

  ClientId GetId() const;
  void SetId(IN ClientId clientId);

  PhyDeviceId GetPhyDevId() const;
  void SetPhyDevId(IN PhyDeviceId phyDevId);

  SerializationType GetInstSerialType() const;
  void SetInstSerialType(IN SerializationType type);

  SerializationType GetDataSerialType() const;
  void SetDataSerialType(IN SerializationType type);

  std::string GetUserIfName() const;
  void SetUserIfName(IN const std::string& userIfName);

  uint32_t GetUserIfMajorVer() const;
  void SetUserIfMajorVer(IN uint32_t majorVer);

  uint32_t GetUserIfMinorVer() const;
  void SetUserIfMinorVer(IN uint32_t minorVer);

  uint32_t GetUserIfPatchVer() const;
  void SetUserIfPatchVer(IN uint32_t patchVer);

 private:
  ClientId _clientId{0};
  ClientPhyType _phyType{CLIENT_PHY_UNKNOWN};
  SerializationType _instSerialType{SERIALIZATION_TYPE_UNKNOWN};
  SerializationType _dataSerialType{SERIALIZATION_TYPE_UNKNOWN};
  PhyDeviceId _phyDevId{0};
  std::string _userIfName;
  uint32_t _userIfMajorVer{0};
  uint32_t _userIfMinorVer{0};
  uint32_t _userIfPatchVer{0};
};

class CanClientDescriptor : public ClientDescriptor {
 public:
  CanClientDescriptor(ClientId clientId, CanNetId canNetId,
                      PhyDeviceId phyDevId);
  CanClientDescriptor(const CanClientDescriptor&) = default;
  CanClientDescriptor(CanClientDescriptor&&) = delete;
  CanClientDescriptor& operator=(const CanClientDescriptor&) = default;
  CanClientDescriptor& operator=(CanClientDescriptor&&) = delete;
  ~CanClientDescriptor() override = default;

  CanNetId GetCanNetId() const;
  void SetCanNetId(IN CanNetId canNetId);

 private:
  CanNetId _canNetId{0};
};

class Rs485ClientDescriptor : public ClientDescriptor {
 public:
  Rs485ClientDescriptor(ClientId clientId, SerializationType instSerialType,
                        SerializationType dataSerialType, PhyDeviceId phyDevId);
  Rs485ClientDescriptor(const Rs485ClientDescriptor&) = default;
  Rs485ClientDescriptor(Rs485ClientDescriptor&&) = delete;
  Rs485ClientDescriptor& operator=(const Rs485ClientDescriptor&) = default;
  Rs485ClientDescriptor& operator=(Rs485ClientDescriptor&&) = delete;
  ~Rs485ClientDescriptor() override = default;
};

class EthClientDescriptor : public ClientDescriptor {
 public:
  EthClientDescriptor(ClientId clientId, SerializationType instSerialType,
                      SerializationType dataSerialType);
  EthClientDescriptor(const EthClientDescriptor&) = default;
  EthClientDescriptor(EthClientDescriptor&&) = delete;
  EthClientDescriptor& operator=(const EthClientDescriptor&) = default;
  EthClientDescriptor& operator=(EthClientDescriptor&&) = delete;
  ~EthClientDescriptor() override = default;
};

class SerializerConfig {
 public:
  SerializerConfig(ClientId clientId, SerializationType serialType);
  SerializerConfig(const SerializerConfig&) = default;
  SerializerConfig(SerializerConfig&&) = delete;
  SerializerConfig& operator=(const SerializerConfig&) = default;
  SerializerConfig& operator=(SerializerConfig&&) = delete;
  virtual ~SerializerConfig() = default;

  ClientId GetClientId() const;
  SerializationType GetType() const;

 private:
  ClientId _clientId{0};
  SerializationType _serialType{SERIALIZATION_TYPE_UNKNOWN};
};

class CanSerializerConfig : public SerializerConfig {
 public:
  CanSerializerConfig(ClientId clientId, LibraryRole role, CanNetId canNetId);
  CanSerializerConfig(const CanSerializerConfig&) = default;
  CanSerializerConfig(CanSerializerConfig&&) = delete;
  CanSerializerConfig& operator=(const CanSerializerConfig&) = default;
  CanSerializerConfig& operator=(CanSerializerConfig&&) = delete;
  ~CanSerializerConfig() override = default;

  CanNetId GetCanNetId() const;
  LibraryRole GetLibraryRole() const;

 private:
  CanNetId _canNetId{0};
  LibraryRole _role{LIBRARY_ROLE_UNKNOWN};
};

class PortBasedSerializerConfig : public SerializerConfig {
 public:
  PortBasedSerializerConfig(ClientId clientId, LibraryRole role);
  PortBasedSerializerConfig(const PortBasedSerializerConfig&) = default;
  PortBasedSerializerConfig(PortBasedSerializerConfig&&) = delete;
  PortBasedSerializerConfig& operator=(const PortBasedSerializerConfig&) =
      default;
  PortBasedSerializerConfig& operator=(PortBasedSerializerConfig&&) = delete;
  ~PortBasedSerializerConfig() override = default;

  LibraryRole GetLibraryRole() const;

 private:
  LibraryRole _role{LIBRARY_ROLE_UNKNOWN};
};

class InterfaceConfig {
 public:
  InterfaceConfig(LinkType linkType);  // NOLINT(*explicit*)
  InterfaceConfig(const InterfaceConfig&) = default;
  InterfaceConfig(InterfaceConfig&&) = delete;
  InterfaceConfig& operator=(const InterfaceConfig&) = default;
  InterfaceConfig& operator=(InterfaceConfig&&) = delete;
  virtual ~InterfaceConfig() = default;

  LinkType GetLinkType() const;

 private:
  LinkType _linkType{LINK_TYPE_UNKNOWN};
};

class CanInterfaceConfig : public InterfaceConfig {
 public:
  CanInterfaceConfig(CanNetId canNetId, CanDevId canDevId);
  CanInterfaceConfig(const CanInterfaceConfig&) = default;
  CanInterfaceConfig(CanInterfaceConfig&&) = delete;
  CanInterfaceConfig& operator=(const CanInterfaceConfig&) = default;
  CanInterfaceConfig& operator=(CanInterfaceConfig&&) = delete;
  ~CanInterfaceConfig() override = default;

  CanNetId GetNetId() const;
  CanDevId GetDevId() const;

  std::string GetUserIfName() const;
  void SetUserIfName(IN const std::string& userIfName);

  uint32_t GetUserIfMajorVer() const;
  void SetUserIfMajorVer(IN uint32_t majorVer);

  uint32_t GetUserIfMinorVer() const;
  void SetUserIfMinorVer(IN uint32_t minorVer);

  uint32_t GetUserIfPatchVer() const;
  void SetUserIfPatchVer(IN uint32_t patchVer);

 private:
  CanNetId _canNetId{0};
  CanDevId _canDevId{0};
  std::string _userIfName;
  uint32_t _userIfMajorVer{0};
  uint32_t _userIfMinorVer{0};
  uint32_t _userIfPatchVer{0};
};

class Rs485InterfaceConfig : public InterfaceConfig {
 public:
  Rs485InterfaceConfig(SerialDevId devId);  // NOLINT(*explicit*)
  Rs485InterfaceConfig(const Rs485InterfaceConfig&) = default;
  Rs485InterfaceConfig(Rs485InterfaceConfig&&) = delete;
  Rs485InterfaceConfig& operator=(const Rs485InterfaceConfig&) = default;
  Rs485InterfaceConfig& operator=(Rs485InterfaceConfig&&) = delete;
  ~Rs485InterfaceConfig() override = default;

  SerialDevId GetDevId() const;

 private:
  SerialDevId _devId{0};
};

class IpConfig : public InterfaceConfig {
 public:
  IpConfig(const std::string& ip, uint16_t port, LinkType type);
  IpConfig(const IpConfig&) = default;
  IpConfig(IpConfig&&) = delete;
  IpConfig& operator=(const IpConfig&) = default;
  IpConfig& operator=(IpConfig&&) = delete;
  ~IpConfig() override = default;

  std::string GetIp() const;
  uint16_t GetPort() const;

 private:
  std::string _ip;
  uint16_t _port{0};
};

class UdpConfig : public IpConfig {
 public:
  UdpConfig(const std::string& ip, uint16_t port);
  UdpConfig(const UdpConfig&) = default;
  UdpConfig(UdpConfig&&) = delete;
  UdpConfig& operator=(const UdpConfig&) = default;
  UdpConfig& operator=(UdpConfig&&) = delete;
  ~UdpConfig() override = default;
};

class MulticastConfig : public IpConfig {
 public:
  MulticastConfig(const std::string& ip, uint16_t port);
  MulticastConfig(const MulticastConfig&) = default;
  MulticastConfig(MulticastConfig&&) = delete;
  MulticastConfig& operator=(const MulticastConfig&) = default;
  MulticastConfig& operator=(MulticastConfig&&) = delete;
  ~MulticastConfig() override = default;
};

class ComLibConfig {
 public:
  ComLibConfig() = default;
  ComLibConfig(uint16_t majorVer, uint16_t minorVer, uint16_t patchVer);
  ComLibConfig(const ComLibConfig&) = default;
  ComLibConfig(ComLibConfig&&) = delete;
  ComLibConfig& operator=(const ComLibConfig&) = default;
  ComLibConfig& operator=(ComLibConfig&&) = delete;
  ~ComLibConfig() = default;

  LibraryRole GetRole() const;
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
  void SetAliveSupport(IN bool support);
  void SetUserInterfaceMajorVersion(uint16_t majorVer);
  void SetUserInterfaceMinorVersion(uint16_t minorVer);
  void SetUserInterfacePatchVersion(uint16_t patchVer);
  void SetComLibConfigMajorVersion(uint16_t majorVer);
  void SetComLibConfigMinorVersion(uint16_t minorVer);
  void SetComLibConfigPatchVersion(uint16_t patchVer);

 private:
  uint16_t _majorVer{0};
  uint16_t _minorVer{0};
  uint16_t _patchVer{0};
  std::string _serialLibPath;
  std::string _configFilesPath;
  ClientId _clientId{0};
  LibraryRole _role{LIBRARY_ROLE_UNKNOWN};
  std::string _userIfName;
  std::string _downloadPath;
  SerializationType _instSerialization{SERIALIZATION_TYPE_UNKNOWN};
  SerializationType _dataSerialization{SERIALIZATION_TYPE_UNKNOWN};
  ClientPhyType _timeSyncHwIfaceType{CLIENT_PHY_UNKNOWN};
  uint8_t _timeSyncHwDevId{};
  bool _isTimeSyncSupported{false};
  TimeSyncRole _timeSyncRole{TIME_SYNC_ROLE_UNKNOWN};
  bool _isAliveSupported{false};
  uint16_t _userIfMajorVer{0};
  uint16_t _userIfMinorVer{0};
  uint16_t _userIfPatchVer{0};
};

class SWUpdateConfigData {
 public:
  void SetData(IN bool pathFlag, IN std::string pathStr, IN bool dataFlag,
               IN int32_t dataSize, IN uint8_t dataEncrypt,
               IN uint8_t* dataPtr);

  void cleanData();

  bool GetPathAvailableFlag() const;
  std::string GetSegmentFilePath() const;
  bool GetDataPtrValidFlag() const;
  uint8_t GetSegmentEncryption() const;
  int32_t GetSegmentBufferSize() const;
  uint8_t* GetSegmentDataPtr();

 private:
  bool _pathAvailableFlag{false};
  std::string _segmentPath;
  bool _dataPtrValidFlag{false};
  int32_t _segmentBufferSize{0};
  uint8_t _segmentEncryption{0};
  uint8_t* _segmentDataPtr{nullptr};
};

class InternalTransportHeader {
 public:
  com::types::ProtocolType GetProtocolType() const;
  void SetProtocolType(IN com::types::ProtocolType protoType);

  bool HasSrcClientId() const;
  ClientId GetSrcClientId() const;
  void SetSrcClientId(IN ClientId clientId);

  bool HasDstClientId() const;
  ClientId GetDstClientId() const;
  void SetDstClientId(IN ClientId clientId);

 private:
  com::types::ProtocolType _protoType{PROTOCOL_TYPE_UNKNOWN};
  ClientId _srcClientId{0};
  bool _srcClientIdValid{false};
  ClientId _dstClientId{0};
  bool _dstClientIdValid{false};
};

}  // namespace types
}  // namespace com

#endif  // COM_TYPES_H
