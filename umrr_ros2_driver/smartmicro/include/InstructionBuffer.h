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

#ifndef COM_COMMON_INSTRUCTION_BUFFER_H
#define COM_COMMON_INSTRUCTION_BUFFER_H

#include <Types.h>

#include <vector>

using namespace com::types;

namespace com {
namespace common {

/** @} */

#define MAX_DIM_ELEMENT \
  2U /**< @brief Max. numbers of available dimension elements */
#define INSTR_HANDLER_CMD_DATA_TYPE 6U

enum RequestType {
  COM_INSTR_PORT_INVALID = 0U, /**< @brief Instruction Requests is invalid */
  COM_INSTR_PORT_PARAM_SET_PARAMETER =
      1U, /**< @brief Instruction Requests for setting a parameter */
  COM_INSTR_PORT_PARAM_GET_PARAMETER =
      2U, /**< @brief Instruction Requests for reading a parameter */
  COM_INSTR_PORT_STATUS_GET_STATUS =
      3U, /**< @brief Instruction Requests for reading a status */
  COM_INSTR_PORT_CMD = 4U, /**< @brief Instruction Requests for command  */
  COM_INSTR_PORT_EXPORT =
      5U, /**< @brief Instruction Requests for export param/status */
  COM_INSTR_PORT_RESERVED_0 = 248U, /**< @brief Reserved 0 */
  COM_INSTR_PORT_RESERVED_1 = 249U, /**< @brief Reserved 1 */
  COM_INSTR_PORT_RESERVED_2 = 250U, /**< @brief Reserved 2 */
  COM_INSTR_PORT_RESERVED_3 = 251U, /**< @brief Reserved 3 */
  COM_INSTR_PORT_RESERVED_4 = 252U, /**< @brief Reserved 4 */
  COM_INSTR_PORT_RESERVED_5 = 253U, /**< @brief Reserved 5 */
  COM_INSTR_PORT_RESERVED_6 = 254U, /**< @brief Reserved 6 */
  COM_INSTR_PORT_RESERVED_7 = 255U, /**< @brief Reserved 7 */
};

enum ResposeType {
  COM_INSTR_PORT_NO_RESPONSE = 0U, /**< @brief No instruction Response */
  COM_INSTR_PORT_SUCCESS =
      1U, /**< @brief Instruction Response was processed successfully */
  COM_INSTR_PORT_ERROR = 2U,         /**< @brief General error */
  COM_INSTR_PORT_ERROR_REQUEST = 3U, /**< @brief Invalid request */
  COM_INSTR_PORT_ERROR_SECTION = 4U, /**< @brief Invalid section */
  COM_INSTR_PORT_ERROR_ID = 5U,      /**< @brief Invalid id */
  COM_INSTR_PORT_ERROR_PROT = 6U,    /**< @brief Invalid protection */
  COM_INSTR_PORT_ERROR_MIN = 7U,     /**< @brief Value out of minimal bounds */
  COM_INSTR_PORT_ERROR_MAX = 8U,     /**< @brief Value out of maximal bounds */
  COM_INSTR_PORT_ERROR_NAN = 9U,     /**< @brief Value is not a number */
  COM_INSTR_PORT_ERROR_TYPE =
      10U, /**< @brief Type of Instruction is not valid */
  COM_INSTR_PORT_ERROR_DIM = 11U, /**< @brief Dim of Instruction is not valid */
  COM_INSTR_PORT_ERROR_ELEMENT =
      12U, /**< @brief Element of Instruction is not valid */
  COM_INSTR_PORT_ERROR_SIGNATURE =
      13U, /**< @brief Signature of Instruction is not valid */
  COM_INSTR_PORT_ERROR_ACCESS_LVL =
      14U, /**< @brief Access level is not valid */
  COM_INSTR_PORT_ERROR_INTERNAL =
      15U, /**< @brief Internal error shall not be sent */
};

class Instruction {
 public:
  Instruction(uint32_t sectionId, uint32_t id)
      : _request(0),
        _response(0),
        _sectionId(sectionId),
        _id(id),
        _dataType(0),
        _dimCount(0),
        _value(0),
        _signature(0) {}

  ~Instruction() {}

  inline uint8_t GetRequest() const { return _request; }

  inline void SetRequest(IN uint8_t request) { _request = request; }

  inline uint8_t GetResponse() const { return _response; }

  inline void SetResponse(IN uint8_t response) { _response = response; }

  inline uint16_t GetSectionId() const { return _sectionId; }

  inline void SetSectionId(IN uint16_t sectionId) { _sectionId = sectionId; }

  inline uint16_t GetId() const { return _id; }

  inline void SetId(IN uint16_t id) { _id = id; }

  inline uint8_t GetDataType() const { return _dataType; }

  inline void SetDataType(IN uint8_t dataType) { _dataType = dataType; }

  inline uint8_t GetDimCount() const { return _dimCount; }

  inline bool SetDimCount(IN uint8_t dimCount) {
    if (dimCount <= MAX_DIM_ELEMENT) {
      _dimCount = dimCount;
      return true;
    }
    return false;
  }

  inline uint16_t GetDimElement(IN uint8_t index) const {
    return (index < _dimCount ? _dimElement[index] : 0);
  }

  inline bool SetDimElement(IN uint8_t index, IN uint16_t element) {
    if (index < _dimCount) {
      _dimElement[index] = element;
      return true;
    }
    return false;
  }

  inline uint32_t GetSignature() const { return _signature; }

  inline void SetSignature(IN uint32_t signature) { _signature = signature; }

  inline uint64_t GetValue() const { return _value; }

  inline void SetValue(IN uint64_t value) { _value = value; }

 private:
  uint8_t _request;    /**< @brief Instruction request command */
  uint8_t _response;   /**< @brief Instruction response */
  uint16_t _sectionId; /**< @brief Section number. */
  uint16_t _id;        /**< @brief Each instruction has a unique ID */
  uint8_t _dataType;   /**< @brief The data type. */
  uint8_t _dimCount;   /**< @brief Number of dimensions.
                            Value 0 for non-array structures. */
  uint16_t _dimElement[MAX_DIM_ELEMENT]; /**< @brief Status element indices
                                            addressing. */
  uint32_t _signature;                   /**< @brief Instruction signature. */
  uint64_t _value;                       /**< @brief Instruction value. */
};

class InstructionBuffer {
 public:
  InstructionBuffer() : _seqCount(0) {}

  InstructionBuffer(uint32_t seqCount) : _seqCount(seqCount) {}

  ~InstructionBuffer() {}

  inline void AddInstruction(IN std::shared_ptr<Instruction> inst) {
    _instructions.push_back(inst);
  }

  inline std::vector<std::shared_ptr<Instruction>>& GetInstructions() {
    return _instructions;
  }

  inline size_t GetNumOfInstructions() const { return _instructions.size(); }

  inline void SetSeqCount(IN SequenceNumber seqCount) { _seqCount = seqCount; }

  inline SequenceNumber GetSeqCount() const { return _seqCount; }

  inline void Reset() { _instructions.clear(); }

  void operator=(const InstructionBuffer& rhs) {
    _instructions = rhs._instructions;
    _seqCount = rhs._seqCount;
  }

 private:
  std::vector<std::shared_ptr<Instruction>> _instructions;
  uint32_t _seqCount;
};

}  // namespace common
}  // namespace com

#endif  // COM_COMMON_INSTRUCTION_BUFFER_H
