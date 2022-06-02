#ifndef MDEFINES_H
#define MDEFINES_H

#include <HardwareSerial.h>

#define DISCRETE_INPUT 0
#define COIL 1
#define INPUT_REG 0
#define HOLDING_REG 1 

// Config.
#define MODBUS_BYTE 8
#define MODBUS_CRC_BYTE_COUNT 2
#define MODBUS_MAX_FRAME_SIZE SERIAL_RX_BUFFER_SIZE // Set SERIAL_RX_BUFFER_SIZE to 256 to follow MODBUS standard.

#define MODBUS_ID_BROADCAST 0
#define MODBUS_ID_MIN 1
#define MODBUS_ID_MAX 247

#define MODBUS_MAX_COILS_TO_READ 2008     // (252 [bytes of data] - 1 [for bytecount]) * 8 [bits in byte] see Frame description in Specification and Implementation Guide V1.02 page 13.
#define MODBUS_MAX_INPUTS_TO_READ 2008    // See MODBUS_MAX_COILS_TO_READ.
#define MODBUS_MAX_HREGISTERS_TO_READ 125 // (251 [bytes of data] - 1 [for bytecount]) / 2 [2 bytes for register] and floor the result.
#define MODBUS_MAX_IREGISTERS_TO_READ 125 // See MODBUS_MAX_HREGISTERS_TO_READ.

#define MODBUS_MAX_COILS_TO_WRITE 2008     // See MODBUS_MAX_COILS_TO_READ.
#define MODBUS_MAX_HREGISTERS_TO_WRITE 125 // See MODBUS_MAX_HREGISTERS_TO_READ.

// Byte position in request frame.
#define MODBUS_ID 0
#define MODBUS_FUNCTION_CODE 1
#define MODBUS_ADDR_HIGH 2
#define MODBUS_ADDR_LOW 3
#define MODBUS_QUANTITY_HIGH 4
#define MODBUS_QUANTITY_LOW 5
#define MODBUS_VALUE_HIGH 4
#define MODBUS_VALUE_LOW 5
#define MODBUS_BYTE_COUNT 6
#define MODBUS_FORCE_MULTIPLE_FIRST_BYTE_OF_DATA 7 // Index of first byte of data after byte count in force multiple (15 and 16) commands.

// Function codes.
#define MODBUS_READ_COIL_STATUS 1
#define MODBUS_READ_INPUT_STATUS 2
#define MODBUS_READ_HOLDING_REGISTER 3
#define MODBUS_READ_INPUT_REGISTER 4
#define MODBUS_FORCE_SINGLE_COIL 5
#define MODBUS_PRESET_SINGLE_REGISTER 6
#define MODBUS_FORCE_MULTIPLE_COILS 0x0F
#define MODBUS_PRESET_MULTIPLE_REGISTERS 0x10

#define MODBUS_ERR_OFFSET 0x80

// Error codes.
#define MODBUS_ERR_ILLEGAL_FUNCTION 1
#define MODBUS_ERR_ILLEGAL_ADDR 2
#define MODBUS_ERR_ILLEGAL_DATA 3
#define MODBUS_ERR_SLAVE_FAILURE 4
#define MODBUS_ERR_SLAVE_BUSY 6
#endif