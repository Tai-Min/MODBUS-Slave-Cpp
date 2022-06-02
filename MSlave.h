#ifndef MSLAVE_H
#define MSLAVE_H

#include "mdefines.h"

#ifdef ARDUINO
#include <Arduino.h>
#else
class HardwareSerial
{
public:
    /**
     * @brief Write single byte to serial device.
     *
     * @param byte Byte to write.
     */
    virtual void write(uint8_t byte) = 0;

    /**
     * @brief Check serial port for unread data.
     *
     * @return Number of bytes available to read.
     */
    virtual int available() = 0;

    /**
     * @brief Read bytes.
     *
     * @param buffer Buffer to place bytes in.
     * @param length Size of the buffer.
     * @return Number of bytes placed in the buffer.
     */
    virtual size_t readBytes(uint8_t *buffer, size_t length) = 0;

    /**
     * @brief Wait for tx tramsmission to complete.
     */
    virtual void flush() = 0;
};
#endif

// <coil, input, holding register, input register>
template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
class MSlave
{
private:
    bool crcDisabled = 0;
    uint8_t id = 255;

    bool busy = 0;

    bool uartUsed = 1;
    void (*actAsTransmitter)(bool) = nullptr;

    uint16_t AQ[AQSize]; // Analog out.
    bool DQ[DQSize];     // Digital out.
    uint16_t AI[AISize]; // Analog in.
    bool DI[DISize];     // Digital in.

    HardwareSerial *S = nullptr;

    static uint8_t toError(uint8_t code) { return code + MODBUS_ERR_OFFSET; }
    static uint16_t toWord(uint8_t H, uint8_t L) { return ((uint16_t)H << MODBUS_BYTE) | L; }
    static uint8_t toHighByte(uint16_t word) { return (word >> MODBUS_BYTE); }
    static uint8_t toLowByte(uint16_t word) { return word; }

    static uint16_t crc(uint8_t command[], uint8_t commandLength);
    void sendResponse(uint8_t tab[], uint16_t length) const;

    bool readCoilStatus(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL) const;
    bool readInputStatus(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL) const;
    bool readHoldingRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL) const;
    bool readInputRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL) const;
    bool forceSingleCoil(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t valH, uint8_t valL);
    bool presetSingleRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t valH, uint8_t valL);
    bool forceMultipleCoils(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL, uint8_t byteCount, uint8_t command[], uint8_t commandLength);
    bool forceMultipleRegisters(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL, uint8_t byteCount, uint8_t command[], uint8_t commandLength);

public:
    /**
     * @brief Class constructor.
     */
    MSlave();

    /**
     * @brief Enable MODBUS server.
     *
     * @param id_ ID of this server
     * @param S_ HardwareSerial object.
     */
    void begin(uint8_t id_, HardwareSerial &S_);

    /**
     * @brief Disable MODBUS server.
     */
    void end();

    /**
     * @brief Check if there is data pending in HardwareSerial object attached to this server.
     *
     * @return True if there is data pending.
     */
    bool available() const;

    /**
     * @brief Set that device can't respond right now.
     */
    void setBusy() { busy = 0; }

    /**
     * @brief Set that device is available for communication.
     */
    void setIdle() { busy = 1; }

    /**
     * @brief Enable CRC for read/write frames.
     */
    void enableCRC() { crcDisabled = 0; }

    /**
     * @brief Disable CRC for read/write frames.
     */
    void disableCRC() { crcDisabled = 1; }

    /**
     * @brief Set that device uses single master - single slave method of communication.
     */
    void useUART();

    /**
     * @brief Set that device uses single master - multiple slave method of communication.
     *
     * @param actAsTransmitter Pointer to function which turns slave device into transmitter if true is passed as an agrument and into receiver if false is passed.
     */
    void useRS485(void (*actAsTransmitter_)(bool));

    /**
     * @brief Read state of an input or a coil.
     *
     * @param type Either DISCRETE_INPUT or COIL.
     * @param addr Address of coil or input to be read.
     * @return State of an input or a coil.
     */
    bool digitalRead(bool type, uint16_t addr) const;

    /**
     * @brief Write input or a coil.
     *
     * @param type Either DISCRETE_INPUT or COIL.
     * @param addr Address of input or coil to be written.
     * @param val Boolean value to be written.
     */
    void digitalWrite(bool type, uint16_t addr, bool val);

    /**
     * @brief Read state of requested register.
     *
     * @param type Either INPUT_REG or HOLDING_REG.
     * @param addr Address of input or holding register to be read.
     * @return Value of requested register.
     */
    uint16_t analogRead(bool type, uint16_t addr) const;

    /**
     * @brief Write value to requested register.
     *
     * @param type Either INPUT_REG or HOLDING_REG.
     * @param addr Address of input or holding register to be written.
     */
    void analogWrite(bool type, uint16_t addr, uint16_t val);

    /**
     * @brief Read and process data received from client device.
     *
     * @returns MODBUS function code if data successfully processed or 0 if there was no data, error occured or invalid request happened.
     */
    uint8_t read();
};

// https://stackoverflow.com/questions/19347685/calculating-modbus-rtu-crc-16
template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
uint16_t MSlave<DQSize, DISize, AQSize, AISize>::crc(uint8_t command[], uint8_t commandLength)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t pos = 0; pos < commandLength; pos++)
    {
        crc ^= (uint16_t)command[pos]; // XOR byte into least sig. byte of crc.

        for (uint8_t i = MODBUS_BYTE; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {              // If the LSB is set.
                crc >>= 1; // Shift right and XOR 0xA001.
                crc ^= 0xA001;
            }
            else           // Else LSB is not set.
                crc >>= 1; // Just shift right.
        }
    }

    // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes).
    return crc;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::sendResponse(uint8_t tab[], uint16_t length) const
{
    if (tab[MODBUS_ID] == MODBUS_ID_BROADCAST)
        return; // Do not answer when broadcast id has been received.

    uint8_t lcrc, hcrc;
    if (!crcDisabled)
    {
        uint16_t computedcrc = crc(tab, length);
        lcrc = toLowByte(computedcrc);
        hcrc = toHighByte(computedcrc);
    }

    if (!uartUsed && actAsTransmitter != nullptr)
        actAsTransmitter(true); // Use function given by user to toggle rs485 into transmitter.

    for (uint16_t i = 0; i < length; i++)
    {
        S->write(tab[i]);
    }
    if (!crcDisabled)
    {
        S->write(lcrc);
        S->write(hcrc);
    }

    if (!uartUsed && actAsTransmitter != nullptr)
        actAsTransmitter(false); // Use function given by user to toggle rs485 into receiver.
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::readCoilStatus(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL) const
{
    if (id == MODBUS_ID_BROADCAST)
        return false;

    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    if (addr >= DQSize || addr + quantity > DQSize) // Check if illegal adress was selected.
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_COIL_STATUS), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return false;
    }
    else if (quantity == 0 || quantity > MODBUS_MAX_COILS_TO_READ) // Check if illegal number of outputs was selected.
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_COIL_STATUS), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return false;
    }

    uint16_t currentAddr = addr;
    uint8_t byteCount = quantity / MODBUS_BYTE; // Number of bytes to iterate over to read bits.
    if (quantity % MODBUS_BYTE != 0)            // Quantity is not multiple of 8 so increase byteCount once so it can hold remaining bits.
        byteCount++;

    uint8_t resp[3 + byteCount];
    for (uint8_t i = 0; i < byteCount; i++) // Read one byte.
    {
        resp[i + 3] = 0;
        for (uint8_t j = 0; j < MODBUS_BYTE; j++) // Read all bits in one byte.
        {
            resp[i + 3] |= (DQ[currentAddr] << j); // Push specified bit to current byte of output buffer.

            currentAddr++;
            if (currentAddr - addr >= quantity) // Check if every selected output has been read and if so stop iterating.
                break;                          // Should be true when quantity is not multiply of 8.
        }
    }

    // Prepare response.
    resp[0] = id;
    resp[1] = MODBUS_READ_COIL_STATUS;
    resp[2] = byteCount;
    sendResponse(resp, byteCount + 3);
    return true;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::readInputStatus(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL) const
{
    if (id == MODBUS_ID_BROADCAST)
        return false;

    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    if (addr >= DISize || addr + quantity > DISize) // Check if illegal adress was selected.
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUT_STATUS), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return false;
    }
    else if (quantity == 0 || quantity > MODBUS_MAX_INPUTS_TO_READ) // Check if illegal number of inputs was selected.
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUT_STATUS), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return false;
    }

    uint16_t currentAddr = addr;
    uint8_t byteCount = quantity / MODBUS_BYTE; // Number of bytes to iterate over to read bits.
    if (quantity % MODBUS_BYTE != 0)            // Quantity is not multiple of 8 so increase byteCount once so it can hold remaining bits.
        byteCount++;

    uint8_t resp[3 + byteCount];
    for (uint8_t i = 0; i < byteCount; i++) // Read one byte.
    {
        resp[i + 3] = 0;
        for (uint8_t j = 0; j < MODBUS_BYTE; j++) // Push specified bit to output buffer.
        {
            resp[i + 3] |= (DI[currentAddr] << j);

            currentAddr++;
            if (currentAddr - addr >= quantity) // Check if every selected output has been read and if so stop iterating.
                break;                          // Should be true when quantity is not multiply of 8.
        }
    }

    // Prepare response.
    resp[0] = id;
    resp[1] = MODBUS_READ_INPUT_STATUS;
    resp[2] = byteCount;
    sendResponse(resp, byteCount + 3);
    return true;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::readHoldingRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL) const
{
    if (id == MODBUS_ID_BROADCAST)
        return false;

    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    if (addr >= AQSize || addr + quantity > AQSize) // Check if illegal adress was selected.
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_HOLDING_REGISTER), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return false;
    }
    else if (quantity == 0 || quantity > MODBUS_MAX_HREGISTERS_TO_READ) // Check if illegal number of output registers was selected.
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_HOLDING_REGISTER), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return false;
    }

    uint8_t resp[3 + quantity];
    uint8_t byteCount = quantity * 2;                      // Quantity is in number of registers (2 bytes per one).
    for (uint8_t i = 0, j = 0; i < byteCount; i += 2, j++) // Read every register into two bytes.
    {
        resp[i + 3] = toHighByte(AQ[addr + j]);
        resp[i + 4] = toLowByte(AQ[addr + j]);
    }

    // Prepare response.
    resp[0] = id;
    resp[1] = MODBUS_READ_HOLDING_REGISTER;
    resp[2] = byteCount;
    sendResponse(resp, byteCount + 3);
    return true;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::readInputRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL) const
{
    if (id == MODBUS_ID_BROADCAST)
        return false;

    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    if (addr >= AISize || addr + quantity > AISize) // Check if illegal adress was selected.
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUT_REGISTER), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return false;
    }
    else if (quantity == 0 || quantity > MODBUS_MAX_IREGISTERS_TO_READ) // Check if illegal number of input registers was selected.
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUT_REGISTER), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return false;
    }

    uint8_t resp[3 + quantity];
    uint8_t byteCount = quantity * 2;                      // Quantity is in number of registers (2 bytes per one).
    for (uint8_t i = 0, j = 0; i < byteCount; i += 2, j++) // Read every register into two bytes.
    {
        resp[i + 3] = toHighByte(AI[addr + j]);
        resp[i + 4] = toLowByte(AI[addr + j]);
    }

    // Prepare response.
    resp[0] = id;
    resp[1] = MODBUS_READ_INPUT_REGISTER;
    resp[2] = byteCount;
    sendResponse(resp, byteCount + 3);
    return true;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::forceSingleCoil(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t valH, uint8_t valL)
{
    uint16_t addr = toWord(addrH, addrL);

    if (addr >= DQSize || DQSize == 0) // Check if illegal adress was selected.
    {
        uint8_t tab[3] = {id, toError(MODBUS_FORCE_SINGLE_COIL), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return false;
    }
    else if ((valH != 0xFF && valH != 0x00) || valL != 0x00) // Check if illegal value was selected (0x00 0x00 or 0xFF 0x00 are only available options).
    {
        uint8_t tab[3] = {id, toError(MODBUS_FORCE_SINGLE_COIL), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return false;
    }

    DQ[addr] = (bool)valH;

    // Prepare response.
    uint8_t tab[6] = {id, MODBUS_FORCE_SINGLE_COIL, addrH, addrL, valH, valL};
    sendResponse(tab, 6);
    return true;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::presetSingleRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t valH, uint8_t valL)
{
    uint16_t addr = toWord(addrH, addrL);
    uint16_t val = toWord(valH, valL);

    if (addr >= AQSize || AQSize == 0) // Check if illegal adress was selected.
    {
        uint8_t tab[3] = {id, toError(MODBUS_PRESET_SINGLE_REGISTER), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return false;
    }

    AQ[addr] = val;

    // Prepare response.
    uint8_t tab[6] = {id, MODBUS_PRESET_SINGLE_REGISTER, addrH, addrL, valH, valL};
    sendResponse(tab, 6);
    return true;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::forceMultipleCoils(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL, uint8_t byteCount, uint8_t command[], uint8_t commandLength)
{
    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    uint8_t requiredByteCount = quantity / MODBUS_BYTE; // Number of bytes required to write quantity number of coils.

    if (quantity % MODBUS_BYTE != 0) // Quantity is not multiple of 8 so increase desiredByteCount once for remainig bits.
        requiredByteCount++;

    uint8_t realNumberOfBytes = commandLength - MODBUS_FORCE_MULTIPLE_FIRST_BYTE_OF_DATA; // Number of bytes of data in given command.

    if (addr >= DQSize || addr + quantity > DQSize) // Check if illegal adress was selected.
    {
        uint8_t tab[3] = {id, toError(MODBUS_FORCE_MULTIPLE_COILS), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return false;
    }
    // Check if illegal number of coils was selected
    // or if required number of bytes for given quantity is not equal to byteCount provided by master
    // or if real number of data bytes is not equal to byteCount provided by master.
    else if (quantity == 0 || quantity > MODBUS_MAX_COILS_TO_WRITE || requiredByteCount != byteCount || realNumberOfBytes != byteCount)
    {
        uint8_t tab[3] = {id, toError(MODBUS_FORCE_MULTIPLE_COILS), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return false;
    }

    uint8_t currentAddr = addr;
    for (uint8_t i = 0; i < byteCount; i++)
    {
        for (uint8_t j = 0; j < MODBUS_BYTE; j++)
        {
            DQ[currentAddr] = command[i + MODBUS_FORCE_MULTIPLE_FIRST_BYTE_OF_DATA] & (1 << j); // Push specified bit of current byte to DQ storage.
            currentAddr++;
            if (currentAddr - addr >= quantity) // Check if every selected output has been read and if so stop iterating.
                break;                          // Should be true when quantity is not multiply of 8.
        }
    }

    uint8_t tab[6] = {id, MODBUS_FORCE_MULTIPLE_COILS, addrH, addrL, quantityH, quantityL};
    sendResponse(tab, 6);
    return true;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::forceMultipleRegisters(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL, uint8_t byteCount, uint8_t command[], uint8_t commandLength)
{
    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    uint8_t requiredByteCount = quantity * 2; // Number of bytes required to write quantity number of registers.

    uint8_t realNumberOfBytes = commandLength - MODBUS_FORCE_MULTIPLE_FIRST_BYTE_OF_DATA; // Number of bytes of data in given command.

    if (addr >= AQSize || addr + quantity > AQSize) // Check if illegal adress was selected.
    {
        uint8_t tab[3] = {id, toError(MODBUS_PRESET_MULTIPLE_REGISTERS), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return false;
    }
    // Check if illegal number of holding registers was selected
    // or if required number of bytes for given quantity is not equal to byteCount provided by master
    // or if real number of data bytes is not equal to byteCount provided by master.
    else if (quantity == 0 || quantity > MODBUS_MAX_HREGISTERS_TO_WRITE || requiredByteCount != byteCount || realNumberOfBytes != byteCount)
    {
        uint8_t tab[3] = {id, toError(MODBUS_PRESET_MULTIPLE_REGISTERS), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return false;
    }

    uint8_t currentAddr = addr;

    for (uint8_t i = 0; i < byteCount; i += 2) // Write two bytes into one two byte long register.
    {
        AQ[currentAddr] = toWord(command[i + MODBUS_FORCE_MULTIPLE_FIRST_BYTE_OF_DATA], command[i + MODBUS_FORCE_MULTIPLE_FIRST_BYTE_OF_DATA + 1]);
        currentAddr++;
    }

    uint8_t tab[6] = {id, MODBUS_PRESET_MULTIPLE_REGISTERS, addrH, addrL, quantityH, quantityL};
    sendResponse(tab, 6);
    return true;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
MSlave<DQSize, DISize, AQSize, AISize>::MSlave()
{
    for (uint16_t i = 0; i < DQSize; i++)
        DQ[i] = 0;
    for (uint16_t i = 0; i < DISize; i++)
        DI[i] = 0;
    for (uint16_t i = 0; i < AQSize; i++)
        AQ[i] = 0;
    for (uint16_t i = 0; i < AISize; i++)
        AI[i] = 0;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::begin(uint8_t id_, HardwareSerial &S_)
{
    id = id_;
    S = &S_;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::end()
{
    id = 255;
    S = nullptr;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::available() const
{
    // return not available when server is not configured properly
    if (S == nullptr || id < MODBUS_ID_MIN || id > MODBUS_ID_MAX)
        return false;
    return S->available();
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::useUART()
{
    uartUsed = 1;
    actAsTransmitter = nullptr;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::useRS485(void (*actAsTransmitter_)(bool))
{
    uartUsed = 0;
    actAsTransmitter = actAsTransmitter_;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::digitalRead(bool type, uint16_t addr) const
{
    if (type == COIL)
        return DQ[addr];
    else
        return DI[addr];
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
uint16_t MSlave<DQSize, DISize, AQSize, AISize>::analogRead(bool type, uint16_t addr) const
{
    if (type == HOLDING_REG)
        return AQ[addr];
    else
        return AI[addr];
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::digitalWrite(bool type, uint16_t addr, bool val)
{
    if (type == COIL)
        DQ[addr] = val;
    else
        DI[addr] = val;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::analogWrite(bool type, uint16_t addr, uint16_t val)
{
    if (type == HOLDING_REG)
        AQ[addr] = val;
    else
        AI[addr] = val;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
uint8_t MSlave<DQSize, DISize, AQSize, AISize>::read()
{
    if (!available())
        return 0; // Return nothing processed when there is no data pending.

    uint8_t command[MODBUS_MAX_FRAME_SIZE]; // Should be bigger probably (The maximum size of a MODBUS RTU frame is 256 bytes. - MODBUS over Serial Line  Specification and Implementation Guide  V1.02).
    uint8_t commandLength = S->readBytes(command, MODBUS_MAX_FRAME_SIZE);

    if (!crcDisabled) // Validate data and check crc.
    {
        if (commandLength < 2 + MODBUS_CRC_BYTE_COUNT) // ID and function code and two bytes of crc are necessary.
        {
            S->flush();
            return 0; // Return nothing processed if there is no required number of bytes.
        }
        uint8_t withoutCRC = commandLength - MODBUS_CRC_BYTE_COUNT;

        uint16_t givencrc = toWord(command[withoutCRC + 1], command[withoutCRC]);
        uint16_t computedcrc = crc(command, withoutCRC);
        if (givencrc != computedcrc)
        {
            S->flush();
            return 0; // Return nothing processed if crc is incorrect.
        }
        commandLength -= MODBUS_CRC_BYTE_COUNT; // Ignore crc bytes in command from now.
    }
    else // Validate without CRC.
    {
        if (commandLength < 2) // ID and function code is necessary.
        {
            S->flush();
            return 0; // Return nothing processed if there is no required number of bytes.
        }
    }

    if (command[MODBUS_ID] != MODBUS_ID_BROADCAST && command[MODBUS_ID] != id) // Other slave selected.
    {
        S->flush();
        return 0; // Return nothing processed if other slave was adressed.
    }

    if (busy) // Server is busy so ignore command.
    {
        uint8_t tab[3] = {command[MODBUS_ID], toError(command[MODBUS_FUNCTION_CODE]), MODBUS_ERR_SLAVE_BUSY};
        sendResponse(tab, 3);
        S->flush();
        return 0; // Return nothing processed if server is busy.
    }

    // Check if data contains minimal amount of bytes for given command (without crc).
    if ((command[MODBUS_FUNCTION_CODE] >= MODBUS_READ_COIL_STATUS && command[MODBUS_FUNCTION_CODE] <= MODBUS_PRESET_SINGLE_REGISTER && commandLength != 6) /* 6 bytes required for functions from MODBUS_READ_COIL_STATUS to MODBUS_PRESET_SINGLE_REGISTER */ ||
        (command[MODBUS_FUNCTION_CODE] == MODBUS_FORCE_MULTIPLE_COILS && commandLength < 8) /* minimum 8 bytes required for command MODBUS_FORCE_MULTIPLE_COILS */ ||
        (command[MODBUS_FUNCTION_CODE] == MODBUS_PRESET_MULTIPLE_REGISTERS && commandLength < 9) /* minimum 9 bytes required for command MODBUS_PRESET_MULTIPLE_REGISTERS */)
    {
        uint8_t tab[3] = {command[MODBUS_ID], toError(command[MODBUS_FUNCTION_CODE]), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return 0;
    }

    bool res = 0;

    switch (command[MODBUS_FUNCTION_CODE]) // Perform command.
    {
    case MODBUS_READ_COIL_STATUS:
        res = readCoilStatus(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_QUANTITY_HIGH], command[MODBUS_QUANTITY_LOW]);
        break;
    case MODBUS_READ_INPUT_STATUS:
        res = readInputStatus(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_QUANTITY_HIGH], command[MODBUS_QUANTITY_LOW]);
        break;
    case MODBUS_READ_HOLDING_REGISTER:
        res = readHoldingRegister(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_QUANTITY_HIGH], command[MODBUS_QUANTITY_LOW]);
        break;
    case MODBUS_READ_INPUT_REGISTER:
        res = readInputRegister(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_QUANTITY_HIGH], command[MODBUS_QUANTITY_LOW]);
        break;
    case MODBUS_FORCE_SINGLE_COIL:
        res = forceSingleCoil(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_VALUE_HIGH], command[MODBUS_VALUE_LOW]);
        break;
    case MODBUS_PRESET_SINGLE_REGISTER:
        res = presetSingleRegister(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_VALUE_HIGH], command[MODBUS_VALUE_LOW]);
        break;
    case MODBUS_FORCE_MULTIPLE_COILS:
        res = forceMultipleCoils(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_QUANTITY_HIGH], command[MODBUS_QUANTITY_LOW], command[MODBUS_BYTE_COUNT], command, commandLength);
        break;
    case MODBUS_PRESET_MULTIPLE_REGISTERS:
        res = forceMultipleRegisters(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_QUANTITY_HIGH], command[MODBUS_QUANTITY_LOW], command[MODBUS_BYTE_COUNT], command, commandLength);
        break;
    default:
        uint8_t tab[3] = {command[MODBUS_ID], toError(command[MODBUS_FUNCTION_CODE]), MODBUS_ERR_ILLEGAL_FUNCTION};
        sendResponse(tab, 3);
        break;
    }
    S->flush();
    if (res)
        return command[MODBUS_FUNCTION_CODE]; // Return function code of performed function.
    return 0;                                 // Return 0 if function failed for some reason.
}

#endif
