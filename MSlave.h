#ifndef MSLAVE_H
#define MSLAVE_H

#include <Arduino.h>
#include "mdefines.h"

//<coil, input, holding register, input register>
template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
class MSlave
{
private:
    bool crcDisabled = 0;
    uint8_t id = 255;

    bool busy = 0;

    bool uartUsed = 1;
    void (*actAsTransmitter)(bool) = nullptr;

    uint16_t AQ[AQSize]; //analog out
    bool DQ[DQSize];     //digital out
    uint16_t AI[AISize]; //analog in
    bool DI[DISize];     //digital in

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
     * @param type INPUT for value that can be changed by clients or this server (coils), OUTPUT for value that can be changed only by this server (inputs) .
     * @param addr Address of coil or input to be read.
     * @return State of an input or a coil.
     */
    bool digitalRead(bool type, uint16_t addr);

    /**
     * @brief Write input or a coil.
     * 
     * @param type INPUT for value that can be changed by clients or this server (coils), OUTPUT for value that can be changed only by this server (inputs) .
     * @param addr Address of input or coil to be written.
     * @param val Boolean value to be written.
     */
    void digitalWrite(bool type, uint16_t addr, bool val);

    /**
     * @brief Read state of requested register.
     * 
     * @param type INPUT for value that can be changed by clients or this server (holding registers), OUTPUT for value that can be changed only by this server (input registers). 
     * @param addr Address of input or holding register to be read.
     * @return Value of requested register.
     */
    uint16_t analogRead(bool type, uint16_t addr);

    /**
     * @brief Write value to requested register.
     * 
     * @param type INPUT for value that can be changed by clients or this server (holding registers), OUTPUT for value that can be changed only by this server (input registers). 
     * @param addr Address of input or holding register to be written.
     */
    void analogWrite(bool type, uint16_t addr, uint16_t val);

    //MODBUS naming convention for functions
    /**
     * @brief Write state to a coil.
     * 
     * @param address Address of a coil.
     * @param value Value to be written.
     */
    void writeCoil(uint16_t address, bool value) { digitalWrite(INPUT, address, value); }

    /**
     * @brief Write value to holding register.
     * 
     * @param address Address of a holding register.
     * @param value Value to be written.
     */
    void writeHoldingRegister(uint16_t address, uint16_t value) { analogWrite(INPUT, address, value); }

    /**
     * @brief Write value to a input.
     * 
     * @param address Address of an input.
     * @param value Value to be written.
     */
    void writeInput(uint16_t address, bool value) { digitalWrite(OUTPUT, address, value); }

    /**
     * @brief Write value to a input register.
     * 
     * @param address Address of an input register.
     * @param value Value to be written.
     */
    void writeInputRegister(uint16_t address, uint16_t value) { analogWrite(OUTPUT, address, value); }

    /**
     * @brief Read state of a coil.
     * 
     * @param address Address of a coil.
     * @return State of a coil.
     */
    bool readCoil(uint16_t address) { return digitalRead(INPUT, address); }

    /**
     * @brief Read state of an input.
     * 
     * @param address Address of an input.
     * @return State of an input.
     */
    bool readInput(uint16_t address) { return digitalRead(OUTPUT, address); }

    /**
     * @brief Read state of a holding register.
     * 
     * @param address Address of a holding register.
     * @return State of a holding register.
     */
    uint16_t readHoldingRegister(uint16_t address) { return analogRead(INPUT, address); }

    /**
     * @brief Read state of an input register.
     * 
     * @param address Address of an input register.
     * @return State of an input register.
     */
    uint16_t readInputRegister(uint16_t address) { return analogRead(OUTPUT, address); }

    /**
     * @brief Read and process data received from client device.
     * 
     * @returns MODBUS function code if data successfully processed or 0 if there was no data, error occured or invalid request happened.
     */
    uint8_t read();
};

//https://stackoverflow.com/questions/19347685/calculating-modbus-rtu-crc-16
template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
uint16_t MSlave<DQSize, DISize, AQSize, AISize>::crc(uint8_t command[], uint8_t commandLength)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t pos = 0; pos < commandLength; pos++)
    {
        crc ^= (uint16_t)command[pos]; // XOR byte into least sig. byte of crc

        for (uint8_t i = MODBUS_BYTE; i != 0; i--)
        { // Loop over each bit
            if ((crc & 0x0001) != 0)
            {              // If the LSB is set
                crc >>= 1; // Shift right and XOR 0xA001
                crc ^= 0xA001;
            }
            else           // Else LSB is not set
                crc >>= 1; // Just shift right
        }
    }
    // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
    return crc;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::sendResponse(uint8_t tab[], uint16_t length) const
{
    if (tab[MODBUS_ID] == MODBUS_ID_BROADCAST)
        return; //do not answer when broadcast id has been received

    uint8_t lcrc, hcrc;
    if (!crcDisabled)
    {
        uint16_t computedcrc = crc(tab, length);
        lcrc = toLowByte(computedcrc);
        hcrc = toHighByte(computedcrc);
    }

    if (!uartUsed && actAsTransmitter != nullptr)
        actAsTransmitter(true); //use function given by user to toggle rs485 into transmitter

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
        actAsTransmitter(false); //use function given by user to toggle rs485 into receiver
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::readCoilStatus(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL) const
{
    if (id == MODBUS_ID_BROADCAST)
        return 0;

    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    if (addr >= DQSize || addr + quantity > DQSize) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_COIL_STATUS), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return 0;
    }
    else if (quantity == 0 || quantity > MODBUS_MAX_COILS_TO_READ) //check if illegal number of outputs was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_COIL_STATUS), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return 0;
    }

    uint16_t currentAddr = addr;
    uint8_t byteCount = quantity / MODBUS_BYTE; //number of bytes to iterate over to read bits
    if (quantity % MODBUS_BYTE != 0)            //quantity is not multiple of 8 so increase byteCount once so it can hold remaining bits
        byteCount++;
    uint8_t outputs[byteCount];

    for (uint8_t i = 0; i < byteCount; i++) //read one byte
    {
        outputs[i] = 0;
        for (uint8_t j = 0; j < MODBUS_BYTE; j++) //read all bits in one byte
        {
            outputs[i] |= (DQ[currentAddr] << j); //push specified bit to current byte of output buffer

            currentAddr++;
            if (currentAddr - addr >= quantity) //check if every selected output has been read and if so stop iterating
                break;                          //should be true when quantity is not multiply of 8
        }
    }

    //prepare response
    uint8_t tab[3 + byteCount];
    tab[0] = id;
    tab[1] = MODBUS_READ_COIL_STATUS;
    tab[2] = byteCount;
    for (uint8_t i = 0; i < byteCount; i++)
    {
        tab[i + 3] = outputs[i];
    }
    sendResponse(tab, byteCount + 3);
    return 1;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::readInputStatus(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL) const
{
    if (id == MODBUS_ID_BROADCAST)
        return 0;

    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    if (addr >= DISize || addr + quantity > DISize) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUT_STATUS), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return 0;
    }
    else if (quantity == 0 || quantity > MODBUS_MAX_INPUTS_TO_READ) //check if illegal number of inputs was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUT_STATUS), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return 0;
    }

    uint16_t currentAddr = addr;
    uint8_t byteCount = quantity / MODBUS_BYTE; //number of bytes to iterate over to read bits
    if (quantity % MODBUS_BYTE != 0)            //quantity is not multiple of 8 so increase byteCount once so it can hold remaining bits
        byteCount++;
    uint8_t inputs[byteCount];

    for (uint8_t i = 0; i < byteCount; i++) //read one byte
    {
        inputs[i] = 0;
        for (uint8_t j = 0; j < MODBUS_BYTE; j++) //push specified bit to output buffer
        {
            inputs[i] |= (DI[currentAddr] << j);

            currentAddr++;
            if (currentAddr - addr >= quantity) //check if every selected output has been read and if so stop iterating
                break;                          //should be true when quantity is not multiply of 8
        }
    }

    //prepare response
    uint8_t tab[3 + byteCount];
    tab[0] = id;
    tab[1] = MODBUS_READ_INPUT_STATUS;
    tab[2] = byteCount;
    for (uint8_t i = 0; i < byteCount; i++)
    {
        tab[i + 3] = inputs[i];
    }
    sendResponse(tab, byteCount + 3);
    return 1;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::readHoldingRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL) const
{
    if (id == MODBUS_ID_BROADCAST)
        return 0;

    uint16_t addr = toWord(addrH, addrL);
    uint8_t quantity = toWord(quantityH, quantityL);

    if (addr >= AQSize || addr + quantity > AQSize) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_HOLDING_REGISTER), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return 0;
    }
    else if (quantity == 0 || quantity > MODBUS_MAX_HREGISTERS_TO_READ) //check if illegal number of output registers was selected (max (2^16-1)/2 floored)
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_HOLDING_REGISTER), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return 0;
    }

    uint8_t byteCount = quantity * 2; //quantity is in number of registers (2 bytes per one)
    uint8_t valL[quantity];
    uint8_t valH[quantity];
    for (uint8_t i = 0; i < quantity; i++) //read every register into two bytes
    {
        valL[i] = toLowByte(AQ[addr + i]);
        valH[i] = toHighByte(AQ[addr + i]);
    }

    uint8_t tab[3 + quantity];
    tab[0] = id;
    tab[1] = MODBUS_READ_HOLDING_REGISTER;
    tab[2] = byteCount;
    for (uint8_t i = 0, j = 0; i < byteCount; i += 2, j++)
    {
        tab[i + 3] = valH[j];
        tab[i + 4] = valL[j];
    }
    sendResponse(tab, byteCount + 3);
    return 1;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::readInputRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL) const
{
    if (id == MODBUS_ID_BROADCAST)
        return 0;

    uint16_t addr = toWord(addrH, addrL);
    uint8_t quantity = toWord(quantityH, quantityL);

    if (addr >= AISize || addr + quantity > AISize) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUT_REGISTER), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return 0;
    }
    else if (quantity == 0 || quantity > MODBUS_MAX_IREGISTERS_TO_READ) //check if illegal number of input registers was selected (max (2^16-1)/2 floored)
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUT_REGISTER), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return 0;
    }

    uint8_t byteCount = quantity * 2; //quantity is in number of registers (2 bytes per one)
    uint8_t valL[quantity];
    uint8_t valH[quantity];
    for (uint8_t i = 0; i < quantity; i++) //read every register into two bytes
    {
        valL[i] = toLowByte(AI[addr + i]);
        valH[i] = toHighByte(AI[addr + i]);
    }

    //prepare response
    uint8_t tab[3 + quantity];
    tab[0] = id;
    tab[1] = MODBUS_READ_INPUT_REGISTER;
    tab[2] = byteCount;
    for (uint8_t i = 0, j = 0; i < byteCount; i += 2, j++)
    {
        tab[i + 3] = valH[j];
        tab[i + 4] = valL[j];
    }
    sendResponse(tab, byteCount + 3);
    return 1;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::forceSingleCoil(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t valH, uint8_t valL)
{
    uint16_t addr = toWord(addrH, addrL);

    if (addr >= DQSize || DQSize == 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_FORCE_SINGLE_COIL), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return 0;
    }
    else if ((valH != 0xFF && valH != 0x00) || valL != 0x00) //check if illegal value was selected (0x00 0x00 or 0xFF 0x00 are only available options)
    {
        uint8_t tab[3] = {id, toError(MODBUS_FORCE_SINGLE_COIL), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return 0;
    }

    DQ[addr] = (bool)valH;

    uint8_t tab[6] = {id, MODBUS_FORCE_SINGLE_COIL, addrH, addrL, valH, valL};
    sendResponse(tab, 6);
    return 1;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::presetSingleRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t valH, uint8_t valL)
{
    uint16_t addr = toWord(addrH, addrL);
    uint16_t val = toWord(valH, valL);

    if (addr >= AQSize || AQSize == 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_PRESET_SINGLE_REGISTER), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return 0;
    }

    AQ[addr] = val;

    uint8_t tab[6] = {id, MODBUS_PRESET_SINGLE_REGISTER, addrH, addrL, valH, valL};
    sendResponse(tab, 6);
    return 1;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::forceMultipleCoils(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL, uint8_t byteCount, uint8_t command[], uint8_t commandLength)
{
    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    uint8_t requiredByteCount = quantity / MODBUS_BYTE; //number of bytes required to write quantity number of coils

    if (quantity % MODBUS_BYTE != 0) //quantity is not multiple of 8 so increase desiredByteCount once for remainig bits
        requiredByteCount++;

    uint8_t realNumberOfBytes = commandLength - MODBUS_FORCE_MULTIPLE_FIRST_BYTE_OF_DATA; //number of bytes of data in given command

    if (addr >= DQSize || addr + quantity > DQSize) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_FORCE_MULTIPLE_COILS), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return 0;
    }
    else if (quantity == 0 || quantity > MODBUS_MAX_COILS_TO_WRITE || requiredByteCount != byteCount || realNumberOfBytes != byteCount) //check if illegal number of input registers was selected
    {                                                                                                                                   //or if number of bytes required for given quntity is equal to byte count
        uint8_t tab[3] = {id, toError(MODBUS_FORCE_MULTIPLE_COILS), MODBUS_ERR_ILLEGAL_DATA};                                           //or if number of bytes after byte count is equal to byte count
        sendResponse(tab, 3);
        return 0;
    }

    uint8_t currentAddr = addr;
    for (uint8_t i = 0; i < byteCount; i++)
    {
        for (uint8_t j = 0; j < MODBUS_BYTE; j++)
        {
            DQ[currentAddr] = command[i + MODBUS_FORCE_MULTIPLE_FIRST_BYTE_OF_DATA] & (1 << j); //push specified bit of current byte to DQ storage
            currentAddr++;
            if (currentAddr - addr >= quantity) //check if every selected output has been read and if so stop iterating
                break;                          //should be true when quantity is not multiply of 8
        }
    }

    uint8_t tab[6] = {id, MODBUS_FORCE_MULTIPLE_COILS, addrH, addrL, quantityH, quantityL};
    sendResponse(tab, 6);
    return 1;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::forceMultipleRegisters(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL, uint8_t byteCount, uint8_t command[], uint8_t commandLength)
{
    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    uint8_t requiredByteCount = quantity * 2; //number of bytes required to write quantity number of coils

    uint8_t realNumberOfBytes = commandLength - MODBUS_FORCE_MULTIPLE_FIRST_BYTE_OF_DATA; //number of bytes of data in given command

    if (addr >= AQSize || addr + quantity > AQSize) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_PRESET_MULTIPLE_REGISTERS), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return 0;
    }
    else if (quantity == 0 || quantity > MODBUS_MAX_HREGISTERS_TO_WRITE || requiredByteCount != byteCount || realNumberOfBytes != byteCount) //check if illegal number of input registers was selected
    {                                                                                                                                        //or if number of bytes required for given quntity is equal to byte count
        uint8_t tab[3] = {id, toError(MODBUS_PRESET_MULTIPLE_REGISTERS), MODBUS_ERR_ILLEGAL_DATA};                                           //or if number of bytes after byte count is equal to byte count
        sendResponse(tab, 3);
        return 0;
    }

    uint8_t currentAddr = addr;

    for (auto i = 0; i < byteCount; i += 2) //write two bytes into one two byte long register
    {
        AQ[currentAddr] = toWord(command[i + MODBUS_FORCE_MULTIPLE_FIRST_BYTE_OF_DATA], command[i + MODBUS_FORCE_MULTIPLE_FIRST_BYTE_OF_DATA + 1]);
        currentAddr++;
    }

    uint8_t tab[6] = {id, MODBUS_PRESET_MULTIPLE_REGISTERS, addrH, addrL, quantityH, quantityL};
    sendResponse(tab, 6);
    return 1;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
MSlave<DQSize, DISize, AQSize, AISize>::MSlave()
{
    for (auto i = 0; i < DQSize; i++)
        DQ[i] = 0;
    for (auto i = 0; i < DISize; i++)
        DI[i] = 0;
    for (auto i = 0; i < AQSize; i++)
        AQ[i] = 0;
    for (auto i = 0; i < AISize; i++)
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
    //return not available when server is not configured properly
    if (S == nullptr || id < MODBUS_ID_MIN || id > MODBUS_ID_MAX)
        return 0;
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
bool MSlave<DQSize, DISize, AQSize, AISize>::digitalRead(bool type, uint16_t addr)
{
    if (type == INPUT)
    {
        return DQ[addr];
    }
    else
    {
        return DI[addr];
    }
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
uint16_t MSlave<DQSize, DISize, AQSize, AISize>::analogRead(bool type, uint16_t addr)
{
    if (type == INPUT)
    {
        return AQ[addr];
    }
    else
    {
        return AI[addr];
    }
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::digitalWrite(bool type, uint16_t addr, bool val)
{
    if (type == INPUT)
    {
        DQ[addr] = val;
    }
    else
    {
        DI[addr] = val;
    }
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::analogWrite(bool type, uint16_t addr, uint16_t val)
{
    if (type == INPUT)
    {
        AQ[addr] = val;
    }
    else
    {
        AI[addr] = val;
    }
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
uint8_t MSlave<DQSize, DISize, AQSize, AISize>::read()
{
    if (!available())
        return 0; //return nothing processed when there is no data pending

    uint8_t command[MODBUS_MAX_FRAME_SIZE]; //should be bigger probably (The maximum size of a MODBUS RTU frame is 256 bytes. - MODBUS over Serial Line  Specification and Implementation Guide  V1.02)
    uint8_t commandLength = S->readBytes(command, MODBUS_MAX_FRAME_SIZE);

    if (!crcDisabled) //validate data and check crc
    {
        if (commandLength < 2 + MODBUS_CRC_BYTE_COUNT) //id and function code and two bytes of crc are necessary
        {
            S->flush();
            return 0; //return nothing processed if there is no required number of bytes
        }
        uint8_t withoutCRC = commandLength - MODBUS_CRC_BYTE_COUNT;

        uint16_t givencrc = toWord(command[withoutCRC + 1], command[withoutCRC]);
        uint16_t computedcrc = crc(command, withoutCRC);
        if (givencrc != computedcrc)
        {
            S->flush();
            return 0; //return nothing processed if crc is incorrect
        }
        commandLength -= MODBUS_CRC_BYTE_COUNT; //ignore crc bytes in command from now
    }
    else //validate without CRC
    {
        if (commandLength < 2) //id and function code is necessary
        {
            S->flush();
            return 0; //return nothing processed if there is no required number of bytes
        }
    }

    if (command[MODBUS_ID] != MODBUS_ID_BROADCAST && command[MODBUS_ID] != id) //other slave selected
    {
        S->flush();
        return 0; //return nothing processed if other slave was adressed
    }

    if (busy) //server is busy so ignore command
    {
        uint8_t tab[3] = {command[MODBUS_ID], toError(command[MODBUS_FUNCTION_CODE]), MODBUS_ERR_SLAVE_BUSY};
        sendResponse(tab, 3);
        S->flush();
        return 0; //return nothing processed if server is busy
    }

    //check if data contains minimal amount of bytes for given command (without crc)
    if ((command[MODBUS_FUNCTION_CODE] >= MODBUS_READ_COIL_STATUS && command[MODBUS_FUNCTION_CODE] <= MODBUS_PRESET_SINGLE_REGISTER && commandLength != 6) /*6 bytes required for functions from MODBUS_READ_COIL_STATUS to MODBUS_PRESET_SINGLE_REGISTER*/ ||
        (command[MODBUS_FUNCTION_CODE] == MODBUS_FORCE_MULTIPLE_COILS && commandLength < 8) /*minimum 8 bytes required for command MODBUS_FORCE_MULTIPLE_COILS*/ ||
        (command[MODBUS_FUNCTION_CODE] == MODBUS_PRESET_MULTIPLE_REGISTERS && commandLength < 9) /*minimum 9 bytes required for command MODBUS_PRESET_MULTIPLE_REGISTERS*/)
    {
        uint8_t tab[3] = {command[MODBUS_ID], toError(command[MODBUS_FUNCTION_CODE]), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return 0;
    }

    bool res = 0;
    switch (command[MODBUS_FUNCTION_CODE]) //perform command
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
        return command[MODBUS_FUNCTION_CODE]; //return function code of performed function
    return 0;                                 //return 0 if function failed for some reason
}

#endif
