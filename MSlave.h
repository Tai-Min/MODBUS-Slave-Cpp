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

    bool uartUsed = 1;
    void (*actAsTransmitter)(bool) = nullptr;

    uint16_t AQ[AQSize]; //analog out
    bool DQ[DQSize];     //digital out
    uint16_t AI[AISize]; //analog in
    bool DI[DISize];     //digital in

    HardwareSerial *S = nullptr;

    uint8_t toError(uint8_t code);
    uint16_t toWord(uint8_t H, uint8_t L);
    uint8_t toHighByte(uint16_t word);
    uint8_t toLowByte(uint16_t word);

    uint16_t crc(uint8_t command[], uint8_t commandLength);
    void sendResponse(uint8_t tab[], uint8_t length);

    bool readCoilStatus(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL);
    bool readInputStatus(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL);
    bool readHoldingRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL);
    bool readInputRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL);
    bool forceSingleCoil(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t valH, uint8_t valL);
    bool presetSingleRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t valH, uint8_t valL);
    bool forceMultipleCoils(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL, uint8_t byteCount, uint8_t command[], uint8_t commandLength);
    bool forceMultipleRegisters(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL, uint8_t byteCount, uint8_t command[], uint8_t commandLength);

  public:
    MSlave();

    //enable this slave
    //this slave will be enabled only if id is a valid one and S points to non nullptr
    void begin(uint8_t id_, HardwareSerial &S_);

    //disable this slave
    void end();

    //check whether some data is pending in Serial S
    bool available();

    //enable CRC in request/response/exception frames
    void enableCRC();

    //disable CRC in request/response/exception frames
    void disableCRC();

    //use standard serial
    void useUART();

    //when bool in passed function is true, RS485 converte act as transmitter
    void useRS485(void (*actAsTransmitter_)(bool));

    //read coil or input state
    //mode INPUT to read coil
    //mode OUTPUT to read input
    //because coil is an input for arduino server and output for the client
    //and input is an output for arduino server and input for the client
    bool digitalRead(bool type, uint16_t addr);

    //write inputs only
    //input is an output for arduino server and input for the client
    void digitalWrite(uint16_t addr, bool val);

    //read holding register or input register
    //mode INPUT to read holding register
    //mode OUTPUT to read input register
    //because holding register is an input for arduino server and output for the client
    //and input register is an output for arduino server and input for the client
    uint16_t analogRead(bool type, uint16_t addr);

    //write input registers only
    //input register is an output for arduino server and input for the client
    void analogWrite(uint16_t addr, uint16_t val);

    //read data from Serial S and process it
    //returns function code when data was successfully processed
    //returns 0 when there was no data / error occured / invalid request happened
    uint8_t read();
};

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
uint8_t MSlave<DQSize, DISize, AQSize, AISize>::toError(uint8_t code)
{
    return code + MODBUS_ERR_OFFSET;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
uint16_t MSlave<DQSize, DISize, AQSize, AISize>::toWord(uint8_t H, uint8_t L)
{
    return ((uint16_t)H << MODBUS_BYTE) | L;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
uint8_t MSlave<DQSize, DISize, AQSize, AISize>::toHighByte(uint16_t word)
{
    return (word >> MODBUS_BYTE);
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
uint8_t MSlave<DQSize, DISize, AQSize, AISize>::toLowByte(uint16_t word)
{
    return word;
}

//https://stackoverflow.com/questions/19347685/calculating-modbus-rtu-crc-16
template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
uint16_t MSlave<DQSize, DISize, AQSize, AISize>::crc(uint8_t command[], uint8_t commandLength)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t pos = 0; pos < commandLength; pos++)
    {
        crc ^= (uint16_t)command[pos]; // XOR byte into least sig. byte of crc

        for (auto i = MODBUS_BYTE; i != 0; i--)
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
void MSlave<DQSize, DISize, AQSize, AISize>::sendResponse(uint8_t tab[], uint8_t length)
{
    if (tab[MODBUS_ID] == MODBUS_ID_BROADCAST)
        return; //do not send anything for broadcast

    uint8_t lcrc, hcrc;
    if(!crcDisabled)
    {
        uint16_t computedcrc = crc(tab, length);
        lcrc = toLowByte(computedcrc);
        hcrc = toHighByte(computedcrc);
    }

    if(!uartUsed && actAsTransmitter != nullptr)
        actAsTransmitter(true);
    for (auto i = 0; i < length; i++)
    {
        S->write(tab[i]);
    }
    if (!crcDisabled)
    {
        S->write(lcrc);
        S->write(hcrc);
    }
    if(!uartUsed && actAsTransmitter != nullptr)
        actAsTransmitter(false);
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::readCoilStatus(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL)
{
    if (id == MODBUS_ID_BROADCAST)
        return 0;

    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    if (addr >= DQSize || DQSize == 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_COIL_STATUS), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return 0;
    }
    else if (addr + quantity > DQSize || quantity < 1) //check if illegal number of outputs was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_COIL_STATUS), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return 0;
    }

    uint16_t currentAddr = addr;
    uint8_t byteCount = quantity / MODBUS_BYTE;
    if (quantity % MODBUS_BYTE != 0)
        byteCount++;
    uint8_t outputs[byteCount];

    for (auto i = 0; i < byteCount; i++) //push output values to specified byte
    {
        outputs[i] = 0;
        for (auto j = 0; j < MODBUS_BYTE; j++) //push output value to specified bit in byte
        {
            outputs[i] |= (DQ[currentAddr] << j);

            currentAddr++;
            if (currentAddr - addr >= quantity) //check if every selected output has been read
                break;
        }
    }

    uint8_t tab[3 + byteCount];
    tab[0] = id;
    tab[1] = MODBUS_READ_COIL_STATUS;
    tab[2] = byteCount;
    for (auto i = 0; i < byteCount; i++)
    {
        tab[i + 3] = outputs[i];
    }
    sendResponse(tab, byteCount + 3);
    return 1;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::readInputStatus(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL)
{
    if (id == MODBUS_ID_BROADCAST)
        return 0;

    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    if (addr >= DISize || DISize == 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUT_STATUS), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return 0;
    }
    else if (addr + quantity > DISize || quantity < 1) //check if illegal number of inputs was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUT_STATUS), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return 0;
    }

    uint16_t currentAddr = addr;
    uint8_t byteCount = quantity / 8;
    if (quantity % MODBUS_BYTE != 0)
        byteCount++;
    uint8_t inputs[byteCount];

    for (auto i = 0; i < byteCount; i++) //push input values to specified byte
    {
        inputs[i] = 0;
        for (auto j = 0; j < MODBUS_BYTE; j++) //push input value to specified bit in byte
        {
            inputs[i] |= (DI[currentAddr] << j);

            currentAddr++;
            if (currentAddr - addr >= quantity) //check if every selected input has been read
                break;
        }
    }

    uint8_t tab[3 + byteCount];
    tab[0] = id;
    tab[1] = MODBUS_READ_INPUT_STATUS;
    tab[2] = byteCount;
    for (auto i = 0; i < byteCount; i++)
    {
        tab[i + 3] = inputs[i];
    }
    sendResponse(tab, byteCount + 3);
    return 1;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::readHoldingRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL)
{
    if (id == MODBUS_ID_BROADCAST)
        return 0;

    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    if (addr >= AQSize || AQSize == 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_HOLDING_REGISTER), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return 0;
    }
    else if (addr + quantity > AQSize || quantity < 1) //check if illegal number of output registers was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_HOLDING_REGISTER), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return 0;
    }

    uint8_t byteCount = quantity * 2;
    uint8_t valL[quantity];
    uint8_t valH[quantity];
    for (auto i = 0; i < quantity; i++) //read every selected value into two bytes
    {
        valL[i] = toLowByte(AQ[addr + i]);
        valH[i] = toHighByte(AQ[addr + i]);
    }

    uint8_t tab[3 + quantity];
    tab[0] = id;
    tab[1] = MODBUS_READ_HOLDING_REGISTER;
    tab[2] = byteCount;
    for (auto i = 0, j = 0; i < byteCount; i += 2, j++)
    {
        tab[i + 3] = valH[j];
        tab[i + 4] = valL[j];
    }
    sendResponse(tab, byteCount + 3);
    return 1;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
bool MSlave<DQSize, DISize, AQSize, AISize>::readInputRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL)
{
    if (id == MODBUS_ID_BROADCAST)
        return 0;

    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    if (addr >= AISize || AISize == 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUT_REGISTER), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return 0;
    }
    else if (addr + quantity > AISize || quantity < 1) //check if illegal number of input registers was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUT_REGISTER), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return 0;
    }

    uint8_t byteCount = quantity * 2;
    uint8_t valL[quantity];
    uint8_t valH[quantity];
    for (auto i = 0; i < quantity; i++) //read every selected value into two bytes
    {
        valL[i] = toLowByte(AI[addr + i]);
        valH[i] = toHighByte(AI[addr + i]);
    }

    uint8_t tab[3 + quantity];
    tab[0] = id;
    tab[1] = MODBUS_READ_INPUT_REGISTER;
    tab[2] = byteCount;
    for (auto i = 0, j = 0; i < byteCount; i += 2, j++)
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

    uint8_t lastByteQuantity = quantity % MODBUS_BYTE; //quantity of used bits in last byte
    uint8_t desiredByteCount = quantity / MODBUS_BYTE;

    if (lastByteQuantity != 0)
        desiredByteCount++;

    if (addr >= DQSize || DQSize == 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_FORCE_MULTIPLE_COILS), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return 0;
    }
    else if (addr + quantity > DQSize || quantity < 1 || desiredByteCount != byteCount || commandLength - MODBUS_BYTE_COUNT - 1 != byteCount) //check if illegal number of input registers was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_FORCE_MULTIPLE_COILS), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return 0;
    }

    uint8_t start = MODBUS_BYTE_COUNT + 1;
    uint8_t currentAddr = addr;
    for (auto i = start; i < start + byteCount; i++)
    {
        for (auto j = 0; j < MODBUS_BYTE; j++)
        {
            DQ[currentAddr] = command[i] & (1 << j);
            currentAddr++;
            if (currentAddr - addr >= quantity) //check if every selected output has been written to
                break;
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

    uint8_t desiredByteCount = quantity * 2;

    if (addr >= AQSize || AQSize == 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_PRESET_MULTIPLE_REGISTERS), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return 0;
    }
    else if (addr + quantity > AQSize || quantity < 1 || desiredByteCount != byteCount || commandLength - MODBUS_BYTE_COUNT - 1 != byteCount) //check if illegal number of input registers was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_PRESET_MULTIPLE_REGISTERS), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return 0;
    }

    uint8_t lastByte = MODBUS_BYTE_COUNT + byteCount - 1;
    uint8_t start = MODBUS_BYTE_COUNT + 1;
    uint8_t currentAddr = addr;

    for (auto i = start; i <= lastByte; i += 2) //write two bytes into one word long register
    {
        AQ[currentAddr] = toWord(command[i], command[i + 1]);
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
bool MSlave<DQSize, DISize, AQSize, AISize>::available()
{
    if (S == nullptr || id < MODBUS_ID_MIN || id > MODBUS_ID_MAX)
        return 0;
    return (S->available() > 0);
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::enableCRC()
{
    crcDisabled = 0;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::disableCRC()
{
    crcDisabled = 1;
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
void MSlave<DQSize, DISize, AQSize, AISize>::digitalWrite(uint16_t addr, bool val)
{
    DI[addr] = val;
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
void MSlave<DQSize, DISize, AQSize, AISize>::analogWrite(uint16_t addr, uint16_t val)
{
    AI[addr] = val;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
uint8_t MSlave<DQSize, DISize, AQSize, AISize>::read()
{
    if (!available())
        return 0;

    uint8_t command[MODBUS_MAX_FRAME_SIZE]; //should be bigger probably (The maximum size of a MODBUS RTU frame is 256 bytes. - MODBUS over Serial Line  Specification and Implementation Guide  V1.02)
    uint8_t commandLength = S->readBytes(command, MODBUS_MAX_FRAME_SIZE);

    if (!crcDisabled) //validate data and check crc
    {
        if (commandLength < 2 + MODBUS_CRC_BYTE_COUNT) //id and function code and two bytes of crc are necessary
        {
            S->flush();
            return 0;
        }
        uint8_t withoutCRC = commandLength - MODBUS_CRC_BYTE_COUNT;
        //commandLength -= MODBUS_CRC_BYTE_COUNT;//ignore crc bytes in command from now

        uint16_t givencrc = toWord(command[withoutCRC + 1], command[withoutCRC]);
        uint16_t computedcrc = crc(command, withoutCRC);
        if (givencrc != computedcrc)
        {
            S->flush();
            return 0;
        }
        commandLength -= MODBUS_CRC_BYTE_COUNT;//ignore crc bytes in command from now
    }
    else //validate without CRC
    {
        if (commandLength < 2) //id and function code is necessary
        {
            S->flush();
            return 0;
        }
    }

    if (command[MODBUS_ID] != MODBUS_ID_BROADCAST && command[MODBUS_ID] != id)//data not designated for this slave
    {
        S->flush();
        return 0;
    }

    //check if data contains minimal amount of bytes for given command 
    if ((command[MODBUS_FUNCTION_CODE] >= MODBUS_READ_COIL_STATUS && command[MODBUS_FUNCTION_CODE] <= MODBUS_PRESET_SINGLE_REGISTER && commandLength != 6) ||
        (command[MODBUS_FUNCTION_CODE] == MODBUS_FORCE_MULTIPLE_COILS && commandLength < 7) ||
        (command[MODBUS_FUNCTION_CODE] == MODBUS_PRESET_MULTIPLE_REGISTERS && commandLength < 7))
    {
        uint8_t tab[3] = {command[MODBUS_ID], toError(command[MODBUS_FUNCTION_CODE]), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return 0;
    }
    bool res = 0;
    switch (command[MODBUS_FUNCTION_CODE])
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
        return command[MODBUS_FUNCTION_CODE];
    return 0;
}

#endif