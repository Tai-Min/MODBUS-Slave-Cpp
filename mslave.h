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

    uint16_t AQ[AQSize]; //analog out
    bool DQ[DQSize];     //digital out
    uint16_t AI[AISize]; //analog in
    bool DI[DISize];     //digital in

    HardwareSerial *S;

    uint8_t toError(uint8_t code);
    uint16_t toWord(uint8_t H, uint8_t L);
    uint8_t toHighByte(uint16_t word);
    uint8_t toLowByte(uint16_t word);

    uint16_t crc(uint8_t command[], uint8_t commandLength);
    void sendResponse(uint8_t tab[], uint8_t length);

    void readCoilStatus(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL);
    void readInputStatus(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL);
    void readHoldingRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL);
    void readInputRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL);
    void forceSingleCoil(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t valH, uint8_t valL);
    void presetSingleRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t valH, uint8_t valL);
    void forceMultipleCoils(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL, uint8_t byteCount, uint8_t command[], uint8_t commandLength);
    void forceMultipleRegisters(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL, uint8_t byteCount, uint8_t command[], uint8_t commandLength);

  public:
    MSlave(uint8_t id_, HardwareSerial *S_);
    void enableCRC();
    void disableCRC();

    //read coil or input state
    //mode INPUT to read coil
    //mode OUTPUT to read input
    //because coil is an input for arduino server and output for the client
    //and input is an output for arduino server and input for the client
    bool digitalRead(uint16_t addr, bool mode);

    //write inputs only
    //input is an output for arduino server and input for the client
    void digitalWrite(uint16_t addr, bool val);

    //read holding register or input register
    //mode INPUT to read holding register
    //mode OUTPUT to read input register
    //because holding register is an input for arduino server and output for the client
    //and input register is an output for arduino server and input for the client
    uint16_t analogRead(uint16_t addr, bool mode);

    //write input registers only
    //input register is an output for arduino server and input for the client
    void analogWrite(uint16_t addr, uint16_t val);

    void event();
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

    uint16_t computedcrc = crc(tab, length);
    uint8_t lcrc = toLowByte(computedcrc);
    uint8_t hcrc = toHighByte(computedcrc);
    for (auto i = 0; i < length; i++)
    {
        S->write(tab[i]);
    }
    if (!crcDisabled)
    {
        S->write(lcrc);
        S->write(hcrc);
    }
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::readCoilStatus(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL)
{
    if (id == MODBUS_ID_BROADCAST)
        return;

    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    if (addr >= DQSize || DQSize == 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_OUTPUTS), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return;
    }
    else if (addr + quantity > DQSize || quantity < 1) //check if illegal number of outputs was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_OUTPUTS), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return;
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
    tab[1] = MODBUS_READ_OUTPUTS;
    tab[2] = byteCount;
    for (auto i = 0; i < byteCount; i++)
    {
        tab[i + 3] = outputs[i];
    }
    sendResponse(tab, byteCount + 3);
    return;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::readInputStatus(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL)
{
    if (id == MODBUS_ID_BROADCAST)
        return;

    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    if (addr >= DISize || DISize == 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUTS), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return;
    }
    else if (addr + quantity > DISize || quantity < 1) //check if illegal number of inputs was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUTS), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return;
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
    tab[1] = MODBUS_READ_INPUTS;
    tab[2] = byteCount;
    for (auto i = 0; i < byteCount; i++)
    {
        tab[i + 3] = inputs[i];
    }
    sendResponse(tab, byteCount + 3);
    return;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::readHoldingRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL)
{
    if (id == MODBUS_ID_BROADCAST)
        return;

    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    if (addr >= AQSize || AQSize == 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_OUTPUT_REGISTERS), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return;
    }
    else if (addr + quantity > AQSize || quantity < 1) //check if illegal number of output registers was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_OUTPUT_REGISTERS), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return;
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
    tab[1] = MODBUS_READ_OUTPUT_REGISTERS;
    tab[2] = byteCount;
    for (auto i = 0, j = 0; i < byteCount; i += 2, j++)
    {
        tab[i + 3] = valH[j];
        tab[i + 4] = valL[j];
    }
    sendResponse(tab, byteCount + 3);
    return;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::readInputRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL)
{
    if (id == MODBUS_ID_BROADCAST)
        return;

    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    if (addr >= AISize || AISize == 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUT_REGISTERS), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return;
    }
    else if (addr + quantity > AISize || quantity < 1) //check if illegal number of input registers was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUT_REGISTERS), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return;
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
    tab[1] = MODBUS_READ_INPUT_REGISTERS;
    tab[2] = byteCount;
    for (auto i = 0, j = 0; i < byteCount; i += 2, j++)
    {
        tab[i + 3] = valH[j];
        tab[i + 4] = valL[j];
    }
    sendResponse(tab, byteCount + 3);
    return;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::forceSingleCoil(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t valH, uint8_t valL)
{
    uint16_t addr = toWord(addrH, addrL);

    if (addr >= DQSize || DQSize == 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_WRITE_OUTPUT), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return;
    }
    else if ((valH != 0xFF && valH != 0x00) || valL != 0x00) //check if illegal value was selected (0x00 0x00 or 0xFF 0x00 are only available options)
    {
        uint8_t tab[3] = {id, toError(MODBUS_WRITE_OUTPUT), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return;
    }

    DQ[addr] = (bool)valH;

    uint8_t tab[6] = {id, MODBUS_WRITE_OUTPUT, addrH, addrL, valH, valL};
    sendResponse(tab, 6);
    return;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::presetSingleRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t valH, uint8_t valL)
{
    uint16_t addr = toWord(addrH, addrL);
    uint16_t val = toWord(valH, valL);

    if (addr >= AQSize || AQSize == 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_WRITE_REGISTER), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return;
    }

    AQ[addr] = val;

    uint8_t tab[6] = {id, MODBUS_WRITE_REGISTER, addrH, addrL, valH, valL};
    sendResponse(tab, 6);
    return;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::forceMultipleCoils(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL, uint8_t byteCount, uint8_t command[], uint8_t commandLength)
{
    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    uint8_t lastByteQuantity = quantity % MODBUS_BYTE; //quantity of used bits in last byte
    uint8_t desiredByteCount = quantity / MODBUS_BYTE;

    if (lastByteQuantity != 0)
        desiredByteCount++;

    if (addr >= DQSize || DQSize == 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_WRITE_N_OUTPUTS), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return;
    }
    else if (addr + quantity > DQSize || quantity < 1 || desiredByteCount != byteCount || commandLength - MODBUS_BYTE_COUNT - 1 != byteCount) //check if illegal number of input registers was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_WRITE_N_OUTPUTS), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return;
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

    uint8_t tab[6] = {id, MODBUS_WRITE_N_OUTPUTS, addrH, addrL, quantityH, quantityL};
    sendResponse(tab, 6);
    return;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
void MSlave<DQSize, DISize, AQSize, AISize>::forceMultipleRegisters(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL, uint8_t byteCount, uint8_t command[], uint8_t commandLength)
{
    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    uint8_t desiredByteCount = quantity * 2;

    if (addr >= AQSize || AQSize == 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_WRITE_N_REGISTERS), MODBUS_ERR_ILLEGAL_ADDR};
        sendResponse(tab, 3);
        return;
    }
    else if (addr + quantity > AQSize || quantity < 1 || desiredByteCount != byteCount || commandLength - MODBUS_BYTE_COUNT - 1 != byteCount) //check if illegal number of input registers was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_WRITE_N_REGISTERS), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return;
    }

    uint8_t lastByte = MODBUS_BYTE_COUNT + byteCount - 1;
    uint8_t start = MODBUS_BYTE_COUNT + 1;
    uint8_t currentAddr = addr;

    for (auto i = start; i <= lastByte; i += 2) //write two bytes into one word long register
    {
        AQ[currentAddr] = toWord(command[i], command[i + 1]);
        currentAddr++;
    }

    uint8_t tab[6] = {id, MODBUS_WRITE_N_REGISTERS, addrH, addrL, quantityH, quantityL};
    sendResponse(tab, 6);
    return;
}

template <uint16_t DQSize, uint16_t DISize, uint16_t AQSize, uint16_t AISize>
MSlave<DQSize, DISize, AQSize, AISize>::MSlave(uint8_t id_, HardwareSerial *S_)
    : id(id_), S(S_) {}

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
bool MSlave<DQSize, DISize, AQSize, AISize>::digitalRead(uint16_t addr, bool mode)
{
    if (mode == INPUT)
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
uint16_t MSlave<DQSize, DISize, AQSize, AISize>::analogRead(uint16_t addr, bool mode)
{
    if (mode == INPUT)
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
void MSlave<DQSize, DISize, AQSize, AISize>::event()
{
    if (S == nullptr || id < MODBUS_ID_MIN || id > MODBUS_ID_MAX)
        return;

    if (!(S->available() > 0))
        return;

    uint8_t command[MODBUS_MAX_FRAME_SIZE]; //should be bigger probably (The maximum size of a MODBUS RTU frame is 256 bytes. - MODBUS over Serial Line  Specification and Implementation Guide  V1.02)
    uint8_t commandLength = S->readBytes(command, MODBUS_MAX_FRAME_SIZE);

    if (!crcDisabled) //check crc when enabled
    {
        if (commandLength < 2 + MODBUS_CRC_BYTE_COUNT) //id and function code is necessary
        {
            S->flush();
            return;
        }
        commandLength -= MODBUS_CRC_BYTE_COUNT;

        uint16_t givencrc = toWord(command[commandLength + 1], command[commandLength]);
        uint16_t computedcrc = crc(command, commandLength);
        if (givencrc != computedcrc)
        {
            S->flush();
            return;
        }
    }
    else //else ignore crc stuff
    {
        if (commandLength < 2) //id and function code is necessary
        {
            S->flush();
            return;
        }
    }

    if (command[MODBUS_ID] != MODBUS_ID_BROADCAST && command[MODBUS_ID] != id)
    {
        S->flush();
        return;
    }

    if ((command[MODBUS_FUNCTION] >= MODBUS_READ_OUTPUTS && command[MODBUS_FUNCTION] <= MODBUS_WRITE_REGISTER && commandLength != 6) ||
        (command[MODBUS_FUNCTION] == MODBUS_WRITE_N_OUTPUTS && commandLength < 7) ||
        (command[MODBUS_FUNCTION] == MODBUS_WRITE_N_REGISTERS && commandLength < 7))
    {
        uint8_t tab[3] = {command[MODBUS_ID], toError(command[MODBUS_FUNCTION]), MODBUS_ERR_ILLEGAL_DATA};
        sendResponse(tab, 3);
        return;
    }

    switch (command[MODBUS_FUNCTION])
    {
    case MODBUS_READ_OUTPUTS:
        readCoilStatus(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_QUANTITY_HIGH], command[MODBUS_QUANTITY_LOW]);
        break;
    case MODBUS_READ_INPUTS:
        readInputStatus(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_QUANTITY_HIGH], command[MODBUS_QUANTITY_LOW]);
        break;
    case MODBUS_READ_OUTPUT_REGISTERS:
        readHoldingRegister(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_QUANTITY_HIGH], command[MODBUS_QUANTITY_LOW]);
        break;
    case MODBUS_READ_INPUT_REGISTERS:
        readInputRegister(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_QUANTITY_HIGH], command[MODBUS_QUANTITY_LOW]);
        break;
    case MODBUS_WRITE_OUTPUT:
        forceSingleCoil(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_VALUE_HIGH], command[MODBUS_VALUE_LOW]);
        break;
    case MODBUS_WRITE_REGISTER:
        presetSingleRegister(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_VALUE_HIGH], command[MODBUS_VALUE_LOW]);
        break;
    case MODBUS_WRITE_N_OUTPUTS:
        forceMultipleCoils(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_QUANTITY_HIGH], command[MODBUS_QUANTITY_LOW], command[MODBUS_BYTE_COUNT], command, commandLength);
        break;
    case MODBUS_WRITE_N_REGISTERS:
        forceMultipleRegisters(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_QUANTITY_HIGH], command[MODBUS_QUANTITY_LOW], command[MODBUS_BYTE_COUNT], command, commandLength);
        break;
    default:
        uint8_t tab[3] = {command[MODBUS_ID], toError(command[MODBUS_FUNCTION]), MODBUS_ERR_ILLEGAL_FUNCTION};
        sendResponse(tab, 3);
        break;
    }

    S->flush();
    return;
}

#endif