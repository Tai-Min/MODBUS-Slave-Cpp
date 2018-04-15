#include "modbus_slave.h"

uint8_t MSlave::id;
bool MSlave::active = 0;

int MSlave::AQSize;
int MSlave::DQSize;
int MSlave::AISize;
int MSlave::DISize;

HardwareSerial *MSlave::S;

//memory map
uint16_t *MSlave::AQ = nullptr;
bool *MSlave::DQ = nullptr;
uint16_t *MSlave::AI = nullptr;
bool *MSlave::DI = nullptr;

uint8_t MSlave::toError(uint8_t code)
{
    return code + MODBUS_ERR_OFFSET;
}

uint16_t MSlave::toWord(uint8_t H, uint8_t L)
{
    return ((uint16_t)H << MODBUS_BYTE) | L;
}

uint8_t MSlave::toHighByte(uint16_t word)
{
    return (word >> MODBUS_BYTE);
}

uint8_t MSlave::toLowByte(uint16_t word)
{
    return word;
}

//https://stackoverflow.com/questions/19347685/calculating-modbus-rtu-crc-16
uint16_t MSlave::crc(uint8_t command[], uint8_t commandLength)
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

void MSlave::sendData(uint8_t tab[], uint8_t length)
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
    //not sure about byte order here and in master crc sendFrame function
    S->write(lcrc);
    S->write(hcrc);
}

void MSlave::readOutputs(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL)
{
    if (id == MODBUS_ID_BROADCAST)
        return;

    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    if (addr > DQSize - 1 || addr < 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_OUTPUTS), MODBUS_ERR_ILLEGAL_ADDR};
        sendData(tab, 3);
        return;
    }
    else if (addr + quantity - 1 > DQSize - 1 || quantity < 1) //check if illegal number of outputs was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_OUTPUTS), MODBUS_ERR_ILLEGAL_DATA};
        sendData(tab, 3);
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
    sendData(tab, byteCount + 3);
    return;
}

void MSlave::readInputs(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL)
{
    if (id == MODBUS_ID_BROADCAST)
        return;

    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    if (addr > DISize - 1 || addr < 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUTS), MODBUS_ERR_ILLEGAL_ADDR};
        sendData(tab, 3);
        return;
    }
    else if (addr + quantity - 1 > DISize - 1 || quantity < 1) //check if illegal number of inputs was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUTS), MODBUS_ERR_ILLEGAL_DATA};
        sendData(tab, 3);
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
    sendData(tab, byteCount + 3);
    return;
}

void MSlave::readOutputRegisters(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL)
{
    if (id == MODBUS_ID_BROADCAST)
        return;

    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    if (addr > AQSize - 1 || addr < 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_OUTPUT_REGISTERS), MODBUS_ERR_ILLEGAL_ADDR};
        sendData(tab, 3);
        return;
    }
    else if (addr + quantity - 1 > AQSize - 1 || quantity < 1) //check if illegal number of output registers was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_OUTPUT_REGISTERS), MODBUS_ERR_ILLEGAL_DATA};
        sendData(tab, 3);
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
    sendData(tab, byteCount + 3);
    return;
}

void MSlave::readInputRegisters(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL)
{
    if (id == MODBUS_ID_BROADCAST)
        return;

    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    if (addr > AISize - 1 || addr < 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUT_REGISTERS), MODBUS_ERR_ILLEGAL_ADDR};
        sendData(tab, 3);
        return;
    }
    else if (addr + quantity - 1 > AISize - 1 || quantity < 1) //check if illegal number of input registers was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_READ_INPUT_REGISTERS), MODBUS_ERR_ILLEGAL_DATA};
        sendData(tab, 3);
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
    sendData(tab, byteCount + 3);
    return;
}

void MSlave::writeOutput(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t valH, uint8_t valL)
{
    uint16_t addr = toWord(addrH, addrL);

    if (addr > DQSize - 1 || addr < 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_WRITE_OUTPUT), MODBUS_ERR_ILLEGAL_ADDR};
        sendData(tab, 3);
        return;
    }
    else if ((valH != 0xFF && valH != 0x00) || valL != 0x00) //check if illegal value was selected (0x00 0x00 or 0xFF 0x00 are only available options)
    {
        uint8_t tab[3] = {id, toError(MODBUS_WRITE_OUTPUT), MODBUS_ERR_ILLEGAL_DATA};
        sendData(tab, 3);
        return;
    }

    DQ[addr] = (bool)valH;

    uint8_t tab[6] = {id, MODBUS_WRITE_OUTPUT, addrH, addrL, valH, valL};
    sendData(tab, 6);
    return;
}

void MSlave::writeRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t valH, uint8_t valL)
{
    uint16_t addr = toWord(addrH, addrL);
    uint16_t val = toWord(valH, valL);

    if (addr > AQSize - 1 || addr < 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_WRITE_REGISTER), MODBUS_ERR_ILLEGAL_ADDR};
        sendData(tab, 3);
        return;
    }

    AQ[addr] = val;

    uint8_t tab[6] = {id, MODBUS_WRITE_REGISTER, addrH, addrL, valH, valL};
    sendData(tab, 6);
    return;
}

void MSlave::writeNOutputs(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL, uint8_t byteCount, uint8_t command[], uint8_t commandLength)
{
    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    uint8_t lastByteQuantity = quantity % MODBUS_BYTE; //quantity of used bits in last byte
    uint8_t desiredByteCount = quantity / MODBUS_BYTE;

    if (lastByteQuantity != 0)
        desiredByteCount++;

    if (addr > DQSize - 1 || addr < 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_WRITE_N_OUTPUTS), MODBUS_ERR_ILLEGAL_ADDR};
        sendData(tab, 3);
        return;
    }
    else if (addr + quantity - 1 > DQSize - 1 || quantity < 1 || desiredByteCount != byteCount || commandLength - MODBUS_BYTE_COUNT - 1 != byteCount) //check if illegal number of input registers was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_WRITE_N_OUTPUTS), MODBUS_ERR_ILLEGAL_DATA};
        sendData(tab, 3);
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
    sendData(tab, 6);
    return;
}

void MSlave::writeNRegisters(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL, uint8_t byteCount, uint8_t command[], uint8_t commandLength)
{
    uint16_t addr = toWord(addrH, addrL);
    uint16_t quantity = toWord(quantityH, quantityL);

    uint8_t desiredByteCount = quantity * 2;

    if (addr > AQSize - 1 || addr < 0) //check if illegal adress was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_WRITE_N_REGISTERS), MODBUS_ERR_ILLEGAL_ADDR};
        sendData(tab, 3);
        return;
    }
    else if (addr + quantity - 1 > AQSize - 1 || quantity < 1 || desiredByteCount != byteCount || commandLength - MODBUS_BYTE_COUNT - 1 != byteCount) //check if illegal number of input registers was selected
    {
        uint8_t tab[3] = {id, toError(MODBUS_WRITE_N_REGISTERS), MODBUS_ERR_ILLEGAL_DATA};
        sendData(tab, 3);
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
    sendData(tab, 6);
    return;
}

void MSlave::init(uint8_t id_, HardwareSerial *S_, uint16_t *AQ_, int AQs, bool *DQ_, int DQs, uint16_t *AI_, int AIs, bool *DI_, int DIs)
{
    if (id_ == 0 || id_ > 247)
        return;

    S = S_;

    S->setTimeout(15); //too long?

    id = id_;

    AQ = AQ_;
    DQ = DQ_;
    AI = AI_;
    DI = DI_;
    AQSize = AQs;
    DQSize = DQs;
    AISize = AIs;
    DISize = DIs;

    for (auto i = 0; i < AQSize; i++)
        AQ[i] = 0;
    for (auto i = 0; i < DQSize; i++)
        DQ[i] = 0;
    for (auto i = 0; i < AISize; i++)
        AI[i] = 0;
    for (auto i = 0; i < DISize; i++)
        DI[i] = 0;

    active = 1;
}

void MSlave::event()
{
    uint8_t command[MODBUS_MAX_FRAME_SIZE]; //should be bigger probably (The maximum size of a MODBUS RTU frame is 256 bytes. - MODBUS over Serial Line  Specification and Implementation Guide  V1.02)
    uint8_t commandLength = S->readBytes(command, MODBUS_MAX_FRAME_SIZE);

    if (commandLength < 2 + MODBUS_CRC_BYTE_COUNT || !active) //id and function code is necessary
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
        sendData(tab, 3);
        return;
    }

    switch (command[MODBUS_FUNCTION])
    {
    case MODBUS_READ_OUTPUTS:
        readOutputs(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_QUANTITY_HIGH], command[MODBUS_QUANTITY_LOW]);
        break;
    case MODBUS_READ_INPUTS:
        readInputs(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_QUANTITY_HIGH], command[MODBUS_QUANTITY_LOW]);
        break;
    case MODBUS_READ_OUTPUT_REGISTERS:
        readOutputRegisters(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_QUANTITY_HIGH], command[MODBUS_QUANTITY_LOW]);
        break;
    case MODBUS_READ_INPUT_REGISTERS:
        readInputRegisters(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_QUANTITY_HIGH], command[MODBUS_QUANTITY_LOW]);
        break;
    case MODBUS_WRITE_OUTPUT:
        writeOutput(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_VALUE_HIGH], command[MODBUS_VALUE_LOW]);
        break;
    case MODBUS_WRITE_REGISTER:
        writeRegister(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_VALUE_HIGH], command[MODBUS_VALUE_LOW]);
        break;
    case MODBUS_WRITE_N_OUTPUTS:
        writeNOutputs(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_QUANTITY_HIGH], command[MODBUS_QUANTITY_LOW], command[MODBUS_BYTE_COUNT], command, commandLength);
        break;
    case MODBUS_WRITE_N_REGISTERS:
        writeNRegisters(command[MODBUS_ID], command[MODBUS_ADDR_HIGH], command[MODBUS_ADDR_LOW], command[MODBUS_QUANTITY_HIGH], command[MODBUS_QUANTITY_LOW], command[MODBUS_BYTE_COUNT], command, commandLength);
        break;
    default:
        uint8_t tab[3] = {command[MODBUS_ID], toError(command[MODBUS_FUNCTION]), MODBUS_ERR_ILLEGAL_FUNCTION};
        sendData(tab, 3);
        break;
    }

    S->flush();
    return;
}