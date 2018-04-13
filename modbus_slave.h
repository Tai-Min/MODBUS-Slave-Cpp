#ifndef SLAVE_H
#define SLAVE_H

#include <Arduino.h>
#include "modbus_defines.h"

class MODBUS_Slave
{
private:
    static int id;
    static bool active;

    static uint8_t toError(uint8_t code);
    static uint16_t toWord(uint8_t H, uint8_t L);
    static uint8_t toHighByte(uint16_t word);
    static uint8_t toLowByte(uint16_t word);

    static uint16_t crc(uint8_t command[], uint8_t commandLength);
    static void sendData(uint8_t tab[], uint8_t length);
    
    static void readOutputs(uint8_t id, uint8_t addrH,uint8_t addrL, uint8_t quantityH,uint8_t quantityL);
    static void readInputs(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL);
    static void readOutputRegisters(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL);
    static void readInputRegisters(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL);
    static void writeOutput(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t valH, uint8_t valL);
    static void writeRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t valH, uint8_t valL);
    static void writeNOutputs(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL, uint8_t byteCount, uint8_t command[], uint8_t commandLength);
    static void writeNRegisters(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL, uint8_t byteCount, uint8_t command[], uint8_t commandLength);
    
public:
    static void init(int id_);
    static void event();

    static uint16_t AQ[MODBUS_AQ_LENGTH];
    static bool DQ[MODBUS_DQ_LENGTH];
    static uint16_t AI[MODBUS_AI_LENGTH];
    static bool DI[MODBUS_DI_LENGTH];
};

#endif