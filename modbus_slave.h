#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <Arduino.h>
#include "modbus_defines.h"

class MSlave
{
  private:
    static uint8_t id;
    static bool active;

    static int AQSize;
    static int DQSize;
    static int AISize;
    static int DISize;

    static HardwareSerial *S;

    static uint8_t toError(uint8_t code);
    static uint16_t toWord(uint8_t H, uint8_t L);
    static uint8_t toHighByte(uint16_t word);
    static uint8_t toLowByte(uint16_t word);

    static uint16_t crc(uint8_t command[], uint8_t commandLength);
    static void sendData(uint8_t tab[], uint8_t length);

    static void readOutputs(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL);
    static void readInputs(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL);
    static void readOutputRegisters(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL);
    static void readInputRegisters(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL);
    static void writeOutput(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t valH, uint8_t valL);
    static void writeRegister(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t valH, uint8_t valL);
    static void writeNOutputs(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL, uint8_t byteCount, uint8_t command[], uint8_t commandLength);
    static void writeNRegisters(uint8_t id, uint8_t addrH, uint8_t addrL, uint8_t quantityH, uint8_t quantityL, uint8_t byteCount, uint8_t command[], uint8_t commandLength);

    MSlave(){};

  public:
    static void init(uint8_t id_, HardwareSerial *S_, uint16_t *AQ_, int AQs, bool *DQ_, int DQs, uint16_t *AI_, int AIs, bool *DI_, int DIs);
    static void event();

    static uint16_t *AQ;
    static bool *DQ;
    static uint16_t *AI;
    static bool *DI;
};

#endif