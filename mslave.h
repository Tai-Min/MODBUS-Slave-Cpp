#ifndef MSLAVE_H
#define MSLAVE_H

#include <Arduino.h>
#include "mdefines.h"

class MSlave
{
  private:
    bool crcDisabled = 0;
    uint8_t id = 255;

    uint16_t *AQ = nullptr;//analog out
    bool *DQ = nullptr;//digital out
    uint16_t *AI = nullptr;//analog in
    bool *DI = nullptr;//digital in

    int AQSize = 0;
    int DQSize = 0;
    int AISize = 0;
    int DISize = 0;

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
    void setDigitalOut(bool *DQ_, int DQsize_);
    void setDigitalIn(bool *DI_, int DIsize_);
    void setAnalogOut(uint16_t *AQ_, int AQsize_);
    void setAnalogIn(uint16_t *AI_, int AIsize_);
    void event();
};

#endif