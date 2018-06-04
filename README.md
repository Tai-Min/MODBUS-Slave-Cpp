# MODBUS-Slave-for-Arduino

This library turns Arduino board into MODBUS server device capable of responding to eight basic RTU frames:
+ Read coil status (0x01)
+ Read input status (0x02)
+ Read holding register(0x03)
+ Read input register (0x04)
+ Force single coil (0x05)
+ Preset single register (0x06)
+ Force multiple coils (0x0F)
+ Preset multiple registers (0x10)

The library expects full RTU frames consisting of:
+ device ID (1 byte)
+ function ID (1 byte)
+ data (n bytes)
+ crc (2 bytes) (optional)

Also, the library is able to detect invalid frame and respond to it with adequate exception frame.

## Usage

## Example
```cpp
#include "MSlave.h"

int ledPin = 13;
int potPin = A0;
int pwmLedPin = 11;
int buttonPin = 3;

int deviceID = 1;

//1 digital input
//1 digital output
//1 analog input
//2 analog outputs
//addresses works just like arrays
//so if you have 2 analog outputs, you can access adresses from 0 to 1
MSlave<1, 1, 1, 2> server(deviceID, &Serial);//initialize slave device

void setup()
{
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(potPin, INPUT);
  pinMode(pwmLedPin, OUTPUT);
  server.disableCRC();//no need for crc check in this example
  Serial.begin(115200);
  Serial.setTimeout(15);
}

void loop()
{
  if (server.available())//check whether some data exchange happened with this device
  {
    digitalWrite(ledPin, server.digitalRead(0, INPUT));//digitalRead external boolean data received from client devices
    analogWrite(pwmLedPin, server.analogRead(0, INPUT));//analogRead external uint16_t data received from client devices
    server.digitalWrite(0, digitalRead(buttonPin));//digitalWrite button's state to server object so it will be available to be read from clients
    server.analogWrite(0, analogRead(potPin));//analogWrite potentiometer's state to server object so it will be available to be read from clients
    if(server.digitalRead(0, OUTPUT))//read buttons's state from server object
    {
      server.analogWrite(1, 512);//analogWrite some value server object so it will be available to be read from clients
    }
  }
}
```

## About
The library was written using following documents and sites:
+ MODBUS over Serial Line Specification and Implementation Guide V1.02
+ Modbus Protocol Reference Guide
+ http://www.simplymodbus.ca/
