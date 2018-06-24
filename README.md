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
+ crc (2 bytes)

Also, the library is able to detect invalid request frame and respond to it with an adequate exception frame.

## Usage
#### MSlave variable initializer:
```cpp
template <uint16_t dil, uint16_t dol, uint16_t ail, uint16_t aol>
MSlave(uint8_t id, HardwareSerial *serial);
```
**dil** - length of digital inputs array (MODBUS coils) - read only for the server, read/write for the client<br /> 
**dol** - length of digital outputs array (MODBUS inputs) - read/write for the server, read only for the client<br />
**ail** - length of analog inputs array (MODBUS holding registers) - read only for the server, read/write for the client<br />
**aol** - length of analog outputs array (MODBUS input registers) - read/write for the server, read only for the client<br />
**id** - unique id of the server <br />
**serial** - address to Arduino's HardwareSerial object 
<br />

#### Enable or disable CRC in request/response/exception frames:
```cpp
void disableCRC();
void enableCRC();
```
CRC is enabled by default.
<br />

#### Check if there is a request frame in serial buffer and parse it:
```cpp
bool available();
```
returns true only if one of eight rtu frames has been successfully processed <br />
This function should be used as often as possible to provide responsive and dependable server.
<br />

#### Read from digital/analog input/output array:
```cpp
bool digitalRead(uint16_t address, bool mode);
uint16_t analogRead(uint16_t address, bool mode); 
```
**address** - position in specified array <br />
**mode:** 
+ INPUT - things sent from client devices
+ OUTPUT - things written by using analogWrite function

#### Write to digital/analog output array:
```cpp
void digitalWrite(uint16_t address, bool value);
void analogWrite(uint16_t address, uint16_t value);
```
**address** - position in specified array <br />
**value** - value to be written
<br />

## Example
```cpp
#include "MSlave.h"

int ledPin = 13;
int potPin = A0;
int pwmLedPin = 11;
int buttonPin = 3;

int deviceID = 1;

//1 digital input (address 0)
//1 digital output (address 0)
//1 analog input (address 0)
//2 analog outputs (adresses 0,1)
//addresses works just like arrays
//so if you have 2 analog outputs, you can access adresses from 0 to 1
//if you have 40 analog inputs, you can access adresses from 0 to 39 etc
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
  if (server.available())//check whether some successfull data exchange happened with this device
  {
    digitalWrite(ledPin, server.digitalRead(0, INPUT));//digitalRead digital inputs array data received from client devices
    analogWrite(pwmLedPin, server.analogRead(0, INPUT));//analogRead analog inputs array data received from client devices
    server.digitalWrite(0, digitalRead(buttonPin));//digitalWrite button's state to digital outputs array so it will be available to be read from clients
    server.analogWrite(0, analogRead(potPin));//analogWrite potentiometer's state to analog outputs array so it will be available to be read from clients
    if(server.digitalRead(0, OUTPUT))//read buttons's state from digital outputs array
    {
      server.analogWrite(1, 512);//analogWrite 512 to analog outputs array so it will be available to be read from clients
    }
  }
}
```
#### frame to turn on led on pin 13:
|id  |function|address|value  |
|:--:|:------:|:-----:|:-----:|
|1   |5       |0 0    |255 0  |
#### frame to change light intensity of pwm led on pin 11:
|id  |function|address|value  |
|:--:|:------:|:-----:|:-----:|
| 1  | 6      | 0 0   | 0 128 |
#### frame to read state of the button on pin 3:
|id  |function|address of the first input|quantity of inputs to read  |
|:--:|:------:|:-----:|:---------------------------------------------:|
| 1  | 2      | 0 0   | 0 1                                           |
#### frame to read state of the potentiometer on pin A0:
|id  |function|address of the first input|quantity of inputs to read  |
|:--:|:------:|:-----:|:---------------------------------------------:|
| 1  | 4      | 0 0   | 0 1                                           |
#### frame to read state of led on pin 13:
|id  |function|address of the first output|quantity of outputs to read |
|:--:|:------:|:-----:|:----------------------------------------------:|
| 1 | 1 | 0 0 | 0 1                                                    |
#### frame to read states of both potentiometer and value on address 1:
|id  |function|address of the first input|quantity of inputs to read  |
|:--:|:------:|:-----:|:---------------------------------------------:|
| 1 | 4 | 0 0 | 0 2                                                   |
## About
The library was written using following documents and sites:
+ MODBUS over Serial Line Specification and Implementation Guide V1.02
+ Modbus Protocol Reference Guide
+ http://www.simplymodbus.ca/
