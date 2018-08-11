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
+ crc (2 bytes)*

Also, the library is able to detect invalid request frame and respond to it with an adequate exception frame.<br />
<br />
*CRC can be disabled <br />
<br />

## Usage
### MSlave variable initializer:
```cpp
template <uint16_t dis, uint16_t dos, uint16_t ais, uint16_t aos>
MSlave;
```
+ **dis:** Size of digital inputs array (MODBUS coils) - read only for the server, read/write for the client<br /> 
+ **dos:** Size of digital outputs array (MODBUS inputs) - read/write for the server, read only for the client<br />
+ **ais:** Size of analog inputs array (MODBUS holding registers) - read only for the server, read/write for the client<br />
+ **aos:** Size of analog outputs array (MODBUS input registers) - read/write for the server, read only for the client<br />
<br />

### Start MODBUS server
```cpp
void begin(uint8_t id, HardwareSerial S);
```
+ **id:** Unique id of the server <br />
+ **S:** Arduino's HardwareSerial object <br />
<br />

### Stop MODBUS server
```cpp
void end();
```
<br />

### Check whether there is data pending to be processed by the server
```cpp
bool available();
```
+ **returns:** True only when there is data pending to be processed by this server<br />
<br />

### Process pending data:
```cpp
uint8_t read();
```
+ **returns:** Function code of the processed frame or 0 when there was no data / error occured / invalid request happened
This function should be used as often as possible along with available() to provide responsive and dependable server.<br />
<br />

### Read from digital/analog input/output array:
#### 1. Arduino naming convention
```cpp
bool digitalRead(bool type, uint16_t address);
uint16_t analogRead(bool type, uint16_t address); 
```
+ **type:** 
  - INPUT - Input array / things sent from client devices
  - OUTPUT - Output array / things written by using analogWrite function<br />
+ **address:** Position in specified array <br />

+ **returns:** Value stored under given address
<br />

***
#### 2. MODBUS naming convention
```cpp
bool readCoil(uint16_t address); //same as digitalRead(INPUT, address);
bool readInput(uint16_t address);//same as digitalRead(OUTPUT, address);
uint16_t readHoldingRegister(uint16_t address);//same as analogRead(INPUT, address);
uint16_t readInputRegister(uint16_t address);//same as analogRead(OUTPUT, address);
```
+ **address:** Position in specified array <br />
<br />

### Write to digital/analog output array:
#### 1. Arduino naming convention
```cpp
void digitalWrite(uint16_t address, bool value);
void analogWrite(uint16_t address, uint16_t value);
```
+ **address:** Position in specified array <br />
+ **value:** Value to be written<br />
<br />

***
#### 2. MODBUS naming convention
```cpp
void writeInput(uint16_t address, bool value);//same as digitalWrite(uint16_t address, bool value);
void writeInputRegister(uint16_t address, uint16_t value);//same as analogWrite(uint16_t address, uint16_t value);
```
+ **address:** Position in specified array <br />
+ **value:** Value to be written<br />
<br />

### Enable or disable CRC in request/response/exception frames:
```cpp
void disableCRC();
void enableCRC();
```
CRC is enabled by default.<br />
<br />

### Change the standard of serial communication
```cpp
void useUART();
void useRS485(void (*actAsTransmitter)(bool) actAsTransmitter);
```
+ **actAsTransmitter:** Pointer to function that is used to control RS485 converter's direction.

UART is enabled by default.<br />
<br />
Due to variety of uart <-> RS485 converters you need to provide separate function that controls the direction of your converter<br />
This function should be a type of void and should expect one boolean parameter.<br />
When this parameter is true, your function should set the converter into transmitter. Otherwise, when this parameter is false, your function should set the converter into receiver. i.e:<br />

```cpp
void foo(bool t)
{
  digitalWrite(13,t); //when pin 13 is high, the converter act as transmitter
}

void setup()
{
  ...
  server.useRS485(foo);
  ...
}

```
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
MSlave<1, 1, 1, 2> server;//initialize slave device

void setup()
{
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(potPin, INPUT);
  pinMode(pwmLedPin, OUTPUT);
  server.disableCRC();//no need for crc check in this example
  Serial.begin(115200);
  Serial.setTimeout(15);
  server.begin(deviceID, Serial);//start modbus server
}

void loop()
{
  if (server.available())//check whether master sent some data
  {
    int result = server.read();//process data from master and return code of the processed function or 0 when there was no data / error occured / invalid request happened
    digitalWrite(ledPin, server.digitalRead(INPUT, 0));//digitalRead digital inputs array data received from client devices
    analogWrite(pwmLedPin, server.analogRead(INPUT, 0));//analogRead analog inputs array data received from client devices
    server.digitalWrite(0, digitalRead(buttonPin));//digitalWrite button's state to digital outputs array so it will be available to be read from clients
    server.analogWrite(0, analogRead(potPin));//analogWrite potentiometer's state to analog outputs array so it will be available to be read from clients
    if(server.digitalRead(OUTPUT, 0))//read buttons's state from digital outputs array
    {
      server.analogWrite(1, 512);//analogWrite 512 to analog outputs array so it will be available to be read from clients
    }
  }
}
```
### Request frames for this example:
Note: values in frames below are raw bytes, not ascii characters so you should use functions to read/write binary data, not characters. i.e if you send your request frames from other Arduino board, you should use Serial.write() instead of Serial.print()

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
