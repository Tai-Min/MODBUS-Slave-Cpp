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

Also, the library is able to detect invalid frame and respond to it with adequate exception frame.

## Usage
This library is really straightforward to use: <br />
Firstly, you need to create MSlave object with an id and a HardwareSerial address passed to it. i.e:
```
MSlave s(1, &Serial); //where 1 is a device ID and &Serial is an addres to standard Arduino Serial
```
Then you need to pass your boolean and uint16_t arrays and their respective sizes to object by using:
```
s.setDigitalOut(arr1, size1); //bool - this array is read/write for the client
s.setDigitalIn(arr2, size2); //bool - this array is read only for the client
s.setAnalogOut(arr3, size3); //uint16_t - this array is read/write for the client
s.setAnalogIn(arr4, size4); //uint16_t - this array is read only for the client
```
if you want, you can disable or enable CRC check by using:
```
s.disableCRC();
s.enableCRC();
```
CRC is enabled by default. <br /> <br />
The last step is to invoke 
```
s.event();
```
as often as you can in program's loop. <br />

## Example
Using the code shown below you can turn on led on pin 13 by sending to Arduino "Force single coil" frame: <br />
| 1 | 5 | 0 0 | 255 0 | where: <br />
+ 1 is the device ID
+ 5 is "Force single coil" function id
+ 0 0 are two bytes of address to our coil which is 0
+ 255 0 means set coil to 1

or you can turn off led 13 by using the same command with last part of the frame replaced with 0 0: <br />
| 1 | 5 | 0 0 | 0 0 |

Reading status of led 13: <br />
| 1 | 2 | 0 6 | 0 1 | where: <br />
+ 1 is the device ID
+ 2 is "Read input status" function id
+ 0 6 are two bytes of address to our coil which is 6
+ 0 1 are two bytes of quantity of inputs to be read which is 1
```
#include "mslave.h"

int ledPin = 13;
int potPin = A0;
int analogLed = 11;

int deviceID = 1;
const int AnalogOutSize = 10;//randomly chosen array sizes just for this example 
const int AnalogInSize = 8;
const int DigitalOutSize = 5;
const int DigitalInSize = 12;

uint16_t AnalogOut[AnalogOutSize]; //holding registers
uint16_t AnalogIn[AnalogInSize]; //input registers
bool DigitalOut[DigitalOutSize]; //coils
bool DigitalIn[DigitalInSize]; //inputs

MSlave slave(deviceID, &Serial);

void setup()
{
  pinMode(ledPin,OUTPUT);
  pinMode(potPin, INPUT);
  pinMode(analogLed, OUTPUT);
  
  slave.setDigitalOut(DigitalOut, DigitalOutSize);//this array is read/write for the client
  slave.setDigitalIn(DigitalIn, DigitalInSize);//this array is read only for the client
  slave.setAnalogOut(AnalogOut, AnalogOutSize);//this array is read/write for the client
  slave.setAnalogIn(AnalogIn, AnalogInSize);//this array is read only for the client
  
  //serial with 8 bits of data, even parity and one stop bit
  //"Even parity is required, other modes ( odd parity, no parity ) may also be used. In order to ensure a maximum compatibility with
  //other products, it is recommended to support also No parity mode. The default parity mode must be even parity.
  //Remark : the use of no parity requires 2 stop bits."
  //source: MODBUS over Serial Line Specification and Implementation Guide V1.02
  Serial.begin(115200, SERIAL_8E1);//you don't need to set it like this but make sure both client and server use equally configured serial
}

void loop()
{
    slave.event();
    digitalWrite(ledPin, DigitalOut[0]);
    analogWrite(analogLed, AnalogOut[5]);
    DigitalIn[6] = digitalRead(ledPin);
    AnalogIn[3] = analogRead(potPin);
}
```

The library was written using following documents and sites:
+ MODBUS over Serial Line Specification and Implementation Guide V1.02
+ Modbus Protocol Reference Guide
+ http://www.simplymodbus.ca/
