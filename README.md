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
This library is really straightforward to use: <br />
Firstly, you need to create MSlave object with memory cell sizes, an id and a HardwareSerial address passed to it. i.e:
```cpp
//<5> - number of boolean inputs (outputs) - (read only for this, read/write for the client) (uint16_t)
//<12> - number of boolean outputs (inputs) - (read/write for this device, read only for the client) (uint16_t)
//<10> - number of uint16_t inputs (holding registers) - (read only for this, read/write for the client) (uint16_t)
//<8> - number of uint16_t outputs (input registers) - (read/write for this device, read only for the client) (uint16_t)
//1 is the unique device ID (uint8_t)
//&Serial is an addres to Arduino's Serial object (&HardwareSerial)
MSlave<5,12,10,8> slave(1, &Serial);
```
then you need to invoke 
```cpp
s.event(); //(bool) expects nothing;
```
as often as you can in program's loop. <br />
event() returns true only when valid read/write operation has happened.
And that's all. <br /> <br/>
You can read input/outputs from MSlave object by using:
```cpp
s.digitalRead(address, INPUT); //(bool) expects (uint16_t, bool)
s.digitalRead(address, OUTPUT); //(bool) expects (uint16_t, bool)
s.analogRead(address, INPUT); //(uint16_t) expects (uint16_t, bool)
s.analogRead(address, OUTPUT); //(uint16_t) expects (uint16_t, bool)
```
or write to outputs:
```cpp
s.digitalWrite(address, value); //(void) expects (uint16_t, bool)
s.analogWrite(address, value); //(void) expects (uint16_t, uint16_t)
```

### Additional functions: <br />
if you want, you can disable or enable CRC check in request/response/exception frames by using:
```cpp
s.disableCRC(); //(void) expects nothing;
s.enableCRC(); //(void) expects nothing;
```
CRC is enabled by default. <br />

## Example
Using the code shown below you can turn on led on pin on 13 by sending to Arduino "Force single coil" frame: <br />
| 1 | 5 | 0 0 | 255 0 | where: <br />
+ 1 is the device ID
+ 5 is "Force single coil" function id
+ 0 0 are two bytes of address to our coil which is 0
+ 255 0 means set coil to 1

or you can turn off led on 13 by using the same command with last part of the frame replaced with 0 0: <br />
| 1 | 5 | 0 0 | 0 0 |

Reading status of button on pin 3: <br />
| 1 | 2 | 0 6 | 0 1 | where: <br />
+ 1 is the device ID
+ 2 is "Read input status" function id
+ 0 6 are two bytes of address to our coil which is 6
+ 0 1 are two bytes of quantity of inputs to be read which is 1

```cpp
#include "MSlave.h"

int ledPin = 13;
int potPin = A0;
int analogLed = 11;
int buttonPin = 3;

int deviceID = 1;

//<5> - coils boolean values (read only for this, read/write for the client)
//<12> - inputs boolean values (read/write for this device, read only for the client)
//<10> - holding registers uint16_t values (read only for this, read/write for the client)
//<8> - input registers uint16_t values (read/write for this device, read only for the client)
MSlave<5, 12, 10, 8> slave(deviceID, &Serial);

void setup()
{
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(potPin, INPUT);
  pinMode(analogLed, OUTPUT);
  slave.disableCRC();//no need for crc in this example

  //serial with 8 bits of data, even parity and one stop bit
  //"Even parity is required, other modes ( odd parity, no parity ) may also be used. In order to ensure a maximum compatibility with
  //other products, it is recommended to support also No parity mode. The default parity mode must be even parity.
  //Remark : the use of no parity requires 2 stop bits."
  //source: MODBUS over Serial Line Specification and Implementation Guide V1.02
  //you don't need to set it like this but make sure both client and server use equally configured serial
  Serial.begin(115200, SERIAL_8E1);
}

void loop()
{
  if (slave.event())
  {
    //read INPUT because you need a value that come from the outside of the device
    //if you want to read value that has been written to input by the device, use OUTPUT as second argument
    //same for analogRead
    digitalWrite(ledPin, slave.digitalRead(0, INPUT));
    analogWrite(analogLed, slave.analogRead(5, INPUT));

    //you can write only to inputs
    slave.digitalWrite(6, digitalRead(buttonPin));
    slave.analogWrite(3, analogRead(potPin));
  }
}
```

The library was written using following documents and sites:
+ MODBUS over Serial Line Specification and Implementation Guide V1.02
+ Modbus Protocol Reference Guide
+ http://www.simplymodbus.ca/
