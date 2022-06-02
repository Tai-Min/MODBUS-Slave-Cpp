# MODBUS-Slave-Cpp

This library turns devices that support HardwareSerial directly (Arduino) or via HAL into MODBUS server device capable of responding to eight basic RTU frames:
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
*CRC can be disabled.<br />
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
+ **S:** HardwareSerial object <br />
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
```cpp
bool digitalRead(bool type, uint16_t address);
uint16_t analogRead(bool type, uint16_t address); 
```
+ **type:** 
  - DISCRETE_INPUT (digital) / INPUT_REG (analog)
  - COIL (digital) / HOLDING_REG (analog) <br />
+ **address:** Position in specified array <br />

+ **returns:** Value stored under given address
<br />

### Write to digital/analog input/output array:
```cpp
void digitalWrite(bool type, uint16_t address, bool value);
void analogWrite(bool type, uint16_t address, uint16_t value);
```
+ **type:** 
  - DISCRETE_INPUT (digital) / INPUT_REG (analog)
  - COIL (digital) / HOLDING_REG (analog) <br />
+ **address:** Position in specified array <br />
+ **value:** Value to be written<br />
<br />

### Set server busy or idle:
```cpp
void setBusy();
void setIdle();
```
Server is idle by default.<br />

When server is busy then it will ignore any incoming data and will respond with MODBUS_ERR_SLAVE_BUSY exception.<br />
<br />


### Enable or disable CRC in request/response/exception frames:
```cpp
void disableCRC();
void enableCRC();
```
CRC is enabled by default.<br />
<br />

### Change standard of serial communication
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

## Example for Arduino
```cpp
#include "MSlave.h"

int ledPin = 13;
int potPin = A0;
int pwmLedPin = 11;
int buttonPin = 3;

int deviceID = 1;

// 1 digital input (address 0)
// 1 digital output (address 0)
// 1 analog input (address 0)
// 2 analog outputs (adresses 0,1)
// addresses works just like arrays
// so if you have 2 analog outputs, you can access adresses from 0 to 1
// if you have 40 analog inputs, you can access adresses from 0 to 39 etc
MSlave<1, 1, 1, 2> server; // Initialize slave device.

void setup()
{
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(potPin, INPUT);
  pinMode(pwmLedPin, OUTPUT);
  server.disableCRC(); // No need for crc check in this example.
  Serial.begin(115200);
  Serial.setTimeout(15);
  server.begin(deviceID, Serial); // Start modbus server.
}

void loop()
{
  if (server.available()) // Check whether master sent some serial data.
  {
    int result = server.read(); // Process data from master and return code of the processed function or 0 when there was no data / error occured / invalid request happened.
    digitalWrite(ledPin, server.digitalRead(COIL, 0)); // Read digital inputs array data received from client devices.
    analogWrite(pwmLedPin, server.analogRead(HOLDING_REG, 0)); // Read analog inputs array data received from client devices.
    server.digitalWrite(DISCRETE_INPUT, 0, digitalRead(buttonPin)); // Write button's state to digital outputs array so it will be available to be read from clients.
    server.analogWrite(INPUT_REG, 0, analogRead(potPin)); // Write potentiometer's state to analog outputs array so it will be available to be read from clients.

    if (server.digitalRead(DISCRETE_INPUT, 0)) // Read buttons's state from digital outputs array.
      server.analogWrite(INPUT_REG, 1, 512); // Write 512 to analog outputs array so it will be available to be read from clients.
    else
      server.analogWrite(INPUT_REG, 1, 100); // Write 100 to analog outputs array so it will be available to be read from clients.
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

## Non Arduino devices
This was not testes as of yet.

For other devices that don't support Arduino.h library, a pure abstract HardwareSerial class must be inherited by custom Serial implementation.

```cpp
class HardwareSerial
{
public:
    /**
     * @brief Write single byte to serial device.
     *
     * @param byte Byte to write.
     */
    virtual void write(uint8_t byte) = 0;

    /**
     * @brief Check serial port for unread data.
     *
     * @return Number of bytes available to read.
     */
    virtual int available() = 0;

    /**
     * @brief Read bytes.
     *
     * @param buffer Buffer to place bytes in.
     * @param length Size of the buffer.
     * @return Number of bytes placed in the buffer.
     */
    virtual size_t readBytes(uint8_t *buffer, size_t length) = 0;

    /**
     * @brief Wait for tx tramsmission to complete.
     */
    virtual void flush() = 0;
};
```

## About
The library was written using following documents and sites:
+ MODBUS over Serial Line Specification and Implementation Guide V1.02
+ Modbus Protocol Reference Guide
+ http://www.simplymodbus.ca/