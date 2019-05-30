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