#include <thread>
#include <gtest/gtest.h>
#include "./Boost-Serial-Port/BoostSerial.h"

/*
Arduino code
#include "MSlave.h"

MSlave<10, 15, 20, 25> server;

void setup()
{
  server.disableCRC();
  Serial.begin(9600);
  Serial.setTimeout(15);
  server.begin(1, Serial);
  for(int i = 0; i < 15; i++)
  {
    if(i%2)
      server.digitalWrite(i, true);
  }

  for(int i = 0; i < 25; i++)
  {
    if(i%2)
      server.analogWrite(i,1337);
  }
}

void loop()
{
  server.read();
}
*/

BoostSerial s;

//should happen when slave receives unknown function code
TEST(MSlave, illegalFunction)
{
    //write not supported function code
    //with some meaningless data
    s.write({1, 13, 0, 0, 0, 10});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 13 + 0x80, 1}), s.readBytes());
}

TEST(MSlaveRead, validFrames)
{
    //read status of 10 coils starting from address 0
    //there are 10 coils so reading 0-9 is okay
    s.write({1, 1, 0, 0, 0, 10});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 1, 2, 0, 0}), s.readBytes());

    //read status of 5 inputs starting from address 0
    //there are 15 inputs so reading 0-4 is okay
    s.write({1, 2, 0, 0, 0, 5});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 2, 1, 10}), s.readBytes());

    //read status of 5 inputs starting from address 1
    //there are 15 inputs so reading 1-6 is okay
    s.write({1, 2, 0, 1, 0, 5});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 2, 1, 21}), s.readBytes());

    //read status of 20 holding resisters starting from address 0
    //there are 20 holding registers so reading 0-19 is okay
    s.write({1, 3, 0, 0, 0, 20});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 3, 40,
                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0}),
              s.readBytes());

    //read status of 20 input resisters starting from address 0
    //there are 25 input registers so reading 0-19 is okay
    s.write({1, 4, 0, 0, 0, 20});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 4, 40,
                                    0, 0, 5, 57, 0, 0, 5, 57, 0, 0,
                                    5, 57, 0, 0, 5, 57, 0, 0, 5, 57,
                                    0, 0, 5, 57, 0, 0, 5, 57, 0, 0,
                                    5, 57, 0, 0, 5, 57, 0, 0, 5, 57}),
              s.readBytes());
}

TEST(MSlaveRead, invalidAddress)
{
    //read status of 1 coil starting from address 11
    //max addr is 9 so invalid address is selected
    s.write({1, 1, 0, 11, 0, 1});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 1 + 0x80, 2}), s.readBytes());

    //read status of 5 inputs starting from address 223
    //max addr is 14 so invalid address is selected
    s.write({1, 2, 0, 223, 0, 5});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 2 + 0x80, 2}), s.readBytes());

    //read status of 20 holding resisters starting from address 25
    //max addr is 19 so invalid address is selected
    s.write({1, 3, 0, 25, 0, 20});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 3 + 0x80, 2}),
              s.readBytes());

    //read status of 2 input resisters starting from address 25
    //max addr is 24 so invalid address is selected
    s.write({1, 4, 0, 25, 0, 2});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 4 + 0x80, 2}),
              s.readBytes());

    //read status of 3 coils starting from address 9
    //only 1 coil is in valid address range
    //so two coils have invalid addresses selected
    s.write({1, 1, 0, 9, 0, 3});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 1 + 0x80, 2}), s.readBytes());

    //read status of 5 inputs starting from address 14
    //only 1 input is in valid address range
    //so 4 inputs have invalid addresses selected
    s.write({1, 2, 0, 14, 0, 5});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 2 + 0x80, 2}), s.readBytes());

    //read status of 20 holding resisters starting from address 18
    //only 2 holding registers are in valid address range
    //so 18 holding registers have invalid addresses
    s.write({1, 3, 0, 18, 0, 20});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 3 + 0x80, 2}),
              s.readBytes());

    //read status of 200 input resisters starting from address 20
    //invalid number of input registers
    //and only 5 are in valid address range so 195 have invalid addresses
    s.write({1, 4, 0, 20, 0, 200});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 4 + 0x80, 2}),
              s.readBytes());
}

TEST(MSlaveRead, invalidData)
{
    //read status of 0 coils starting from address 9
    //reading state of 0 coils is pointless
    //so return invalid data error
    s.write({1, 1, 0, 9, 0, 0});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 1 + 0x80, 3}), s.readBytes());

    //read status of 0 inputs starting from address 0
    //reading state of 0 inputs is pointless
    //so return invalid data error
    s.write({1, 2, 0, 0, 0, 0});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 2 + 0x80, 3}), s.readBytes());

    //read status of 0 holding resisters starting from address 18
    //reading state of 0 holding resisters is pointless
    //so return invalid data error
    s.write({1, 3, 0, 18, 0, 0});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 3 + 0x80, 3}),
              s.readBytes());

    //read status of 0 input resisters starting from address 20
    //reading state of 0 input resisters is pointless
    //so return invalid data error
    s.write({1, 4, 0, 20, 0, 0});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 4 + 0x80, 3}),
              s.readBytes());
}

TEST(MSlaveWrite, forceSingleValid)
{
    //force single coil
    //on address 0
    s.write({1, 5, 0, 0, 255, 0});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 5, 0, 0, 255, 0}), s.readBytes());

    //check whether coil is forced
    s.write({1, 1, 0, 0, 0, 1});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 1, 1, 1}), s.readBytes());

    //clear single coil
    //on address 0
    s.write({1, 5, 0, 0, 0, 0});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 5, 0, 0, 0, 0}), s.readBytes());

    //check whether coil is cleared
    s.write({1, 1, 0, 0, 0, 1});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 1, 1, 0}), s.readBytes());
}

TEST(MSlaveWrite, presetSingleValid)
{
    //preset single register
    //on address 10
    s.write({1, 6, 0, 10, 255, 20});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 6, 0, 10, 255, 20}), s.readBytes());

    //check whether register is preset
    s.write({1, 3, 0, 10, 0, 1});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 3, 2, 255, 20}), s.readBytes());
}

TEST(MSlaveWrite, forceMultipleValid)
{
    //force multiple coils (0 - 7)
    s.write({1, 0x0f, 0, 0, 0, 8, 1, 125});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x0f, 0, 0, 0, 8}), s.readBytes());

    //check whether coils were forced
    s.write({1, 1, 0, 0, 0, 8});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 1, 1, 125}), s.readBytes());

    //force multiple coils (0 - 1)
    //quantity that is not divisible by 8
    //coils 2 - 7 shall remain untouched
    s.write({1, 0x0f, 0, 0, 0, 2, 1, 130});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x0f, 0, 0, 0, 2}), s.readBytes());

    //check whether coils 0-1 changed
    //read 8 coils to check whether coils 2-7 are untouched
    s.write({1, 1, 0, 0, 0, 8});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 1, 1, 126}), s.readBytes());
}

TEST(MSlaveWrite, presetMultipleValid)
{
    //preset multiple registers
    s.write({1, 0x10, 0, 0, 0, 3, 6, 1, 255, 8, 32, 16, 35});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x10, 0, 0, 0, 3}), s.readBytes());

    //check whether registers were preset
    s.write({1, 3, 0, 0, 0, 3});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 3, 6, 1, 255, 8, 32, 16, 35}), s.readBytes());
}

TEST(MSlaveWrite, invalidAddress)
{
    //force single coil
    //on address 10
    //max address is 9
    s.write({1, 5, 0, 10, 255, 0});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 5 + 0x80, 2}), s.readBytes());

    //preset single register
    //on address 20
    //max address is 19
    s.write({1, 6, 0, 20, 255, 20});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 6 + 0x80, 2}), s.readBytes());

    //force multiple coils
    //from address 10
    //max address is 9
    s.write({1, 0x0f, 0, 10, 0, 8, 1, 255});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x0f + 0x80, 2}), s.readBytes());

    //preset multiple registers
    //from address 23
    //max address is 19
    s.write({1, 0x10, 0, 23, 0, 3, 6, 1, 255, 8, 32, 16, 35});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x10 + 0x80, 2}), s.readBytes());
}

TEST(MSlaveWrite, forceSingleInvalidData)
{
    //clear single coil
    //on address 0
    s.write({1, 5, 0, 0, 0, 0});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 5, 0, 0, 0, 0}), s.readBytes());

    //force single coil
    //with strange value
    s.write({1, 5, 0, 0, 20, 0});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 5 + 0x80, 3}), s.readBytes());

    //check whether coil is still cleared
    s.write({1, 1, 0, 0, 0, 1});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 1, 1, 0}), s.readBytes());

    ////////////////////////////////////////////////////////////////////////////////////

    //force single coil
    //on address 0
    s.write({1, 5, 0, 0, 255, 0});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 5, 0, 0, 255, 0}), s.readBytes());

    //force single coil
    //with strange value
    s.write({1, 5, 0, 0, 20, 0});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 5 + 0x80, 3}), s.readBytes());

    //check whether coil is still forced
    s.write({1, 1, 0, 0, 0, 1});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 1, 1, 1}), s.readBytes());
}

TEST(MSlaveWrite, forceMultipleInvalidAddress)
{
    //clear 2 coils
    //from address 8
    s.write({1, 0x0f, 0, 8, 0, 2, 1, 0});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x0f, 0, 8, 0, 2}), s.readBytes());

    //force 8 coils
    //from address 8
    //only 2 coils are in valid address range
    s.write({1, 0x0f, 0, 8, 0, 8, 1, 255});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x0f + 0x80, 2}), s.readBytes());

    //check whether 2 coils are still cleared
    s.write({1, 1, 0, 8, 0, 2});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 1, 1, 0}), s.readBytes());

    ////////////////////////////////////////////////////////////////////////////////////

    //force 2 coils
    //from address 8
    s.write({1, 0x0f, 0, 8, 0, 2, 1, 3});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x0f, 0, 8, 0, 2}), s.readBytes());

    //force 8 coils
    //from address 8
    //only 2 coils are in valid address range
    s.write({1, 0x0f, 0, 8, 0, 8, 1, 255});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x0f + 0x80, 2}), s.readBytes());

    //check whether 2 coils are still forced
    s.write({1, 1, 0, 8, 0, 2});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 1, 1, 3}), s.readBytes());
}

TEST(MSlaveWrite, forceMultipleInvalidData)
{
    //force 2 coils
    //from address 8
    s.write({1, 0x0f, 0, 8, 0, 2, 1, 3});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x0f, 0, 8, 0, 2}), s.readBytes());

    //force 0 coils
    //from address 8
    s.write({1, 0x0f, 0, 8, 0, 0, 0});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x0f + 0x80, 3}), s.readBytes());

    //check whether 2 coils are still forced
    s.write({1, 1, 0, 8, 0, 2});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 1, 1, 3}), s.readBytes());
}

TEST(MSlaveWrite, presetMultipleInvalidAddress)
{
    //preset single register
    //on address 19
    s.write({1, 6, 0, 19, 8, 11});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 6, 0, 19, 8, 11}), s.readBytes());

    //preset 3 registers
    //from address 19
    //only 1 register is in valid address range
    s.write({1, 0x10, 0, 19, 0, 3, 6, 1, 255, 8, 32, 16, 35});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x10 + 0x80, 2}), s.readBytes());

    //check whether valid register is preset
    s.write({1, 3, 0, 19, 0, 1});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 3, 2, 8, 11}), s.readBytes());
}

TEST(MSlaveWrite, presetMultipleInvalidData)
{
    //preset single register
    //on address 19
    s.write({1, 6, 0, 19, 8, 11});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 6, 0, 19, 8, 11}), s.readBytes());

    //preset 0 registers
    //from address 19
    s.write({1, 0x10, 0, 19, 0, 0, 0});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x10 + 0x80, 3}), s.readBytes());

    //check whether valid register is preset
    s.write({1, 3, 0, 19, 0, 1});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 3, 2, 8, 11}), s.readBytes());
}

int main(int argc, char **argv)
{
    s.open("/dev/ttyUSB0");
    if (!s.isOpen())
        return 1;
    s.setBaud(9600);
    s.setTimeout(100);
    s.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    ::testing::InitGoogleTest(&argc, argv);
    int res = RUN_ALL_TESTS();

    s.close();

    return res;
}