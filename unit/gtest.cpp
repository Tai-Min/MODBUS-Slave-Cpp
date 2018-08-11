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

TEST(MSlave, illegalFunction)
{
    //write not supported function
    //and some data
    s.write({1, 13, 0, 0, 0, 10});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 13 + 0x80, 1}), s.readBytes());
}

TEST(MSlaveRead, validFrames)
{
    //read status of 10 coils starting from address 0
    s.write({1, 1, 0, 0, 0, 10});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 1, 2, 0, 0}), s.readBytes());

    //read status of 5 inputs starting from address 0
    s.write({1, 2, 0, 0, 0, 5});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 2, 1, 10}), s.readBytes());

    //read status of 5 inputs starting from address 1
    s.write({1, 2, 0, 1, 0, 5});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 2, 1, 21}), s.readBytes());

    //read status of 20 holding resisters starting from address 0
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

TEST(MSlaveRead, addressError)
{
    //read status of 1 coil starting from address 11
    //max addr is 9
    s.write({1, 1, 0, 11, 0, 1});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 1 + 0x80, 2}), s.readBytes());

    //read status of 5 inputs starting from address 223
    //max addr is 14
    s.write({1, 2, 0, 223, 0, 5});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 2 + 0x80, 2}), s.readBytes());

    //read status of 20 holding resisters starting from address 25
    //max addr is 19
    s.write({1, 3, 0, 25, 0, 20});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 3 + 0x80, 2}),
              s.readBytes());

    //read status of 2 input resisters starting from address 25
    //max addr is 24
    s.write({1, 4, 0, 25, 0, 2});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 4 + 0x80, 2}),
              s.readBytes());
}

TEST(MSlaveRead, valueError)
{
    //read status of 2 coils starting from address 9
    //only 1 coil is in valid address range
    s.write({1, 1, 0, 9, 0, 3});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 1 + 0x80, 3}), s.readBytes());

    //read status of 5 inputs starting from address 14
    //only 1 input is in valid address range
    s.write({1, 2, 0, 14, 0, 5});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 2 + 0x80, 3}), s.readBytes());

    //read status of 20 holding resisters starting from address 18
    //good number of registers
    //but most of them are out of valid address range
    s.write({1, 3, 0, 18, 0, 20});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 3 + 0x80, 3}),
              s.readBytes());

    //read status of 200 input resisters starting from address 20
    //too many input registers
    //and only 5 are in valid address range
    s.write({1, 4, 0, 20, 0, 200});
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
    //force multiple coils
    s.write({1, 0x0f, 0, 0, 0, 8, 1, 255});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x0f, 0, 0, 0, 8}), s.readBytes());

    //check whether coils were forced
    s.write({1, 1, 0, 0, 0, 8});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 1, 1, 255}), s.readBytes());
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

TEST(MSlaveWrite, addressError)
{
    //force single coil
    //on address 10
    //max is 9
    s.write({1, 5, 0, 10, 255, 0});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 5 + 0x80, 2}), s.readBytes());

    //preset single register
    //on address 20
    //max is 19
    s.write({1, 6, 0, 20, 255, 20});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 6 + 0x80, 2}), s.readBytes());

    //force multiple coils
    //from address 10
    //max is 9
    s.write({1, 0x0f, 0, 10, 0, 8, 1, 255});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x0f + 0x80, 2}), s.readBytes());

    //preset multiple registers
    //from address 23
    //max is 19
    s.write({1, 0x10, 0, 23, 0, 3, 6, 1, 255, 8, 32, 16, 35});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x10 + 0x80, 2}), s.readBytes());
}

TEST(MSlaveWrite, forceSingleValueError)
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

//preset single register shouldn't be possible

TEST(MSlaveWrite, forceMultipleValueError)
{
    //clear multiple coils
    //from address 8
    s.write({1, 0x0f, 0, 8, 0, 2, 1, 0});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x0f, 0, 8, 0, 2}), s.readBytes());

    //force multiple coils
    //from address 8
    s.write({1, 0x0f, 0, 8, 0, 8, 1, 255});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x0f + 0x80, 3}), s.readBytes());

    //check whether coils are still cleared
    //check whether coil is still cleared
    s.write({1, 1, 0, 8, 0, 2});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 1, 1, 0}), s.readBytes());

    ////////////////////////////////////////////////////////////////////////////////////

    //force multiple coils
    //from address 8
    s.write({1, 0x0f, 0, 8, 0, 2, 1, 3});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x0f, 0, 8, 0, 2}), s.readBytes());

    //force multiple coils
    //from address 8
    s.write({1, 0x0f, 0, 8, 0, 8, 1, 255});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x0f + 0x80, 3}), s.readBytes());

    //check whether coils are still forced
    //check whether coil is still forced
    s.write({1, 1, 0, 8, 0, 2});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 1, 1, 3}), s.readBytes());
}

TEST(MSlaveWrite, presetMultipleValueError)
{
    //preset single register
    //on address 10
    s.write({1, 6, 0, 19, 8, 11});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 6, 0, 19, 8, 11}), s.readBytes());

    //preset multiple registers
    //from address 19
    s.write({1, 0x10, 0, 19, 0, 3, 6, 1, 255, 8, 32, 16, 35});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 0x10 + 0x80, 3}), s.readBytes());

    //check whether register is preset
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