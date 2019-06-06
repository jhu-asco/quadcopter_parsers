#include <gtest/gtest.h>
#include <tf_parser/tf_parser.h>

using namespace tf_parser;

TEST(TFParserTests, BaseCtor) {
  ASSERT_NO_THROW(TFParser());
}

TEST(TFParserTests, Initializer) {
    TFParser quad_simulator_instance;
    ASSERT_NO_THROW(quad_simulator_instance.initialize());
}

TEST(TFParserTests, Takeoff) {
    TFParser quad_simulator_instance;
    quad_simulator_instance.takeoff();
    //Get Data and ensure we are in right mode
    parsernode::common::quaddata data;
    quad_simulator_instance.getquaddata(data);
    ASSERT_EQ(data.localpos.z, 0.5);
    ASSERT_STREQ(data.quadstate.c_str(), "ARMED ENABLE_CONTROL ");
}

TEST(TFParserTests, Land) {
    TFParser quad_simulator_instance;
    quad_simulator_instance.takeoff();
    quad_simulator_instance.land();
    //Get Data and ensure we are in right mode
    parsernode::common::quaddata data;
    quad_simulator_instance.getquaddata(data);
    ASSERT_EQ(data.localpos.x, 0.0);
    ASSERT_EQ(data.localpos.y, 0.0);
    ASSERT_EQ(data.localpos.z, 0.0);
    ASSERT_STREQ(data.quadstate.c_str(), "ENABLE_CONTROL ");
}

TEST(TFParserTests, Disarm) {
    TFParser quad_simulator_instance;
    quad_simulator_instance.takeoff();
    quad_simulator_instance.disarm();
    //Get Data and ensure we are in right mode
    parsernode::common::quaddata data;
    quad_simulator_instance.getquaddata(data);
    ASSERT_EQ(data.localpos.x, 0.0);
    ASSERT_EQ(data.localpos.y, 0.0);
    ASSERT_EQ(data.localpos.z, 0.0);
    ASSERT_STREQ(data.quadstate.c_str(), "");
}

TEST(TFParserTests, DisableFlowControl) {
    TFParser quad_simulator_instance;
    quad_simulator_instance.takeoff();
    quad_simulator_instance.flowControl(false);
    //Get Data and ensure we are in right mode
    parsernode::common::quaddata data;
    quad_simulator_instance.getquaddata(data);
    ASSERT_EQ(data.localpos.z, 0.5);
    ASSERT_STREQ(data.quadstate.c_str(), "ARMED ");
}

TEST(TFParserTests, ImuCalibrate) {
    TFParser quad_simulator_instance;
    ASSERT_THROW(quad_simulator_instance.calibrateimubias(), std::runtime_error);
}

TEST(TFParserTests, TestBatteryPercent) {
  TFParser quad_simulator_instance;
  parsernode::common::quaddata data;
  quad_simulator_instance.getquaddata(data);
  ASSERT_EQ(data.batterypercent, 100);
  quad_simulator_instance.setBatteryPercent(20);
  quad_simulator_instance.getquaddata(data);
  ASSERT_EQ(data.batterypercent, 20);
}
// Add a func to get rpytcmd queue and check the cmdrpyt is doing the right thing

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
