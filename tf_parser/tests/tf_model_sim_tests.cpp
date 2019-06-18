#include <gtest/gtest.h>
#include <tf_parser/tf_model_sim.h>

using namespace tf_parser;

TEST(TFModelSimTests, BaseCtor) {
  ASSERT_NO_THROW(TFModelSim());
}

TEST(TFModelSimTests, Initializer) {
    TFModelSim quad_simulator_instance;
    ASSERT_NO_THROW(quad_simulator_instance.initialize());
}

TEST(TFModelSimTests, Takeoff) {
    TFModelSim quad_simulator_instance;
    quad_simulator_instance.takeoff();
    //Get Data and ensure we are in right mode
    parsernode::common::quaddata data;
    quad_simulator_instance.getquaddata(data);
    ASSERT_EQ(data.localpos.z, 0.5);
    ASSERT_STREQ(data.quadstate.c_str(), "ARMED ENABLE_CONTROL ");
}

TEST(TFModelSimTests, Land) {
    TFModelSim quad_simulator_instance;
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

TEST(TFModelSimTests, Disarm) {
    TFModelSim quad_simulator_instance;
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

TEST(TFModelSimTests, DisableFlowControl) {
    TFModelSim quad_simulator_instance;
    quad_simulator_instance.takeoff();
    quad_simulator_instance.flowControl(false);
    //Get Data and ensure we are in right mode
    parsernode::common::quaddata data;
    quad_simulator_instance.getquaddata(data);
    ASSERT_EQ(data.localpos.z, 0.5);
    ASSERT_STREQ(data.quadstate.c_str(), "ARMED ");
}

TEST(TFModelSimTests, ImuCalibrate) {
    TFModelSim quad_simulator_instance;
    ASSERT_THROW(quad_simulator_instance.calibrateimubias(), std::runtime_error);
}

TEST(TFModelSimTests, TestBatteryPercent) {
  TFModelSim quad_simulator_instance;
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
