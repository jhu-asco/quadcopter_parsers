#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <ros/ros.h>

using namespace quad_simulator;

TEST(QuadSimulatorTests, BaseCtor) {
  ASSERT_NO_THROW(QuadSimulator());
}

TEST(QuadSimulatorTests, Initializer) {
    QuadSimulator quad_simulator_instance;
    ASSERT_NO_THROW(quad_simulator_instance.non_ros_initialize());
}

TEST(QuadSimulatorTests, Takeoff) {
    QuadSimulator quad_simulator_instance;
    quad_simulator_instance.takeoff();
    //Get Data and ensure we are in right mode
    parsernode::common::quaddata data;
    quad_simulator_instance.getquaddata(data);
    ASSERT_EQ(data.localpos.z, 0.5);
    ASSERT_STREQ(data.quadstate.c_str(), "ARMED ENABLE_CONTROL ");
}

TEST(QuadSimulatorTests, Land) {
    QuadSimulator quad_simulator_instance;
    quad_simulator_instance.takeoff();
    quad_simulator_instance.land();
    //Get Data and ensure we are in right mode
    parsernode::common::quaddata data;
    quad_simulator_instance.getquaddata(data);
    ASSERT_EQ(data.localpos.x, 0.0);
    ASSERT_EQ(data.localpos.y, 0.0);
    ASSERT_EQ(data.localpos.z, 0.0);
    ASSERT_STREQ(data.quadstate.c_str(), "");
}

TEST(QuadSimulatorTests, Disarm) {
    QuadSimulator quad_simulator_instance;
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

TEST(QuadSimulatorTests, DisableFlowControl) {
    QuadSimulator quad_simulator_instance;
    quad_simulator_instance.takeoff();
    quad_simulator_instance.flowControl(false);
    //Get Data and ensure we are in right mode
    parsernode::common::quaddata data;
    quad_simulator_instance.getquaddata(data);
    ASSERT_EQ(data.localpos.z, 0.5);
    ASSERT_STREQ(data.quadstate.c_str(), "ARMED ");
}

TEST(QuadSimulatorTests, ImuCalibrate) {
    QuadSimulator quad_simulator_instance;
    ASSERT_THROW(quad_simulator_instance.calibrateimubias(), std::runtime_error);
}

TEST(QuadSimulatorTests, TestBatteryPercent) {
  QuadSimulator quad_simulator_instance;
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
