#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <parsernode/parser.h>
#include <ros/ros.h>


TEST(QuadSimulatorPluginTests, LoadPlugin) {
  std::unique_ptr<parsernode::Parser> quad_simulator_parser;
  pluginlib::ClassLoader<parsernode::Parser> parser_loader("parsernode", "parsernode::Parser");
  quad_simulator_parser.reset(parser_loader.createUnmanagedInstance("quad_simulator_parser/QuadSimParser"));
  quad_simulator_parser->initialize();
  SUCCEED();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "quad_simulator_plugin_load_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
