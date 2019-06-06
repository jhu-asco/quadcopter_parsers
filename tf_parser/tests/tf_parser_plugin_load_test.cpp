#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <parsernode/parser.h>
#include <ros/ros.h>


TEST(TFParserPluginTests, LoadPlugin) {
  std::unique_ptr<parsernode::Parser> tf_parser;
  pluginlib::ClassLoader<parsernode::Parser> parser_loader("parsernode", "parsernode::Parser");
  tf_parser.reset(parser_loader.createUnmanagedInstance("tf_parser/TFParser"));
  tf_parser->initialize();
  SUCCEED();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_parser_plugin_load_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
