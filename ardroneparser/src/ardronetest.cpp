#include <pluginlib/class_loader.h>
#include <parsernode/parser.h>

//using namespace rqt_quadcopter_parsers;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "parser");
	ros::NodeHandle nh_;
	pluginlib::ClassLoader<parsernode::Parser> parser_loader("parsernode", "parsernode::Parser");
	ROS_INFO("I am fine");
	//parsernode::Parser *ardrone_parser = NULL;
	try
	{
		boost::shared_ptr<parsernode::Parser> ardrone_parser = parser_loader.createInstance("ardroneparser/ArdroneParser");
		ardrone_parser->initialize();

		ROS_INFO("Created Ardrone Parser Successfully");
	}
	catch(pluginlib::PluginlibException& ex)
	{
		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
	}
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
