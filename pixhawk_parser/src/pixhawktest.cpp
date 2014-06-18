#include <pluginlib/class_loader.h>
#include <rqt_quadcoptergui/parser.h>

//using namespace rqt_quadcopter_parsers;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "parser");
	ros::NodeHandle nh_;
	pluginlib::ClassLoader<rqt_quadcoptergui::Parser> parser_loader("rqt_quadcoptergui", "rqt_quadcoptergui::Parser");
	ROS_INFO("I am fine");
	rqt_quadcoptergui::Parser *ardrone_parser = NULL;
	try
	{
		ardrone_parser = parser_loader.createClassInstance("pixhawk_parser/PixhawkParser");
		ardrone_parser->initialize(nh_);

		ROS_INFO("Created Pixhawk Parser Successfully");
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
