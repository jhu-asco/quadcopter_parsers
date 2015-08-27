#include <pluginlib/class_loader.h>
#include <parsernode/parser.h>
#include <boost/thread.hpp>

//using namespace rqt_quadcopter_parsers;

boost::mutex user_mutex_;
boost::shared_ptr<parsernode::Parser> pixhawk_parser;

void userFunction()
{
  std::string  input;
  std::cout<<"Menu: Grip; Quit"<<endl;
  while(ros::ok())
  {
    std::cin>>input;
    user_mutex_.lock();
    if(!strcmp("Grip", input.c_str()))
    {
      std::cout<<"Open[O] Close[C] Relax[R]"<<std::endl;
      char sub_input;
      sub_input = getchar();
      if(sub_input == 'O')
      {
        pixhawk_parser->grip(-1);
      }
      else if(sub_input == 'C')
      {
        pixhawk_parser->grip(1);
      }
      else if(sub_input == 'R')
      {
        pixhawk_parser->grip(0);
      }
    }
    else if(!strcmp("Quit", input.c_str()))
    {
      ros::shutdown();
    }
    user_mutex_.unlock();
  }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "parser");
	ros::NodeHandle nh_;
	pluginlib::ClassLoader<parsernode::Parser> parser_loader("parsernode", "parsernode::Parser");
	ROS_INFO("I am fine");
	try
	{
		pixhawk_parser = parser_loader.createInstance("pixhawk_parser/PixhawkParser");
		pixhawk_parser->initialize(nh_);

		ROS_INFO("Created Pixhawk Parser Successfully");
	}
	catch(pluginlib::PluginlibException& ex)
	{
		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
	}
  //Create Boost Thread
  boost::thread user_thread(userFunction);
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
    user_mutex_.lock();
		ros::spinOnce();
    user_mutex_.unlock();
		loop_rate.sleep();
	}
	return 0;
}
