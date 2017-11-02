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
      std::cin>>sub_input;
      std::cout<<"sub_input: "<<sub_input<<std::endl;
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
    else if(!strcmp("Arm", input.c_str()))
    {
      pixhawk_parser->takeoff();
    }
    else if(!strcmp("Disarm", input.c_str()))
    {
      pixhawk_parser->land();
    }
    else if(!strcmp("Mode", input.c_str()))
    {
      std::cout<<"Available Modes: STABILIZE ALT_HOLD LAND POSHOLD"<<std::endl;
      std::string sub_input;
      std::cin>>sub_input;
      std::cout<<"sub_input: "<<sub_input<<std::endl;
      pixhawk_parser->setmode(sub_input);
    }
    else if(!strcmp("Status",input.c_str()))
    {
      parsernode::common::quaddata data;
      pixhawk_parser->getquaddata(data);
      //Print Status of Quadcopter
      std::cout<<"data.quadstate: "<<data.quadstate<<std::endl;
    }
    else if(!strcmp("Quit", input.c_str()))
    {
      pixhawk_parser.reset();
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
		pixhawk_parser->initialize();

		ROS_INFO("Created Pixhawk Parser Successfully");
	}
	catch(pluginlib::PluginlibException& ex)
	{
		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
	}
  //Create Boost Thread
  while(!pixhawk_parser->initialized && ros::ok());
  boost::thread user_thread(userFunction);
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
    user_mutex_.lock();
		ros::spinOnce();
    user_mutex_.unlock();
		loop_rate.sleep();
	}
  user_thread.join();
	return 0;
}
