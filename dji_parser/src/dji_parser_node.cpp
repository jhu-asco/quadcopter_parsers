#include <pluginlib/class_loader.h>
//#include <parsernode/parser.h>
#include <dji_parser/dji_parser.h>
#include <boost/thread.hpp>

//using namespace rqt_quadcopter_parsers;

boost::mutex user_mutex_;
boost::shared_ptr<parsernode::Parser> parser;

void userFunction()
{
  std::string  input;
  std::cout<<"Menu: Grip; Quit"<<endl;
  while(ros::ok())
  {
    std::cin>>input;
    user_mutex_.lock();
    if(!strcmp("Arm", input.c_str()))
    {
      parser->takeoff();
    }
    else if(!strcmp("Disarm", input.c_str()))
    {
      parser->land();
    }
    else if(!strcmp("Status",input.c_str()))
    {
      parsernode::common::quaddata data;
      parser->getquaddata(data);
      //Print Status of Quadcopter
      std::cout<<"data.quadstate: "<<data.quadstate<<std::endl
	      <<" battery: "<<data.batterypercent<<std::endl
	      <<" rpydata: "<<data.rpydata<<std::endl
	      <<" linvel: "<<data.linvel<<std::endl
	      <<" linacc: "<<data.linacc<<std::endl
	      <<" localpos: "<<data.localpos<<std::endl
	      <<" armed: "<<data.armed<<std::endl
	      <<" rc_sdk: "<<data.rc_sdk_control_switch<<std::endl;
    }
    else if(!strcmp("Quit", input.c_str()))
    {
      parser.reset();
      ros::shutdown();
    }
    user_mutex_.unlock();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "parser");
  pluginlib::ClassLoader<parsernode::Parser> parser_loader("parsernode", "parsernode::Parser");
  ROS_INFO("I am fine");
  try
  {
    parser = parser_loader.createInstance("dji_parser/DjiParser");
    //parser.reset(new dji_parser::DjiParser());
    parser->initialize();
    ROS_INFO("DJI ACtivation: %d",parser->initialized);

    ROS_INFO("Created DJI Parser Successfully");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }
  //Create Boost Thread
  while(!parser->initialized && ros::ok());
  boost::thread user_thread(userFunction);
  ros::Rate loop_rate(100);
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
