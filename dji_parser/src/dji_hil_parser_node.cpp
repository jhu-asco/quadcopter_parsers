#include <pluginlib/class_loader.h>
#include <parsernode/parser.h>
#include <boost/thread.hpp>

//using namespace rqt_quadcopter_parsers;

boost::mutex user_mutex_;
boost::shared_ptr<parsernode::Parser> dji_parser;

void userFunction()
{
  std::string  input;
  std::cout<<"Menu: Status; Arm; Disarm; Quit"<<endl;
  while(ros::ok())
  {
    std::cin>>input;
    user_mutex_.lock();
    if(!strcmp("Arm", input.c_str()))
    {
      dji_parser->takeoff();
    }
    else if(!strcmp("Disarm", input.c_str()))
    {
      dji_parser->land();
    }
    else if(!strcmp("Status",input.c_str()))
    {
      parsernode::common::quaddata data;
      dji_parser->getquaddata(data);
      //Print Status of Quadcopter
      std::cout<<"data.quadstate: "<<data.quadstate<<std::endl;
    }
    else if(!strcmp("Quit", input.c_str()))
    {
      dji_parser.reset();
      ros::shutdown();
    }
    user_mutex_.unlock();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "parser");
  ros::NodeHandle nh_("~");
  pluginlib::ClassLoader<parsernode::Parser> parser_loader("parsernode", "parsernode::Parser");
  ROS_INFO("I am fine");
  try
  {
    dji_parser = parser_loader.createInstance("dji_parser/DjiHILParser");
    dji_parser->initialize(nh_);
    ROS_INFO("DJI ACtivation: %d",dji_parser->initialized);

    ROS_INFO("Created DJI Parser Successfully");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }
  //Create Boost Thread
  while(!dji_parser->initialized && ros::ok());
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
