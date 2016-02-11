#include <pluginlib/class_loader.h>
#include <parsernode/parser.h>
#include <boost/thread.hpp>

//using namespace rqt_quadcopter_parsers;

boost::mutex user_mutex_;
boost::shared_ptr<parsernode::Parser> quad_simulator_parser;

void userFunction()
{
  std::string  input;
  std::cout<<"Menu: Arm; Disarm; Status; Quit"<<endl;
  while(ros::ok())
  {
    std::cin>>input;
    user_mutex_.lock();
    if(!strcmp("Arm", input.c_str()))
    {
      quad_simulator_parser->takeoff();
    }
    else if(!strcmp("Disarm", input.c_str()))
    {
      quad_simulator_parser->land();
    }
    else if(!strcmp("Status",input.c_str()))
    {
      parsernode::common::quaddata data;
      quad_simulator_parser->getquaddata(data);
      //Print Status of Quadcopter
      std::cout<<"data.quadstate: "<<data.quadstate<<std::endl;
    }
    else if(!strcmp("Quit", input.c_str()))
    {
      quad_simulator_parser.reset();
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
    quad_simulator_parser = parser_loader.createInstance("quad_simulator_parser/QuadSimParser");
    quad_simulator_parser->initialize(nh_);

    ROS_INFO("Created Quad Sim Parser Successfully");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }
  //Create Boost Thread
  while(!quad_simulator_parser->initialized && ros::ok());
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
