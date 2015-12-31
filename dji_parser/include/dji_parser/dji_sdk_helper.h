#ifndef DJI_SDK_HELPER_H
#define DJI_SDK_HELPER_H
#include <iostream>
//SDK library
#include <dji_sdk/DJI_LIB/DJI_Pro_App.h>
#include <dji_sdk/DJI_LIB/DJI_Pro_Codec.h>
#include <dji_sdk/DJI_LIB/DJI_Pro_Config.h>
#include <dji_sdk/DJI_LIB/DJI_Pro_Hw.h>
#include <dji_sdk/DJI_LIB/DJI_Pro_Link.h>
#include <dji_sdk/DJI_LIB/DJI_Pro_Rmu.h>

//DJI Messages
#include <dji_sdk/GlobalPosition.h>
#include <dji_sdk/LocalPosition.h>

#define C_EARTH (double) 6378137.0
#define C_PI (double) 3.141592653589793

#define DEG2RAD(DEG) ((DEG)*((C_PI)/(180.0)))

// Defined in DJI_Pro_App.h
//typedef std::function<void()> User_Broadcast_Handler_Func;

/** Helper Functions copied from DJI SDK Main Node 
*/
namespace DJI_SDK {

  inline void gps_convert_ned(double &ned_x, double &ned_y,
      double gps_t_lon, double gps_t_lat,
      double gps_r_lon, double gps_r_lat)
  {
    double d_lon = gps_t_lon - gps_r_lon;
    double d_lat = gps_t_lat - gps_r_lat;
    ned_x = DEG2RAD(d_lat) * C_EARTH;
    ned_y = DEG2RAD(d_lon) * C_EARTH * cos(DEG2RAD(gps_t_lat));
  }

  int DJI_Setup(std::string serial_port, int baudrate) 
  {
    int ret;
    char uart_name[32];
    strcpy(uart_name, serial_port.c_str());
    printf("Serial port: %s\n", uart_name);
    printf("Baudrate: %d\n", baudrate);
    printf("=========================\n");

    //Serial Port Init
    ret = Pro_Hw_Setup(uart_name, baudrate);
    if(ret < 0)
      return ret;

    //Setup Other Things
    DJI_Pro_Setup(NULL);
    return 0;
  }

  int init_parameters_and_activate(ros::NodeHandle& nh_, activate_data_t &user_act_data, User_Broadcast_Handler_Func broadcast_function)
  {
    std::string serial_name;
    int baud_rate;
    int app_id;
    int app_api_level;
    int app_version;
    std::string app_bundle_id;
    std::string enc_key;

    nh_.param("serial_name", serial_name, std::string("/dev/cu.usbserial-A603T4HK"));
    nh_.param("baud_rate", baud_rate, 230400);
    nh_.param("app_id", app_id, 1022384);
    nh_.param("app_api_level", app_api_level, 2);
    nh_.param("app_version", app_version, 1);
    nh_.param("app_bundle_id", app_bundle_id, std::string("12345678901234567890123456789012"));
    nh_.param("enc_key", enc_key,
        std::string("e7bad64696529559318bb35d0a8c6050d3b88e791e1808cfe8f7802150ee6f0d"));

    // activation
    user_act_data.app_id = app_id;
    user_act_data.app_api_level = app_api_level;
    user_act_data.app_ver = SDK_VERSION;
    strcpy((char*) user_act_data.app_bundle_id, app_bundle_id.c_str());
    user_act_data.app_key = new char[65];//Create a char on heap
    strcpy(user_act_data.app_key, enc_key.c_str());

    printf("=================================================\n");
    printf("app id: %d\n", user_act_data.app_id);
    printf("api level: %d\n", user_act_data.app_api_level);
    printf("app version: 0x0%X\n", user_act_data.app_ver);
    printf("app key: %s\n", user_act_data.app_key);
    printf("=================================================\n");

    if (DJI_Setup(serial_name.c_str(), baud_rate) < 0) {
      printf("Serial Port Cannot Open\n");
      return -1;
    }

    DJI_Pro_Activate_API(&user_act_data, NULL);
    DJI_Pro_Register_Broadcast_Callback(broadcast_function);

    return 0;
  }
};
#endif
