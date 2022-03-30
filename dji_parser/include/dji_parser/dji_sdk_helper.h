#ifndef DJI_SDK_HELPER_H
#define DJI_SDK_HELPER_H
#include <iostream>

//DJI Messages
//#include <dji_sdk/GlobalPosition.h>
//#include <dji_sdk/LocalPosition.h>

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

};
#endif
