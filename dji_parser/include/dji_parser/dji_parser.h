#ifndef DJI_PARSER_H
#define DJI_PARSER_H
/* This header is subclassed from parser.h and will include data from DJI Matrice Quadcopter. It will also provide interfaces for commanding the DJI Quadcopter. It uses the official dji_sdk drivers to do the commanding and getting the navdata
 * This class uses internal locking to ensure the quaddata is available to Qt thread without bumping into ros serial. This is enough since the order of working is not important to us. Read internal vs external locking: http://www.boost.org/doc/libs/1_55_0/doc/html/thread/synchronization.html
 */
#include <parsernode/parser.h> //main parser base class

// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string>
#include <inttypes.h>
#include <fstream>
#include <sys/time.h>
#include <time.h>
#include <bitset>         // std::bitset
#include <boost/thread.hpp>

//Messages:
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>

//SDK library
#include <dji_sdk_lib/DJI_API.h>
#include <dji_sdk_lib/DJI_Flight.h>
#include <dji_sdk/DJI_HardDriver_Manifold.h>

//DJI SDK Helper:
#include "dji_sdk_helper.h"

#include <tf/transform_broadcaster.h>


#ifndef FILE_BUFFER_SIZE
#define FILE_BUFFER_SIZE 1024
#endif

using namespace std;
namespace dji_parser{
class DjiParser: public parsernode::Parser
{
private:
    enum FlightStatus {
        STANDBY = 1,
        TAKEOFF = 2,
        IN_AIR = 3,
        LANDING = 4,
        FINISH_LANDING=5
    };
    enum HardwareType {
        MATRICE,
        A3
    };
    ros::NodeHandle nh_;///< Internal node handle

    //Members depicting the state of the quadcopter
    uint8_t control_mode;///< Mode corresponding to dji
    bool rp_angle_yawrate_mode;///< State used to switch between rate control vs angle control of y in rpyt
    bool vel_yaw_ratemode;///< State used to switch between rate control vs angle control of y in vel
    
    //File Streams
    ofstream cmdfile;//Cmd logging
    ofstream servofile;//Raw servo pwm logging
    ofstream rcinputfile;//Raw rc input logging
    ofstream imufile;//Imu data logging
    ofstream velfile;//Vel data logging
    ofstream accfile;//Acc data logging
    ofstream magfile;//Mag data logging
    ofstream localposfile;//Local pos data logging
    ofstream statusfile;//Status of various systems controlling quadrotor
    //Publishers ROS
    ros::Publisher global_ref_pub;
    ros::Publisher gps_pub;

    ros::Subscriber guidance_sub_;

    ros::Timer tf_timer_;
    tf::TransformBroadcaster tf_broadcaster;
    //GPS Pub Info
    double gps_pub_rate; //Hz
    ros::Time last_gps_pub_time;
    //Create Buffers for each of these files:
    char cmdfile_buffer[FILE_BUFFER_SIZE];//Buffer for ofstream
    char imufile_buffer[FILE_BUFFER_SIZE];//Buffer for ofstream
    char velfile_buffer[FILE_BUFFER_SIZE];//Buffer for ofstream
    char accfile_buffer[FILE_BUFFER_SIZE];//Buffer for ofstream
    char magfile_buffer[FILE_BUFFER_SIZE];//Buffer for ofstream
    char servofile_buffer[FILE_BUFFER_SIZE];//Buffer for ofstream
    char localposfile_buffer[FILE_BUFFER_SIZE];//Buffer for ofstream
    char statusfile_buffer[FILE_BUFFER_SIZE];//Buffer for ofstream
    char rcinputfile_buffer[FILE_BUFFER_SIZE];//Buffer for ofstream
    bool enable_log;
    int fd;
    //DJI SDK Member Variables:
    DJI::onboardSDK::ActivateData user_act_data_;///< App activation data dji
    DJI::onboardSDK::VersionData version_data;///< Provides the sdk version, hardware type used
    bool sdk_opened;
    double global_ref_lat, global_ref_long;///<Lat and Long of Home
    double global_ref_x, global_ref_y, global_ref_z;
    bool use_guidance_pos_;
    uint8_t quad_status;///< Quad status standby takeoff etc
    uint8_t ctrl_mode;///< Quadcopter Controlled by either RC or APP or SER
    uint8_t sdk_status;///< Whether sdk is open or close
    uint8_t gps_health;///< Health of GPS
    uint8_t shift_bit;///< For hardware a3, N3, M600, the shift bit is 2 for extracting certain data
    uint16_t rc_f_pwm; ///< RC switch signal given when in SDK mode
    HardwareType hardware_type; ///< Flight controller hardware


    void guidanceCallback(const geometry_msgs::Point&);
    void tfTimerCallback(const ros::TimerEvent&);
    static void* APIRecvThread(void* param);
    static void statReceiveDJIData(DJI::onboardSDK::CoreAPI *, DJI::onboardSDK::Header *, void *);//receive dji data from its lib 
    int init_parameters_and_activate(ros::NodeHandle& nh_, DJI::onboardSDK::ActivateData* user_act_data);
    void init(std::string device, unsigned int baudrate);
    static void takeoffCb(DJI::onboardSDK::CoreAPI *, DJI::onboardSDK::Header * header, void * userData);
    static void landingCb(DJI::onboardSDK::CoreAPI *, DJI::onboardSDK::Header * header, void * userData);

    DJI::onboardSDK::CoreAPI* coreAPI;
    DJI::onboardSDK::Flight* flight;
    pthread_t m_recvTid;

    struct CbResponse
    {
      CbResponse() : received(false), succeeded(false) {}
      volatile bool received;
      volatile bool succeeded;
    };

protected:
    parsernode::common::quaddata data; ///< Collected data struct for the entire quadrotor state
    boost::mutex spin_mutex; ///< Mutex to sync receiving data from DJI and getter functions
    virtual void receiveDJIData();//receive dji data from its lib 

public:
    DjiParser();
    virtual ~DjiParser()
    {
        disarm();//release sdk control
    }
    //Extend functions from Parser:
    virtual bool takeoff();
    virtual bool land();
    virtual bool disarm();
    virtual bool flowControl(bool);
    virtual bool calibrateimubias();
    virtual bool cmdrpythrust(geometry_msgs::Quaternion &rpytmsg, bool sendyaw = false);
    virtual bool cmdvelguided(geometry_msgs::Vector3 &vel_cmd, double &yaw_inp);
    virtual bool cmdvel_yaw_rate_guided(geometry_msgs::Vector3 &vel_cmd, double &yaw_rate);
    virtual bool cmdvel_yaw_angle_guided(geometry_msgs::Vector3 &vel_cmd, double &yaw_angle);
    virtual bool cmdwaypoint(geometry_msgs::Vector3 &desired_pos, double desired_yaw = 0);
    void grip(int state);
    void reset_attitude(double roll, double pitch, double yaw);
    void setmode(std::string mode);
    void initialize();
    void getquaddata(parsernode::common::quaddata &d1);
    void setaltitude(double altitude_)
    {
        return;
    }

    void setlogdir(string logdir)
    {
        cmdfile.open((logdir+"/cmd.dat").c_str());
        servofile.open((logdir+"/servo.dat").c_str());
        rcinputfile.open((logdir+"/rcinput.dat").c_str());
        imufile.open((logdir+"/imu.dat").c_str());
        velfile.open((logdir+"/vel.dat").c_str());
        accfile.open((logdir+"/acc.dat").c_str());
        magfile.open((logdir+"/mag.dat").c_str());
        localposfile.open((logdir+"/localposfile.dat").c_str());
        statusfile.open((logdir+"/statusfile.dat").c_str());

        cmdfile.precision(10);
        servofile.precision(10);
        rcinputfile.precision(10);
        imufile.precision(10);
        velfile.precision(10);
        accfile.precision(10);
        magfile.precision(10);
        localposfile.precision(10);
        statusfile.precision(10);

        //Create Buffer:
        imufile.rdbuf()->pubsetbuf(imufile_buffer, FILE_BUFFER_SIZE);
        cmdfile.rdbuf()->pubsetbuf(cmdfile_buffer, FILE_BUFFER_SIZE);
        servofile.rdbuf()->pubsetbuf(servofile_buffer, FILE_BUFFER_SIZE);
        imufile.rdbuf()->pubsetbuf(imufile_buffer, FILE_BUFFER_SIZE);
        velfile.rdbuf()->pubsetbuf(velfile_buffer, FILE_BUFFER_SIZE);
        accfile.rdbuf()->pubsetbuf(accfile_buffer, FILE_BUFFER_SIZE);
        magfile.rdbuf()->pubsetbuf(magfile_buffer, FILE_BUFFER_SIZE);
        localposfile.rdbuf()->pubsetbuf(localposfile_buffer, FILE_BUFFER_SIZE);
        statusfile.rdbuf()->pubsetbuf(statusfile_buffer, FILE_BUFFER_SIZE);

        //#DEBUG Print Buffer size:
        std::cout<<"Imu file buffer size: "<<imufile.rdbuf()->in_avail();

        cmdfile<<"#Time\t Roll \t Pitch \t Yaw \t Thrust"<<endl;
        servofile<<"#Time\t SERVO_1\t SERVO_2\t SERVO_3\t SERVO_4\t TIME_US\t BATT_VOLTS"<<endl;
        rcinputfile<<"#Time\t RC_1\t RC_2\t RC_3\t RC_4"<<endl;
        imufile<<"#Time\t Roll \t Pitch \t Yaw\t Wx\t Wy\t Wz"<<endl;
        velfile<<"#Time\t Vx \t Vy \t Vz"<<endl;
        accfile<<"#Time\t Ax \t Ay \t Az"<<endl;
        magfile<<"#Time\t Mx \t My \t Mz"<<endl;
        localposfile<<"#Time\t Posx \t Posy \t Posz"<<endl;
        statusfile<<"#Time\t quad_status\t sdk_status \t ctrl_mode \t gps_health\t vel_rateflag\t rpy_rateflag"<<endl;
    }
    void controllog(bool logswitch)
    {
        enable_log = logswitch;
    }
};
}
#endif // DJI_PARSHER_H
