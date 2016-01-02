#ifndef DJI_PARSER_H
#define DJI_PARSHER_H
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

//DJI SDK Helper:
#include "dji_sdk_helper.h"


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

private:
    //Members depicting the state of the quadcopter
    parsernode::common::quaddata data;
    uint8_t control_mode;///< Mode corresponding to dji
    //File Streams
    ofstream cmdfile;//Cmd logging
    ofstream servofile;//Raw servo pwm logging
    ofstream rcinputfile;//Raw rc input logging
    ofstream imufile;//Imu data logging
    //Create Buffers for each of these files:
    char cmdfile_buffer[FILE_BUFFER_SIZE];//Buffer for ofstream
    char imufile_buffer[FILE_BUFFER_SIZE];//Buffer for ofstream
    char servofile_buffer[FILE_BUFFER_SIZE];//Buffer for ofstream
    char rcinputfile_buffer[FILE_BUFFER_SIZE];//Buffer for ofstream
    bool enable_log;
    int fd;
    //DJI SDK Member Variables:
    activate_data_t user_act_data_;///< App activation data dji
    bool sdk_opened;
    double global_ref_lat, global_ref_long;///<Lat and Long of Home
    boost::mutex spin_mutex;
    uint8_t quad_status;///< Quad status standby takeoff etc
    uint8_t ctrl_mode;///< Quadcopter Controlled by either RC or APP or SER
    uint8_t sdk_status;///< Whether sdk is open or close
    uint8_t gps_health;///< Health of GPS


private:
    void receiveDJIData();//receive dji data from its lib 
public:
    DjiParser();
    ~DjiParser()
    {
        disarm();//release sdk control
    }
    //Extend functions from Parser:
    bool takeoff();
    bool land();
    bool disarm();
    bool flowControl(bool);
    bool calibrateimubias();
    bool cmdrpythrust(geometry_msgs::Quaternion &rpytmsg, bool sendyaw = false);
    bool cmdvelguided(geometry_msgs::Vector3 &vel_cmd, double &yaw_ang);
    void grip(int state);
    void reset_attitude(double roll, double pitch, double yaw);
    void setmode(std::string mode);
    void initialize(ros::NodeHandle &nh_);
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

        cmdfile.precision(10);
        servofile.precision(10);
        rcinputfile.precision(10);
        imufile.precision(10);
        //Create Buffer:
        imufile.rdbuf()->pubsetbuf(imufile_buffer, FILE_BUFFER_SIZE);
        cmdfile.rdbuf()->pubsetbuf(cmdfile_buffer, FILE_BUFFER_SIZE);
        servofile.rdbuf()->pubsetbuf(servofile_buffer, FILE_BUFFER_SIZE);
        imufile.rdbuf()->pubsetbuf(imufile_buffer, FILE_BUFFER_SIZE);
        //#DEBUG Print Buffer size:
        std::cout<<"Imu file buffer size: "<<imufile.rdbuf()->in_avail();

        cmdfile<<"#Time\t Roll \t Pitch \t Yaw \t Thrust"<<endl;
        servofile<<"#Time\t SERVO_1\t SERVO_2\t SERVO_3\t SERVO_4\t TIME_US\t BATT_VOLTS"<<endl;
        rcinputfile<<"#Time\t RC_1\t RC_2\t RC_3\t RC_4"<<endl;
        imufile<<"#Time\t Roll \t Pitch \t Yaw"<<endl;
    }
    void controllog(bool logswitch)
    {
        enable_log = logswitch;
    }
};
}
#endif // DJI_PARSHER_H
