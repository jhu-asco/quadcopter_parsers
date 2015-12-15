#ifndef COMMONDEF
#define COMMONDEF
#include <ardupilotmega/mavlink.h>
#include "string"

// Auto Pilot modes Got from defines.h in ArduCopter code
// ----------------
enum autopilot_modes {
  STABILIZE =     0,  // manual airframe angle with manual throttle
  ACRO =          1,  // manual body-frame angular rate with manual throttle
  ALT_HOLD =      2,  // manual airframe angle with automatic throttle
  AUTO =          3,  // fully automatic waypoint control using mission commands
  GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
  LOITER =        5,  // automatic horizontal acceleration with automatic throttle
  RTL =           6,  // automatic return to launching point
  CIRCLE =        7,  // automatic circular flight with automatic throttle
  LAND =          9,  // automatic landing with horizontal position control
  OF_LOITER =    10,  // deprecated
  DRIFT =        11,  // semi-automous position, yaw and throttle control
  SPORT =        13,  // manual earth-frame angular rate control with manual throttle
  FLIP =         14,  // automatically flip the vehicle on the roll axis
  AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
  POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
  BRAKE =        17   // full-brake using inertial/GPS system, no pilot input
};

#define hostsysid 255
#define hostcompid 110

using namespace std;
//casting function
static uint8_t mavlink_crcs[] = MAVLINK_MESSAGE_CRCS;
static uint8_t mavlink_msg_len[] = MAVLINK_MESSAGE_LENGTHS;
/*static void castmavmsgtoros(mavlink_ros::Mavlink &mavlink_ros_msg, mavlink_message_t &mavmsg)
{
  //mavlink_ros::Mavlink mavlink_ros_msg;
  //manually cast the mavlink_msg_t to mavlink_ros_msg
  mavlink_ros_msg.len = mavmsg.len;
  mavlink_ros_msg.seq = mavmsg.seq;
  mavlink_ros_msg.sysid = mavmsg.sysid;
  mavlink_ros_msg.compid = mavmsg.compid;
  mavlink_ros_msg.msgid = mavmsg.msgid;
  uint8_t PAYLOAD_SIZE = mavlink_msg_len[mavmsg.msgid];
  for (int i = 0; i < PAYLOAD_SIZE ; i++)
      {
        (mavlink_ros_msg.payload64).push_back(mavmsg.payload64[i]);
      }
     //ROS_INFO("%u\t%u",((MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7)/8),mavmsg.len );
}
*/


  /**
static void castrosmsgtomav(mavlink_ros::Mavlink mavlink_ros_msg,mavlink_message_t &msg)
{
	*** PREVIOUS COMMENT START
   * Convert mavlink_ros::Mavlink to mavlink_message_t
	 *** PREVIOUS COMMENT END
  msg.msgid = mavlink_ros_msg.msgid;
  copy(mavlink_ros_msg.payload64.begin(), mavlink_ros_msg.payload64.end(), msg.payload64);
  mavlink_finalize_message_chan(&msg, mavlink_ros_msg.sysid, mavlink_ros_msg.compid, MAVLINK_COMM_0,
                                mavlink_ros_msg.len, mavlink_crcs[msg.msgid]);
}
   */

static string custom_mode_map(uint32_t custom_mode)
{
	string result = "";
	
  switch (custom_mode) {
    case 0 : result = " STABILIZE"; break;
    case 1 : result = " ACRO"; break;
    case 2 : result = " ALT_HOLD"; break;
    case 3 : result = " AUTO"; break;
    case 4 : result = " GUIDED"; break;
    case 5 : result = " LOITER"; break;
    case 6 : result = " RTL"; break;
    case 7 : result = " CIRCLE"; break;
    case 8 : result = " POSITION"; break;
    case 9 : result = " LAND"; break;
    case 10 : result = " OF_LOITER"; break;
    case 11 : result = " DRIFT"; break;
    case 13 : result = " SPORT"; break;
    case 14 : result = " FLIP"; break;
    case 15 : result = " AUTOTUNE"; break;
    case 16 : result = " POSHOLD"; break;
  }
return result;
}
static string base_mode_map(uint8_t base_mode)
{
	string result = "";
	uint8_t filter = 0x01;
	for(int i = 0;i<8;i++)
	{
					//fprintf(stdout,"%d %d %d \n",i,filter,base_mode);
					if(filter&base_mode)
					{
									switch(i)
									{
													case 0:
													result += " CUSTOM_MODE_ENABLED";
													break;
													case 1:
													result += " FLAG_TEST_ENABLED";
													break;
													case 2:
													result += " FLAG_AUTO_ENABLED";
													break;
													case 3:
													result += " FLAG_GUIDED_ENABLED";
													break;
													case 4:
													result += " FLAG_STABILIZE_ENABLED";
													break;
													case 5:
													result += " FLAG_HIL_ENABLED";
													break;
													case 6:
													result += " FLAG_MANUAL_INPUT_ENABLED";
													break;
													case 7:
													result += " FLAG_SAFETY_ENABLED";
													break;
									}
					}
						filter = filter<<1; 
	}
return result;
}
static string mavlink_message_type_map(int val)
{
	switch(val)
	{
		case 0:
			return "MAVLINK_TYPE_CHAR";
		case 1:
			return "MAVLINK_TYPE_UINT8_T";
		case 2:
			return "MAVLINK_TYPE_INT8_T";
		case 3:
				return "MAVLINK_TYPE_UINT16_T";
		case 4:
				return "MAVLINK_TYPE_INT16_T";
		case 5:
				return "MAVLINK_TYPE_UINT32_T";
		case 6:
				return "MAVLINK_TYPE_INT32_T";
		case 7:
				return "MAVLINK_TYPE_UINT64_T";
		case 8:
				return "MAVLINK_TYPE_INT64_T";
		case 9:
				return "MAVLINK_TYPE_FLOAT";
		case 10:
				return "MAVLINK_TYPE_DOUBLE";
	}
	return "NULL";
}
#endif
