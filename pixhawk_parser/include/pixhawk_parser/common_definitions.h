#ifndef COMMONDEF
#define COMMONDEF
#include <ardupilotmega/mavlink.h>
#include "string"

// Auto Pilot modes Got from defines.h in ArduCopter code
// ----------------
#define STABILIZE 0                     // hold level position
#define ACRO 1                          // rate control
#define ALT_HOLD 2                      // AUTO control
#define AUTO 3                          // AUTO control
#define GUIDED 4                        // AUTO control
#define LOITER 5                        // Hold a single location
#define RTL 6                           // AUTO control
#define CIRCLE 7                        // AUTO control
#define LAND 9                          // AUTO control
#define OF_LOITER 10                    // Hold a single location using optical flow sensor
#define DRIFT 11                        // DRIFT mode (Note: 12 is no longer used)
#define SPORT 13                        // earth frame rate control
#define FLIP        14                  // flip the vehicle on the roll axis
#define AUTOTUNE    15                  // autotune the vehicle's roll and pitch gains
#define HYBRID      16                  // hybrid - position hold with manual override
#define NUM_MODES   17

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
													result += "\tCUSTOM_MODE_ENABLED";
													break;
													case 1:
													result += "\tFLAG_TEST_ENABLED";
													break;
													case 2:
													result += "\tFLAG_AUTO_ENABLED";
													break;
													case 3:
													result += "\tFLAG_GUIDED_ENABLED";
													break;
													case 4:
													result += "\tFLAG_STABILIZE_ENABLED";
													break;
													case 5:
													result += "\tFLAG_HIL_ENABLED";
													break;
													case 6:
													result += "\tFLAG_MANUAL_INPUT_ENABLED";
													break;
													case 7:
													result += "\tFLAG_SAFETY_ENABLED";
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
