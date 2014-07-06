// MESSAGE ARM_CTRL_PWM PACKING

#define MAVLINK_MSG_ID_ARM_CTRL_PWM 223

typedef struct __mavlink_arm_ctrl_pwm_t
{
 uint16_t pwm1; ///< PWM of the first motor
 uint16_t pwm2; ///< PWM of the second motor
 uint16_t pwm3; ///< PWM of the third motor
} mavlink_arm_ctrl_pwm_t;

#define MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN 6
#define MAVLINK_MSG_ID_223_LEN 6

#define MAVLINK_MSG_ID_ARM_CTRL_PWM_CRC 170
#define MAVLINK_MSG_ID_223_CRC 170



#define MAVLINK_MESSAGE_INFO_ARM_CTRL_PWM { \
	"ARM_CTRL_PWM", \
	3, \
	{  { "pwm1", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_arm_ctrl_pwm_t, pwm1) }, \
         { "pwm2", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_arm_ctrl_pwm_t, pwm2) }, \
         { "pwm3", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_arm_ctrl_pwm_t, pwm3) }, \
         } \
}


/**
 * @brief Pack a arm_ctrl_pwm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pwm1 PWM of the first motor
 * @param pwm2 PWM of the second motor
 * @param pwm3 PWM of the third motor
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arm_ctrl_pwm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t pwm1, uint16_t pwm2, uint16_t pwm3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN];
	_mav_put_uint16_t(buf, 0, pwm1);
	_mav_put_uint16_t(buf, 2, pwm2);
	_mav_put_uint16_t(buf, 4, pwm3);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN);
#else
	mavlink_arm_ctrl_pwm_t packet;
	packet.pwm1 = pwm1;
	packet.pwm2 = pwm2;
	packet.pwm3 = pwm3;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARM_CTRL_PWM;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN, MAVLINK_MSG_ID_ARM_CTRL_PWM_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN);
#endif
}

/**
 * @brief Pack a arm_ctrl_pwm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pwm1 PWM of the first motor
 * @param pwm2 PWM of the second motor
 * @param pwm3 PWM of the third motor
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arm_ctrl_pwm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t pwm1,uint16_t pwm2,uint16_t pwm3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN];
	_mav_put_uint16_t(buf, 0, pwm1);
	_mav_put_uint16_t(buf, 2, pwm2);
	_mav_put_uint16_t(buf, 4, pwm3);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN);
#else
	mavlink_arm_ctrl_pwm_t packet;
	packet.pwm1 = pwm1;
	packet.pwm2 = pwm2;
	packet.pwm3 = pwm3;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARM_CTRL_PWM;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN, MAVLINK_MSG_ID_ARM_CTRL_PWM_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN);
#endif
}

/**
 * @brief Encode a arm_ctrl_pwm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param arm_ctrl_pwm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arm_ctrl_pwm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_arm_ctrl_pwm_t* arm_ctrl_pwm)
{
	return mavlink_msg_arm_ctrl_pwm_pack(system_id, component_id, msg, arm_ctrl_pwm->pwm1, arm_ctrl_pwm->pwm2, arm_ctrl_pwm->pwm3);
}

/**
 * @brief Encode a arm_ctrl_pwm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arm_ctrl_pwm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arm_ctrl_pwm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_arm_ctrl_pwm_t* arm_ctrl_pwm)
{
	return mavlink_msg_arm_ctrl_pwm_pack_chan(system_id, component_id, chan, msg, arm_ctrl_pwm->pwm1, arm_ctrl_pwm->pwm2, arm_ctrl_pwm->pwm3);
}

/**
 * @brief Send a arm_ctrl_pwm message
 * @param chan MAVLink channel to send the message
 *
 * @param pwm1 PWM of the first motor
 * @param pwm2 PWM of the second motor
 * @param pwm3 PWM of the third motor
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_arm_ctrl_pwm_send(mavlink_channel_t chan, uint16_t pwm1, uint16_t pwm2, uint16_t pwm3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN];
	_mav_put_uint16_t(buf, 0, pwm1);
	_mav_put_uint16_t(buf, 2, pwm2);
	_mav_put_uint16_t(buf, 4, pwm3);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARM_CTRL_PWM, buf, MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN, MAVLINK_MSG_ID_ARM_CTRL_PWM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARM_CTRL_PWM, buf, MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN);
#endif
#else
	mavlink_arm_ctrl_pwm_t packet;
	packet.pwm1 = pwm1;
	packet.pwm2 = pwm2;
	packet.pwm3 = pwm3;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARM_CTRL_PWM, (const char *)&packet, MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN, MAVLINK_MSG_ID_ARM_CTRL_PWM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARM_CTRL_PWM, (const char *)&packet, MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_arm_ctrl_pwm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t pwm1, uint16_t pwm2, uint16_t pwm3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, pwm1);
	_mav_put_uint16_t(buf, 2, pwm2);
	_mav_put_uint16_t(buf, 4, pwm3);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARM_CTRL_PWM, buf, MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN, MAVLINK_MSG_ID_ARM_CTRL_PWM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARM_CTRL_PWM, buf, MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN);
#endif
#else
	mavlink_arm_ctrl_pwm_t *packet = (mavlink_arm_ctrl_pwm_t *)msgbuf;
	packet->pwm1 = pwm1;
	packet->pwm2 = pwm2;
	packet->pwm3 = pwm3;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARM_CTRL_PWM, (const char *)packet, MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN, MAVLINK_MSG_ID_ARM_CTRL_PWM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARM_CTRL_PWM, (const char *)packet, MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ARM_CTRL_PWM UNPACKING


/**
 * @brief Get field pwm1 from arm_ctrl_pwm message
 *
 * @return PWM of the first motor
 */
static inline uint16_t mavlink_msg_arm_ctrl_pwm_get_pwm1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field pwm2 from arm_ctrl_pwm message
 *
 * @return PWM of the second motor
 */
static inline uint16_t mavlink_msg_arm_ctrl_pwm_get_pwm2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field pwm3 from arm_ctrl_pwm message
 *
 * @return PWM of the third motor
 */
static inline uint16_t mavlink_msg_arm_ctrl_pwm_get_pwm3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Decode a arm_ctrl_pwm message into a struct
 *
 * @param msg The message to decode
 * @param arm_ctrl_pwm C-struct to decode the message contents into
 */
static inline void mavlink_msg_arm_ctrl_pwm_decode(const mavlink_message_t* msg, mavlink_arm_ctrl_pwm_t* arm_ctrl_pwm)
{
#if MAVLINK_NEED_BYTE_SWAP
	arm_ctrl_pwm->pwm1 = mavlink_msg_arm_ctrl_pwm_get_pwm1(msg);
	arm_ctrl_pwm->pwm2 = mavlink_msg_arm_ctrl_pwm_get_pwm2(msg);
	arm_ctrl_pwm->pwm3 = mavlink_msg_arm_ctrl_pwm_get_pwm3(msg);
#else
	memcpy(arm_ctrl_pwm, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ARM_CTRL_PWM_LEN);
#endif
}
