#pragma once
/**
  ******************************************************************************
  * @file    GKV_CustomPacket.h
  * @author  Alex Galkin,  <info@mp-lab.ru>
  * @brief   GKV-10 structures module for packets WITH CUSTOM PARAMETERS OF EACH ALGORITHM
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 Laboratory of Microdevices, Ltd. </center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of Laboratory of Microdevices nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#ifndef __GKV_CUSTOM_PACKET_H__
#define __GKV_CUSTOM_PACKET_H__

/** @defgroup CUSTOM_PACKET_CODES
  * @brief    packet codes for "type" field of custom data/parameters packet
  * @{
  */ 
#define GKV_CUSTOM_PACKET 								0x13								/*	type of data packet custom data	*/
#define GKV_CUSTOM_PACKET_PARAM_REQUEST 	            0x26								/*	type of empty packet (with length = 0) to request parameters of "custom data packet"	*/
#define GKV_CUSTOM_DATA_PARAM_PACKET		 	        0x27								/*	type of packet with current parameters list of "custom data packet"	*/
/**
  * @}
  */


/** 
  * @brief  Any algorithm. Packet 0x13 with custom parameters that can be selected instead standart packet of each algorithm (receive only).
  */
typedef struct __CustomData
{
	uint8_t number_of_parameters;           /*	number of parameters that device sends in custom packet	*/
	float paramerer[65];                    /*	value of parameter N from list of 'custom_data_parameters'	*/
}CustomData;


/** @defgroup CUSTOM_PACKET_PARAMETERS
  * @brief 		LIST OF PARAMETER CODES FOR CUSTOM RESPONSE PACKET
  * @{
  */
#define STATUS 														    0x00					/* device status */
#define SAMPLE_COUNTER 										            0x01					/*	16-bit sample counter to control lost packets (changes from 0 to 65535)	*/
#define NAX 															0x02					/*	non-calibrated data from X axis of accelerometer 	*/
#define NAY 															0x03					/*	non-calibrated data from Y axis of accelerometer 	*/
#define NAZ 															0x04					/*	non-calibrated data from Z axis of accelerometer 	*/
#define NWX 															0x05					/*	non-calibrated data from X axis of gyroscope 	*/
#define NWY 															0x06					/*	non-calibrated data from Y axis of gyroscope 	*/
#define NWZ 															0x07					/*	non-calibrated data from Z axis of gyroscope 	*/
#define NT0 															0x08					/*	non-calibrated data from 0 channel of 12-bit adc of MCU (temperature 0)	*/
#define NT1 															0x09					/*	non-calibrated data from 1 channel of 12-bit adc of MCU (temperature 1)	*/
#define NT2 															0x0A					/*	non-calibrated data from 2 channel of 12-bit adc of MCU (temperature 2)	*/
#define NT3 															0x0B					/*	non-calibrated data from 3 channel of 12-bit adc of MCU (temperature 3)	*/
#define NT4 															0x0C					/*	non-calibrated data from 4 channel of 12-bit adc of MCU (temperature 4)	*/
#define NT5 															0x0D					/*	non-calibrated data from 5 channel of 12-bit adc of MCU (temperature 5)	*/
#define NT6 															0x0E					/*	non-calibrated data from 6 channel of 12-bit adc of MCU (temperature 6)	*/
#define NT7 															0x0F					/*	non-calibrated data from 7 channel of 12-bit adc of MCU (temperature 7)	*/
#define NT8 															0x10					/*	non-calibrated data from 8 channel of 12-bit adc of MCU (temperature 8)	*/
#define NT9 															0x11					/*	non-calibrated data from 9 channel of 12-bit adc of MCU (temperature 9)	*/
#define AX 																0x12					/*	calibrated data from X axis of accelerometer 	*/
#define AY 																0x13					/*	calibrated data from Y axis of accelerometer 	*/
#define AZ 																0x14					/*	calibrated data from Z axis of accelerometer 	*/
#define WX 																0x15					/*	calibrated data from X axis of gyroscope 	*/
#define WY 																0x16					/*	calibrated data from X axis of gyroscope	*/
#define WZ 																0x17					/*	calibrated data from X axis of gyroscope	*/
#define T0 																0x18					/*	calibrated data from 0 channel of 12-bit adc of MCU (temperature 0)	*/
#define T1 																0x19					/*	calibrated data from 1 channel of 12-bit adc of MCU (temperature 1)	*/
#define T2																0x1A					/*	calibrated data from 2 channel of 12-bit adc of MCU (temperature 2)	*/
#define T3 																0x1B					/*	calibrated data from 3 channel of 12-bit adc of MCU (temperature 3)	*/
#define T4 																0x1C					/*	calibrated data from 4 channel of 12-bit adc of MCU (temperature 4)	*/
#define T5 																0x1D					/*	calibrated data from 5 channel of 12-bit adc of MCU (temperature 5)	*/
#define T6 																0x1E					/*	calibrated data from 6 channel of 12-bit adc of MCU (temperature 6)	*/
#define T7 																0x1F					/*	calibrated data from 7 channel of 12-bit adc of MCU (temperature 7)	*/
#define T8 																0x20					/*	calibrated data from 8 channel of 12-bit adc of MCU (temperature 8)	*/
#define T9 																0x21					/*	calibrated data from 9 channel of 12-bit adc of MCU (temperature 9)	*/
#define ALPHA 														    0x22					/*	alpha angle of inclinometer	*/
#define BETA 															0x23					/*	beta angle of inclinometer	*/
#define PITCH 														    0x24					/*	pitch angle of euler orientation system	*/
#define ROLL 															0x25					/*	roll angle of euler orientation system	*/
#define YAW 															0x26					/*	yaw angle of euler orientation system	*/
#define Q0 																0x27					/*	scalar part of orientation quaternion	*/
#define Q1 																0x28					/*	x part of orientation quaternion	*/
#define Q2 																0x29					/*	y part of orientation quaternion	*/
#define Q3 																0x2A					/*	z part of orientation quaternion	*/
#define X 																0x2B					/*	position x	*/
#define Y 																0x2C					/*	position y	*/
#define Z 																0x2D					/*	position z	*/
#define VX 																0x2E					/*	linear velocity x	*/
#define VY 																0x2F					/*	linear velocity y	*/
#define VZ 																0x30					/*	linear velocity z	*/
#define IWX 															0x31					/*	integrated angle from rate of x axis of gyro	*/
#define IWY 															0x32					/*	integrated angle from rate of y axis of gyro	*/
#define IWZ 															0x33					/*	integrated angle from rate of z axis of gyro	*/
#define YAW_NOPH													    0x34					/*	yaw angle without phase delay (when use_phase_corr = 1)*/
#define PITCH_NOPH												        0x35					/*	pitch angle without phase delay (when use_phase_corr = 1)*/
#define ROLL_NOPH													    0x36					/*	roll angle without phase delay (when use_phase_corr = 1)*/
#define ALG_INT_LAT_NOPH									            0x37					/*	latitude calculated using BINS in codes (to convert into radians multiply 2*pi/(2^32))  (when use_phase_corr = 1)*/
#define ALG_INT_LON_NOPH									            0x38					/*	longitude calculated using BINS in codes (to convert into radians multiply 2*pi/(2^32))  (when use_phase_corr = 1)*/
#define ALG_INT_ALT_NOPH									            0x39					/*	altitude calculated using BINS in m (when use_phase_corr = 1)*/
/* parameter codes 0x3A:0x3F are reserved*/
#define LAX 															0x40					/*	linear acceleration x	*/
#define LAY 															0x41					/*	linear acceleration y	*/
#define LAZ 															0x42					/*	linear acceleration z	*/
#define AZIMUTH 												    	0x43					/*	azimuth angle (when using GNSS)	*/
#define UTC_TIME 												    	0x44					/*	Coordinated Universal Time (when using GNSS)	*/
#define LAT 															0x45					/*	latitude (when using GNSS)	*/
#define LON 															0x46					/*	longitude (when using GNSS)	*/
#define ALT 															0x47					/*	altitude (when using GNSS)	*/
#define GNSS_STATUS 											        0x48					/*	state of GNSS receiver	*/
#define GNSS_TDOP 												        0x49					/*	geometry factor of GNSS receiver	*/
#define GNSS_HDOP 												        0x4A					/*	geometry factor of GNSS receiver	*/
#define GNSS_VDOP 												        0x4B					/*	geometry factor of GNSS receiver	*/
#define GNSS_VEL 												    	0x4C					/*	horizontal velocity calculated using GNSS	*/
#define GNSS_YAW 												    	0x4D					/*	yaw angle calculated using GNSS	*/
#define GNSS_ALT_VEL 										        	0x4E					/*	verical velocity calculated using GNSS	*/
#define GNSS_SAT_NUM 										            0x4F					/*	number of sattelites using to calculate GNSS parameters	*/
#define MX	 															0x50					/*	magnetometer data x	*/
#define MY 																0x51					/*	magnetometer data y	*/
#define MZ 																0x52					/*	magnetometer data z	*/
#define GNSS_LAT_VEL 											        0x53					/*	velocity on latitude (using GNSS)	*/
#define GNSS_LON_VEL 											        0x54					/*	velocity on longitude (using GNSS)	*/
#define GNSS_SIG_LAT 											        0x55					/*	STD of latitude data	*/
#define GNSS_SIG_LON 											        0x56					/*	STD of longitude data	*/
#define GNSS_SIG_ALT 										        	0x57					/*	STD of altitude data	*/
#define GNSS_SIG_VLAT 										            0x58					/*	STD of velocity on latitude	*/
#define GNSS_SIG_VLON 										            0x59					/*	STD of velocity on longitude	*/
#define GNSS_SIG_VALT 										            0x5A					/*	STD of velocity on altitude	*/
#define ALG_INT_LAT												        0X5B					/*	latitude, calculated by BINS in codes (to convert into radians multiply 2*pi/(2^32))	*/
#define ALG_INT_LON												        0X5C					/*	longitude, calculated by BINS in codes (to convert into radians multiply 2*pi/(2^32))	*/
#define ALG_ALT														    0X5D					/*	altitude, calculated by BINS in float32	*/
#define GNSS_INT_LAT											        0X5E					/*	latitude from GNSS in codes (to convert into radians multiply 2*pi/(2^32))	*/
#define GNSS_INT_LON											        0X5F					/*	longitude from GNSS in codes (to convert into radians multiply 2*pi/(2^32))	*/
#define GNSS_INT_ALT											        0X60					/*	altitude from GNSS in codes (to convert into radians multiply 2*pi/(2^32))	*/
#define BAROMETER_ADC											        0X61					/*	Barometer data in codes	*/
#define ALG_VAR_X													    0X62					/*	variance of position error of X axis in m2	*/
#define ALG_VAR_Y													    0X63					/*	variance of position error of Y axis in m2	*/
#define ALG_VAR_Z													    0X64					/*	variance of position error of Z axis in m2	*/
#define ALG_VAR_VX												        0X65					/*	variance of velocity error of X axis in (m/s)^2	*/
#define ALG_VAR_VY												        0X66					/*	variance of velocity error of Y axis in (m/s)^2	*/
#define ALG_VAR_VZ												        0X67					/*	variance of velocity error of Z axis in (m/s)^2	*/
#define ALG_VAR_PSI												        0X68					/*	variance of orientation error of yaw axis in rad^2	*/
#define ALG_VAR_THETA											        0X69					/*	variance of orientation error of pitch axis in rad^2	*/
#define ALG_VAR_PHI												        0X6A					/*	variance of orientation error of roll axis in rad^2	*/
#define GPS_INT_X												        0X6B					/*	X axis in ECEF coordinates in integer value (to convert into cm multiply 2*pi/(2^32))	*/
#define GPS_INT_Y												        0X6C					/*	Y axis in ECEF coordinates in integer value (to convert into cm multiply 2*pi/(2^32))	*/
#define GPS_INT_Z												        0X6D					/*	Z axis in ECEF coordinates in integer value (to convert into cm multiply 2*pi/(2^32))	*/
/**
  * @}
  */


/** 
  * @brief  Packet 0x26 to request parameters of custom packet (send only) 
  */
typedef struct __GetCustomDataParam {}GetCustomDataParam;



/** 
  * @brief  Packet 0x27 with list of parameters of custom packet (send/receive) 
  */
typedef struct __CustomDataParam
{
	uint8_t num;                            /*	number of parameters that device sends in custom packet	*/
	uint8_t param[63];                      /*	type of parameter N from list of 'custom_data_parameters'	*/
}CustomDataParam;




/*----------------------------------------------------------------------------DATA_REQUEST-------------------------------------------------------------------------------------*/

#ifndef  __GET_DATA_PACKET__
#define __GET_DATA_PACKET__

/** @defgroup REQUEST_PACKET_CODE
  * @brief    packet codes for "type" field of request data packet
  * @{
  */ 
#define GKV_DATA_REQUEST 								0x17								/*	type of empty packet (with length = 0) to request data packet in "by request" mode	*/  /*!!!!!!!!!*/
/**
  * @}
  */



/** 
  * @brief  Packet 0x17 is used to request data (send only)
  */
typedef struct __GetData {}GetData;

#endif


#endif