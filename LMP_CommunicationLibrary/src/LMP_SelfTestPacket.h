#pragma once
/**
  ******************************************************************************
  * @file    GKV_SelfTestPacket.h
  * @author  Alex Galkin,  <info@mp-lab.ru>
  * @brief   GKV-10 structures module for packets with SelfTest parameters and results
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
#ifndef __GKV_SELFTEST_PACKET_H__
#define __GKV_SELFTEST_PACKET_H__


/** @defgroup SELFTEST_PACKET_CODES
  * @brief    packet codes for "type" field of selftest packet
  * @{
  */ 
#define GKV_SELFTEST_REQUEST_PACKET			            0x28								/*	type of packet (with length = 8) to start device selftest	*/
#define GKV_SELFTEST_RESULT_PACKET			 	        0x29								/*	type of packet with result of device selftest (gyro and accelerometers)	*/
/**
  * @}
  */


/** @defgroup SELFTEST_COMMANDS
  * @brief 		settings for "selftest_commands" field of request packet 0x28 (command to start one of device selftest depending on "command" and "parameters" fields)
  * @{
  */
#define GKV_SELFTEST_OFF 											        0x00            /*  individual test off     */
#define GKV_REQ_SELFTEST_RESULT 							                0x01            /*  request the current state (results of the last test)    */
#define GKV_START_SELFTEST 										            0x02            /*  self-test starts, upon completion, GKV sends frame 41 with results  */
#define GKV_START_CUSTOM_TEST 								                0x03            /*  run an individual test according to the code    */
#define GKV_CUSTOM_TEST_OFF 									            0x04            /*  individual test off */
/**
  * @}
  */



/** @defgroup SELFTEST_CODES
  * @brief 		settings for "selftest_params" field of request packet 0x28 (command to start one of device selftest depending on "command" and "parameters" fields)
  * @{
  */
#define GKV_GYRO_POSITIVE_SELFTEST 						                    0x00                    /*  run test of rate sensor positive axis direction  */
#define GKV_GYRO_NEGATIVE_SELFTEST 						                    0x01                    /*  run test of rate sensor negative axis direction  */
#define GKV_ACCEL_X_SELFTEST 								            	0x02                    /*  run test of accelerometer X axis */
#define GKV_ACCEL_Y_SELFTEST 								            	0x03                    /*  run test of accelerometer Y axis */
#define GKV_ACCEL_Z_SELFTEST 									            0x04                    /*  run test of accelerometer Z axis */
/**
  * @}
  */


/** 
  * @brief  Packet 0x28 is used as command to start one of device selftest depending on "cmd" and "mode" fields (send only) 
  */
typedef struct __GKV_SelfTest
{
	uint32_t cmd;           					                /*	code of selftest settings	*/
	uint32_t mode;          					                /*	code for selection selftest type	*/
}GKV_SelfTest;


/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/** @defgroup GYRO_SELFTEST_RESULTS
  * @brief 		parameters for  "gyro_selftest_result" field of packet 0x29 (results of device selftest)
  * @{
  */
#define GKV_GYRO_Z_ERROR 											        1<<2                    /*  selftest detected rate sensor X axis error  */
#define GKV_GYRO_Y_ERROR 											        1<<1                    /*  selftest detected rate sensor Y axis error  */
#define GKV_GYRO_X_ERROR 											        1                       /*  selftest detected rate sensor Z axis error  */
/**
  * @}
  */



/** @defgroup ACCEL_SELFTEST_RESULTS
  * @brief 		parameters for  "accel_selftest_result" field of packet 0x29 (results of device selftest)
  * @{
  */
#define GKV_ACCEL_Z_ERROR 										            1<<2                /*  selftest detected accelerometer X axis error  */
#define GKV_ACCEL_Y_ERROR 										            1<<1                /*  selftest detected accelerometer Y axis error  */
#define GKV_ACCEL_X_ERROR 										            1                   /*  selftest detected accelerometer Y axis error  */
/**
  * @}
  */



/** 
  * @brief 	Packet 0x29 with results of device selftest (receive only)
  */
typedef struct __GKV_SelfTestResult
{
	uint32_t gyro_result;                                       /*	detected errors in accelerometer test	*/
	uint32_t acc_result;                                        /*	detected errors in rate sensor test	*/
}GKV_SelfTestResult;




#endif
