/*
 *		Created on: 27/07/2021
 *      Author: Valerio Mazzoni
 *      Company: OMD
 *
 *      Version: 1.0.0
 *
 *      Changelog:
 *      	- 27/07/2021 - 1.0.0: start of the source code writing.
 *
 */
#ifndef _COMMON_DEF_H_
#define _COMMON_DEF_H_

#include "stm32f4xx_hal.h"
#include "stdbool.h"

//--------------------------------------
//---------- GENERAL VALUES ------------
//--------------------------------------
#define ENABLE 1
#define DISABLE 0

#define ULONG_MAX 0xFFFFFFFFUL

#define BLINK_DELAY_MS 250
#define DO_PULSE_DUR_MS 20
#define SPI_READ_TIMEOUT_MS 100

#define AC1_IMU_READY 0x3
#define AC1_INIZIALIZED 0x7
#define AC1_IMU_CONFIG_MODE 0x10
#define AC1_DATA_ANALYSIS_STARTED 0x20
#define AC1_IMU_FAULT 0x100
#define AC1_DAC_FAULT 0x200

//#define USB_DEBUG

#define MAGIC_VALUE 0xBEBE

#define USER_DATA_VALID_VALUE 0xAC
//--------------------------------------
//--------------- CLASS ----------------
//--------------------------------------
typedef struct {
	bool Power;
	bool IMUInit;
	bool DACInit;
	bool Booted;
	bool IMUAnalysisON;
	bool IMUFault;
	bool DACFault;
	bool DACOutON;
} AC1_StatusWord_t;

typedef struct
{
	/* Accelerometer */
	uint8_t OMD_CONFIG_ACC_FSR;
	uint8_t OMD_CONFIG_ACC_ODR;
	uint8_t OMD_CONFIG_ACC_FILT_BW;

	/* Gyroscope */
	uint8_t OMD_CONFIG_GYRO_FSR;
	uint8_t OMD_CONFIG_GYRO_ODR;
	uint8_t OMD_CONFIG_GYRO_FILT_BW;

	/* WOM configuration */
	uint8_t OMD_CONFIG_WOM_X_TH;
	uint8_t OMD_CONFIG_WOM_Y_TH;
	uint8_t OMD_CONFIG_WOM_Z_TH;
	uint8_t OMD_CONFIG_WOM_INT_MODE;
	uint8_t OMD_CONFIG_WOM_MODE;
	uint8_t OMD_CONFIG_SMD_MODE;

	/* Validating byte */
	uint8_t OMD_CONFIG_VBYTE;
} AC1_Config_t;
//--------------------------------------
//--------------- MACRO ----------------
//--------------------------------------
#define ARRAYSIZE(x) (sizeof x/sizeof x[0])

#define _DELAY_AUTO(x)	\
do {	\
	if (osKernelRunning() == 1)	\
	{	\
		osDelay(x);\
	}	\
	else	\
	{		\
		HAL_Delay(x);\
	}		\
} while(0)
//--------------------------------------
//----------- ENUMERATORS --------------
//--------------------------------------
typedef enum
{
	AC1_Notify_FIFOOverflow = 0x01,
	AC1_Notify_FIFOThsReached = 0x02,
	AC1_Notify_NewDataAvaiable = 0x04,
	AC1_Notify_TelemetryEnChange = 0x08,
	AC1_Notify_AnalysisON = 0x10,
	AC1_Notify_AnalysisOFF = 0x20,
	AC1_Notify_SaveConfig = 0x40,
	AC1_Notify_INT1Event = 0x80,
	AC1_Notify_INT2Event = 0x100,
} AC1_TaskNotify_t;

typedef enum
{
	AC1_Notify_FIFOOverflow_Bit = 0,
	AC1_Notify_FIFOThsReached_Bit = 1,
	AC1_Notify_NewDataAvaiable_Bit = 2,
	AC1_Notify_TelemetryEnChange_Bit = 3,
	AC1_Notify_AnalysisON_Bit = 4,
	AC1_Notify_AnalysisOFF_Bit = 5,
	AC1_Notify_SaveConfig_Bit = 6,
	AC1_Notify_INT1Event_Bit = 7,
	AC1_Notify_INT2Event_Bit = 8,
} AC1_TaskNotifyBits_t;

#endif
