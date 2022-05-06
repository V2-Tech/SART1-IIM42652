/*
 *		Created on: 27/07/2021
 *      Author: Valerio Mazzoni
 *      Company: OMD
 *
 *      Version: 1.0.0
 *
 *      Changelog:
 *      	- 27/07/2021 - 1.0.0: start of the source code writing.
 */

#ifndef _IIM42652_H_
#define _IIM42652_H_

//--------------------------------------
// ------------ LIBRARYS ---------------
//--------------------------------------
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "arm_math.h"
#include <stdbool.h>
#include "RingBuffer.h"
#include "stream_buffer.h"
#include "cobstranscoder.h"
#include "USBCommProtocol_def.h"
#include "common_def.h"
#include "vc_vector.h"

//--------------------------------------
// ------------ REGISTERS --------------
//--------------------------------------

/*
 * User Bank 0
 */
#define REG_DEVICE_CONFIG 0x11
#define REG_DRIVE_CONFIG 0x13
#define REG_INT_CONFIG 0x14
#define REG_FIFO_CONFIG 0x16
#define	REG_TEMP_DATA1_UI 0x1D
#define	REG_ACCEL_DATA_X1_UI 0x1F
#define	REG_ACCEL_DATA_Y1_UI 0x21
#define REG_ACCEL_DATA_Z1_UI 0x23
#define REG_GYRO_DATA_X1_UI 0x25
#define	REG_GYRO_DATA_Y1_UI 0x27
#define	REG_GYRO_DATA_Z1_UI 0x29
#define	REG_TMST_FSYNCH 0x2B
#define REG_INT_STATUS 0x2D
#define REG_FIFO_COUNTH 0x2E
#define REG_FIFO_COUNTL 0x2F
#define	REG_FIFO_DATA 0x30
#define REG_APEX_DATA0 0x31
#define	REG_APEX_DATA1 0x32
#define	REG_APEX_DATA2 0x33
#define	REG_APEX_DATA3 0x34
#define	REG_APEX_DATA4 0x35
#define	REG_APEX_DATA5 0x36
#define	REG_INT_STATUS2 0x37
#define	REG_INT_STATUS3 0x38
#define	REG_SIGNAL_PATH_RESET 0x4B
#define REG_INTF_CONFIG0 0x4C
#define REG_INTF_CONFIG1 0x4D
#define	REG_PWR_MGMT0 0x4E
#define	REG_GYRO_CONFIG0 0x4F
#define	REG_ACCEL_CONFIG0 0x50
#define REG_GYRO_CONFIG1 0x51
#define	REG_GYRO_ACCEL_CONFIG0 0x52
#define REG_ACCEL_CONFIG1 0x53
#define REG_TMST_CONFIG 0x54
#define REG_APEX_CONFIG0 0x56
#define REG_SMD_CONFIG 0x57
#define REG_FIFO_CONFIG1 0x5F
#define REG_FIFO_CONFIG2 0x60
#define REG_FIFO_CONFIG3 0x61
#define REG_FSYNC_CONFIG 0x62
#define REG_INT_CONFIG0 0x63
#define REG_INT_CONFIG1 0x64
#define REG_INT_SOURCE0 0x65
#define REG_INT_SOURCE1 0x66
#define REG_INT_SOURCE3 0x68
#define REG_INT_SOURCE4 0x69
#define REG_FIFO_LOST_PKT0 0x6C
#define REG_FIFO_LOST_PKT1 0x6D
#define REG_SELF_TEST_CONFIG 0x70
#define REG_WHO_AM_I 0x75
#define REG_REG_BANK_SEL 0x76

/*
 * User Bank 1
 */
#define REG_SENSOR_CONFIG0 0x03
#define REG_GYRO_CONFIG_STATIC2 0x0B
#define REG_GYRO_CONFIG_STATIC3 0x0C
#define REG_GYRO_CONFIG_STATIC4 0x0D
#define REG_GYRO_CONFIG_STATIC5 0x0E
#define REG_GYRO_CONFIG_STATIC6 0x0F
#define REG_GYRO_CONFIG_STATIC7 0x10
#define REG_GYRO_CONFIG_STATIC8 0x11
#define REG_GYRO_CONFIG_STATIC9 0x12
#define REG_GYRO_CONFIG_STATIC10 0x13
#define REG_XG_ST_DATA 0x5F
#define	REG_YG_ST_DATA 0x5F
#define REG_ZG_ST_DATA 0x5F
#define REG_TMSTVAL0 0x62
#define REG_TMSTVAL1 0x63
#define REG_TMSTVAL2 0x64
#define REG_INTF_CONFIG4 0x7A
#define REG_INTF_CONFIG5 0x7B
#define REG_INTF_CONFIG6 0x7C

/*
 * User Bank 2
 */
#define REG_ACCEL_CONFIG_STATIC2 0x03
#define REG_ACCEL_CONFIG_STATIC3 0x04
#define REG_ACCEL_CONFIG_STATIC4 0x05
#define REG_XA_ST_DATA 0x3B
#define REG_YA_ST_DATA 0x3C
#define REG_ZA_ST_DATA 0x3D

/*
 * User Bank 3
 */
#define REG_PU_PD_CONFIG1 0x06
#define REG_PU_PD_CONFIG2 0x0E

/*
 * User Bank 4
 */
#define REG_FDR_CONFIG 0x09
#define REG_APEX_CONFIG1 0x40
#define REG_APEX_CONFIG2 0x41
#define REG_APEX_CONFIG3 0x42
#define REG_APEX_CONFIG4 0x43
#define REG_APEX_CONFIG5 0x44
#define REG_APEX_CONFIG6 0x45
#define REG_APEX_CONFIG7 0x46
#define REG_APEX_CONFIG8 0x47
#define REG_APEX_CONFIG9 0x48
#define REG_APEX_CONFIG10 0x49
#define REG_ACCEL_WOM_X_THR 0x4A
#define REG_ACCEL_WOM_Y_THR 0x4B
#define REG_ACCEL_WOM_Z_THR 0x4C
#define REG_INT_SOURCE6 0x4D
#define REG_INT_SOURCE7  0x4E
#define REG_INT_SOURCE8  0x4F
#define REG_INT_SOURCE9  0x50
#define REG_INT_SOURCE10  0x51
#define REG_OFFSET_USER0 0x77
#define REG_OFFSET_USER1 0x78
#define REG_OFFSET_USER2 0x79
#define REG_OFFSET_USER3 0x7A
#define REG_OFFSET_USER4 0x7B
#define REG_OFFSET_USER5 0x7C
#define REG_OFFSET_USER6 0x7D
#define REG_OFFSET_USER7 0x7E
#define REG_OFFSET_USER8 0x7F

//--------------------------------------
// --------- DEFINED VALUES ------------
//--------------------------------------
#define IIM42652_WHO_AM_I_ID 0x6F

#define IIM42652_SOFTRESET 0x01

#define ACCEL_DATA_SIZE               6
#define GYRO_DATA_SIZE                6
#define TEMP_DATA_SIZE                2

#define FIFO_HEADER_SIZE              1
#define FIFO_ACCEL_DATA_SIZE          ACCEL_DATA_SIZE
#define FIFO_GYRO_DATA_SIZE           GYRO_DATA_SIZE
#define FIFO_TEMP_DATA_SIZE           1
#define FIFO_TS_FSYNC_SIZE            2
#define FIFO_TEMP_HIGH_RES_SIZE       1
#define FIFO_ACCEL_GYRO_HIGH_RES_SIZE 3

#define FIFO_16BYTES_PACKET_SIZE      (FIFO_HEADER_SIZE + FIFO_ACCEL_DATA_SIZE + FIFO_GYRO_DATA_SIZE + FIFO_TEMP_DATA_SIZE + FIFO_TS_FSYNC_SIZE)
#define FIFO_20BYTES_PACKET_SIZE      (FIFO_HEADER_SIZE + FIFO_ACCEL_DATA_SIZE + FIFO_GYRO_DATA_SIZE + FIFO_TEMP_DATA_SIZE + FIFO_TS_FSYNC_SIZE +\
                                       FIFO_TEMP_HIGH_RES_SIZE + FIFO_ACCEL_GYRO_HIGH_RES_SIZE)

#define FIFO_HEADER_ODR_ACCEL         0x01
#define FIFO_HEADER_ODR_GYRO          0x02
#define FIFO_HEADER_FSYNC             0x04
#define FIFO_HEADER_TMST              0x08
#define FIFO_HEADER_HEADER_20         0x10
#define FIFO_HEADER_GYRO              0x20
#define FIFO_HEADER_ACC               0x40
#define FIFO_HEADER_MSG               0x80

#define IIM42652_FIFO_MIRRORING_SIZE (2048 + 16)
#define IIM42652_FIFO_MIRRORING_PACKETS_SIZE 128

#define FIFO_INVALID_SAMPLE -32768

#define IMU_FFT_SIZE 256U

#define BIG_ENDIAN_DATA 1
#define LITTLE_ENDIAN_DATA 0

#define STREAM_BUFFER_SIZE_BYTES FIFO_16BYTES_PACKET_SIZE*128
#define IMU_OUTPUT_DATA_QUEUE_SIZE 5*IIM42652_FIFO_MIRRORING_PACKETS_SIZE
//--------------------------------------
// ------------- MACROS ----------------
//--------------------------------------
/*
#define __IMU_TURN_ON(imu) \
do { \
	IIM42652_SetAccelMode(imu,IMU_AccelMode_LN); \
	IIM42652_SetGyroMode(imu,IMU_GyroMode_LN); \
	IIM42652_SetTempMode(imu,ENABLE); \
} while(0)

#define __IMU_TURN_OFF(imu) \
do { \
	IIM42652_SetAccelMode(imu,IMU_AccelMode_OFF); \
	IIM42652_SetGyroMode(imu,IMU_GyroMode_OFF); \
	IIM42652_SetTempMode(imu, DISABLE); \
} while(0)
*/

#define TO_MASK(a) (1U << (unsigned)(a))

//--------------------------------------
// ------------ VARIABLES --------------
//--------------------------------------

/**
 * @brief Description of the content of the single 16-byte FIFO packet
 * */
typedef struct {
	/* Header */
	uint8_t m_FIFO_Header;

	/* Accel */
	int16_t m_FIFO_Accel_X;
	int16_t m_FIFO_Accel_Y;
	int16_t m_FIFO_Accel_Z;

	/* Gyro */
	int16_t m_FIFO_Gyro_X;
	int16_t m_FIFO_Gyro_Y;
	int16_t m_FIFO_Gyro_Z;

	/* Temperature */
	int8_t m_FIFO_Temp;

	/* Timestamp */
	uint16_t m_FIFO_TimeStamp;

	/* APEX INT */
	uint8_t m_APEX_INT;

} IIM42652_FIFO_Packet16_t; //16Byte

/*
 * @brief Struct used for log IMU datat over USB COM
 */
typedef struct {
	/* Accel */
	float Accel_X_mps2;
	float Accel_Y_mps2;
	float Accel_Z_mps2;

	/* Gyro */
	float Gyro_X_rps;
	float Gyro_Y_rps;
	float Gyro_Z_rps;

	/* Temperature */
	float Temp_C;

	/* Timestamp */
	uint16_t TimeStamp_ODR;

} IMU_Data_Packet_t;

/*
 * @brief Description of the content of the FIFO header
 */
typedef union
{
	unsigned char Byte;
	struct
	{
		unsigned char gyro_odr_different : 1;
		unsigned char accel_odr_different : 1;
		unsigned char fsync_bit : 1;
		unsigned char timestamp_bit : 1;
		unsigned char twentybits_bit : 1;
		unsigned char gyro_bit : 1;
		unsigned char accel_bit : 1;
		unsigned char msg_bit : 1;
	}bits;
} fifo_header_t;

typedef struct {
	/* SPI */
	SPI_HandleTypeDef *m_spiHandle;
	GPIO_TypeDef 	  *m_csImuPinBank;
	uint16_t 		   m_csImuPin;
	osSemaphoreId m_spiSemaphoreHandle;
	uint8_t bSingleShot;

	/* DMA */
	volatile uint8_t rxBufDMA[2];
	uint8_t txBufDMA[IIM42652_FIFO_MIRRORING_SIZE];

	/* Telemetry report status */
	uint8_t TelemetryON;

	/* FIFO */
	volatile uint8_t m_FIFO_reading;
	volatile uint8_t m_FIFOData[IIM42652_FIFO_MIRRORING_SIZE];
	volatile uint8_t m_FIFO_Count;

	/* APEX */
	uint8_t m_DMPStarted;
	volatile uint8_t m_INT2State;

	/* Actual device configuration */
	uint8_t m_RegisterBank;
	float AccelScaleFactor; // LSB/g Accelerometer scale factor
	float GyroScaleFactorDegree; // LSB/°/s Gyroscope scale factor
	float GyroScaleFactorRad; // LSB/rad/s Gyroscope scale factor
	uint8_t decimationFactor; // Divider factor of telemetry data sent to USB gateway

	/* Actual measurements */
	float Acc_mps2[3];
	float Gyro_rps[3];
	float Temp_C;
	uint16_t TimeStamp_ms;

	/* Sensor configuration */
	AC1_Config_t *m_actUserConfig;
	float AccXSensitivity, AccYSensitivity, AccZSensitivity;
} IIM42652_t;

typedef struct
{
	uint8_t buffer[(FIFO_16BYTES_PACKET_SIZE-FIFO_HEADER_SIZE+1)]; //+1 for APEX interrupt state byte
} IMU_Data_Buf_t;
//--------------------------------------
// ------------ ENUMERTORS -------------
//--------------------------------------

/**
 * @brief  Parameter for accelerometer range
 */
typedef enum  {
	IIM42652_Accelerometer_16G = 0x00, /*!< Range is +- 16G */
	IIM42652_Accelerometer_8G = 0x01,
	IIM42652_Accelerometer_4G = 0x02,
	IIM42652_Accelerometer_2G = 0x03
} IIM42652_AccFSRValues;

/**
 * @brief  Parameter for gyroscope range
 */
typedef enum  {
	IIM42652_Gyroscope_2000dps = 0x00, /*!< Range is +- 2000degree/sec */
	IIM42652_Gyroscope_1000dps = 0x01,
	IIM42652_Gyroscope_500dps = 0x02,
	IIM42652_Gyroscope_250dps = 0x03,
	IIM42652_Gyroscope_125dps = 0x04,
	IIM42652_Gyroscope_62_5dps = 0x05,
	IIM42652_Gyroscope_31_25dps = 0x06,
	IIM42652_Gyroscope_15_625dps = 0x07
} IIM42652_GyroFSRValues;

/**
 * @brief  Parameter for accelerometer output data rate (ODR)
 */
typedef enum  {
	IIM42652_Accelerometer_32kHz = 0x01, /*!< ODR is 32kHz */
	IIM42652_Accelerometer_16kHz = 0x02,
	IIM42652_Accelerometer_8kHz = 0x03, /* Require LN mode */
	IIM42652_Accelerometer_4kHz = 0x04, /* Require LN mode */
	IIM42652_Accelerometer_2kHz = 0x05, /* Require LN mode */
	IIM42652_Accelerometer_1kHz = 0x06, /* Default value, require LN mode */
	IIM42652_Accelerometer_200Hz = 0x07, /* Require LN or LP mode */
	IIM42652_Accelerometer_100Hz = 0x08, /* Require LN or LP mode */
	IIM42652_Accelerometer_50Hz = 0x09, /* Require LN or LP mode */
	IIM42652_Accelerometer_25Hz = 0x0A, /* Require LN or LP mode */
	IIM42652_Accelerometer_12_5Hz = 0x0B, /* Require LN or LP mode */
	IIM42652_Accelerometer_6_25Hz = 0x0C, /* Require LP mode */
	IIM42652_Accelerometer_3_125Hz = 0x0D, /* Require LP mode */
	IIM42652_Accelerometer_1_5625Hz = 0x0E, /* Require LP mode */
	IIM42652_Accelerometer_500Hz = 0x0F /* Require LN or LP mode */
} IIM42652_AccODRValues;

/**
 * @brief  Parameter for gyroscope output data rate (ODR)
 */
typedef enum  {
	IIM42652_Gyroscope_32kHz = 0x01, /*!< ODR is 32kHz */
	IIM42652_Gyroscope_16kHz = 0x02,
	IIM42652_Gyroscope_8kHz = 0x03,
	IIM42652_Gyroscope_4kHz = 0x04,
	IIM42652_Gyroscope_2kHz = 0x05,
	IIM42652_Gyroscope_1kHz = 0x06, /* Default value */
	IIM42652_Gyroscope_200Hz = 0x07,
	IIM42652_Gyroscope_100Hz = 0x08,
	IIM42652_Gyroscope_50Hz = 0x09,
	IIM42652_Gyroscope_25Hz = 0x0A,
	IIM42652_Gyroscope_12_5Hz = 0x0B,
	IIM42652_Gyroscope_500Hz = 0x0F
} IIM42652_GyroODRValues;

/**
 * @brief  Parameter for accelerometer bandwidth
 */
typedef enum  {
	IIM42652_Accelerometer_ODR_2 = 0x00,
	IIM42652_Accelerometer_ODR_4 = 0x01, /* Default value */
	IIM42652_Accelerometer_ODR_5 = 0x02,
	IIM42652_Accelerometer_ODR_8 = 0x03,
	IIM42652_Accelerometer_ODR_10 = 0x04,
	IIM42652_Accelerometer_ODR_16 = 0x05,
	IIM42652_Accelerometer_ODR_20 = 0x06,
	IIM42652_Accelerometer_ODR_40 = 0x07,
	IIM42652_Accelerometer_ODR_LL = 0x0E,
	IIM42652_Accelerometer_ODR_LLx8 = 0x0F,
} IIM42652_AccBWValues;

/**
 * @brief  Parameter for gyroscope bandwidth
 */
typedef enum  {
	IIM42652_Gyroscope_ODR_2 = 0x00,
	IIM42652_Gyroscope_ODR_4 = 0x01, /* Default value */
	IIM42652_Gyroscope_ODR_5 = 0x02,
	IIM42652_Gyroscope_ODR_8 = 0x03,
	IIM42652_Gyroscope_ODR_10 = 0x04,
	IIM42652_Gyroscope_ODR_16 = 0x05,
	IIM42652_Gyroscope_ODR_20 = 0x06,
	IIM42652_Gyroscope_ODR_40 = 0x07,
	IIM42652_Gyroscope_ODR_LL = 0x0E,
	IIM42652_Gyroscope_ODR_LLx8 = 0x0F,
} IIM42652_GyroBWValues;

/**
 * @brief  Parameter for UI filter order
 */
typedef enum  {
	IIM42652_FilterOrder_1 = 0x00,
	IIM42652_FilterOrder_2 = 0x01, /* Default value */
	IIM42652_FilterOrder_3 = 0x02,
} IIM42652_UIFilterOrder;

/**
 * @brief  Parameter for interrupt output pin chose
 */
typedef enum  {
	IMU_Int_1 = 1,
	IMU_Int_2 = 2,
	IMU_Int_1_2 = 3
} IIM42652_InterruptPinNumber;

/**
 * @brief  Parameter for interrupt output type (Pulsed or Latched)
 */
typedef enum  {
	IMU_Int_Pulsed = 0,
	IMU_Int_Latched = 1,
} IIM42652_InterruptType;

/**
 * @brief  Parameter for interrupt output circuit (Open drain or Push-pull)
 */
typedef enum  {
	IMU_Int_OpenDrain = 0,
	IMU_Int_PushPull = 1,
} IIM42652_InterruptCircuit;

/**
 * @brief  Parameter for interrupt output polarity (Active low or high)
 */
typedef enum  {
	IMU_Int_ActiveLow = 0,
	IMU_Int_ActiveHigh = 1,
} IIM42652_InterruptPolarity;

/**
 * @brief  Parameter for interrupt output duration chose (100usec or 8usec)
 */
typedef enum  {
	IMU_IntPuls_100u = 0,
	IMU_IntPuls_8u = 1,
} IIM42652_InterruptDuration;

/**
 * @brief  Parameter for interrupt output source event chose
 */
typedef enum  {
	/* INT_SOURCE0 or INT_SOURCE3 */
	IMU_IntSource_UI_FSYNC = 6,
	IMU_IntSource_PLL_RDY = 5,
	IMU_IntSource_RESET_DONE = 4,
	IMU_IntSource_UI_Data_RDY = 3,
	IMU_IntSource_FIFO_THS = 2,
	IMU_IntSource_FIFO_FULL = 1,
	IMU_IntSource_UI_AGC_RDY = 0,
	/* INT_SOURCE1 or INT_SOURCE4 */
	IMU_IntSource_SMD = 11,
	IMU_IntSource_WOM_Z = 10,
	IMU_IntSource_WOM_Y = 9,
	IMU_IntSource_WOM_X = 8
} IIM42652_InterruptSource;

/**
 * @brief  Parameter for Wake On Motion detection interrupt mode (OR or AND with all set accel threshold)
 */
typedef enum  {
	IMU_WOMIntMode_OR = 0,
	IMU_WOMIntMode_AND = 1,
} IIM42652_WOMInterruptMode;

/**
 * @brief  Parameter for Wake On Motion mode (compare with: initial or previous sample)
 */
typedef enum  {
	IMU_WOMMode_Initial = 0,
	IMU_WOMMode_Previous = 1,
} IIM42652_WOMMode;

/**
 * @brief  Parameter for Wake On Motion detection interrupt mode (OR or AND with all set accel threshold)
 */
typedef enum  {
	IMU_SMDMode_Disabled = 0,
	IMU_SMDMode_WOM = 1,
	IMU_SMDMode_WOM_short = 2, /* two WOM are detected 1 sec apart */
	IMU_SMDMode_WOM_long = 3 /* two WOM are detected 3 sec apart */
} IIM42652_SMDMode;

/**
 * @brief  Parameter for Accelerometer enabling mode
 */
typedef enum  {
	IMU_AccelMode_OFF = 0,
	IMU_AccelMode_LP = 2, /* Low power mode */
	IMU_AccelMode_LN = 3 /* Low noise mode */
} IIM42652_AccelMode;

/**
 * @brief  Parameter for Gyroscope enabling mode
 */
typedef enum  {
	IMU_GyroMode_OFF = 0,
	IMU_GyroMode_Standby = 1,
	IMU_GyroMode_LN = 3 /* Low noise mode */
} IIM42652_GyroMode;

/**
 * @brief  Parameter for select the FIFO steaming mode
 */
typedef enum  {
	IMU_FIFOMode_Bypass = 0,
	IMU_FIFOMode_Steam_Continuos = 1,
	IMU_FIFOMode_Steam_StopOnFull = 3
} IIM42652_FIFOMode;

/**
 * @brief  Parameter for select the FIFO counting mode
 */
typedef enum  {
	IMU_FIFOCountMode_Bytes = 0,
	IMU_FIFOCountMode_Records = 1 //(1 record = 16 bytes for header + gyro + accel + temp sensor data + time stamp, or 8 bytes for header + gyro/accel + temp sensor data)
} IIM42652_FIFOCountMode;

/**
 * @brief  Parameter for select the desired register bank
 */
typedef enum  {
	IMU_RegBank_0 = 0,
	IMU_RegBank_1 = 1,
	IMU_RegBank_2 = 2,
	IMU_RegBank_3 = 3,
	IMU_RegBank_4 = 4
} IIM42652_RegBankValue;

/**
 * @brief  Parameter for decrypt interrupt status register
 */
typedef enum  {
	IMU_IntStatus_AGC_Ready = 0,
	IMU_IntStatus_FIFO_Full = 1,
	IMU_IntStatus_FIFO_Ths = 2,
	IMU_IntStatus_Data_Ready = 3,
	IMU_IntStatus_Reset_Done = 4,
	IMU_IntStatus_PLL_Ready = 5,
	IMU_IntStatus_UiFSync = 6
} IIM42652_IntStatus_Bits;

/**
 * @brief  Parameter for decrypt interrupt status register 2
 */
typedef enum  {
	IMU_IntStatus_SMD = 0,
	IMU_IntStatus_WOM_X = 1,
	IMU_IntStatus_WOM_Y = 2,
	IMU_IntStatus_WOM_Z = 3,
} IIM42652_IntStatus2_Bits;

/**
 * @brief  Parameter for decrypt interrupt status register 3
 */
typedef enum  {
	IMU_IntStatus_TAP = 0,
	IMU_IntStatus_FF = 1,
	IMU_IntStatus_TILT = 3,
	IMU_IntStatus_STEP_OVF = 4,
	IMU_IntStatus_STEP = 5,
} IIM42652_IntStatus3_Bits;

/**
 * @brief  Parameter for select the DMP ODR
 */
typedef enum  {
	IIM42652_DMP_25Hz = 0x00,
	IIM42652_DMP_500Hz = 0x01,
	IIM42652_DMP_50Hz = 0x02, /* Default */
	IIM42652_DMP_100Hz = 0x03,
} IIM42652_DMPODRValueS;

//--------------------------------------
// ---------- CLASS METHODS ------------
//--------------------------------------
/* Initialization */
uint8_t IIM42652_Init(IIM42652_t *imu, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csImuPinBank, uint16_t csImuPin, AC1_Config_t *actConfig);
uint8_t IIM42652_InitBusGuard(IIM42652_t *imu, osSemaphoreId spiSemaphoreHandler);
uint8_t IIM42652_CheckCommunication(IIM42652_t *imu);
uint8_t IIM42652_SetBasicConfig(IIM42652_t *imu);
uint8_t IIM42652_SetUserConfig(IIM42652_t *imu);

/* Communication */
//Polling
uint8_t IIM42652_ReadRegister(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank, uint8_t *data);
uint8_t IIM42652_ReadRegisterBits(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank, uint8_t start_bit, uint8_t len, uint8_t* data);
uint8_t IIM42652_WriteRegister(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank, uint8_t data);
uint8_t IIM42652_WriteRegisterBits(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank, uint8_t start_bit, uint8_t len, uint8_t data);
uint8_t IIM42652_ReadMultiRegisters(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank, uint8_t *pRxBuf, uint16_t byteQuantity);
uint8_t IIM42652_WriteMultiRegisters(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank, uint8_t *data, uint16_t byteQuantity);

//DMA
uint8_t IIM42652_ReadRegisterDMA(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank);
uint8_t IIM42652_WriteRegisterDMA(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank, uint8_t data);
void IIM42652_ReadRegisterDMA_Complete(IIM42652_t *imu);
void IIM42652_WriteRegisterDMA_Complete(IIM42652_t *imu);
uint8_t IIM42652_ReadMultiRegisters_DMA(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank, volatile uint8_t *pRxBuf, uint16_t byteQuantity);
uint8_t IIM42652_ReadMultiRegisters_IT(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank, volatile uint8_t *pRxBuf, uint16_t byteQuantity);

/* Configuration */
uint8_t __IMU_TURN_ON(IIM42652_t *imu);
uint8_t __IMU_LP(IIM42652_t *imu);
uint8_t __IMU_TURN_OFF(IIM42652_t *imu);
void __IMU_INT1_ON();
void __IMU_INT1_OFF();
void __IMU_INT2_ON();
void __IMU_INT2_OFF();
uint8_t IIM42652_SoftReset(IIM42652_t *imu);

uint8_t IIM42652_SetRegisterBank(IIM42652_t *imu, IIM42652_RegBankValue RegisterBankNumber);

uint8_t IIM42652_SetIntType (IIM42652_t *imu, IIM42652_InterruptPinNumber InterruptPinNumber, IIM42652_InterruptType type);
uint8_t IIM42652_SetIntCircuit (IIM42652_t *imu, IIM42652_InterruptPinNumber InterruptPinNumber, IIM42652_InterruptCircuit type);
uint8_t IIM42652_SetIntPolarity (IIM42652_t *imu, IIM42652_InterruptPinNumber InterruptPinNumber, IIM42652_InterruptPolarity type);

uint8_t IIM42652_SetFIFOMode(IIM42652_t *imu, IIM42652_FIFOMode mode);
uint8_t IIM42652_SetFIFOCountMode(IIM42652_t *imu, IIM42652_FIFOCountMode mode);
uint8_t IIM42652_SetFIFOHoldLastValidData_Enable(IIM42652_t *imu, uint8_t EnableState);
uint8_t IIM42652_SetFIFOIntThreshold_Enable(IIM42652_t *imu, uint8_t EnableState);
uint8_t IIM42652_SetFIFOStore_Accel(IIM42652_t *imu, uint8_t EnableState);
uint8_t IIM42652_SetFIFOStore_Gyro(IIM42652_t *imu, uint8_t EnableState);
uint8_t IIM42652_SetFIFOStore_Temperature(IIM42652_t *imu, uint8_t EnableState);
uint8_t IIM42652_SetFIFOStore_FsyncTimestamp(IIM42652_t *imu, uint8_t EnableState);
uint8_t IIM42652_SetFIFOIntThreshold_Point(IIM42652_t *imu, uint16_t threshold);

uint8_t IIM42652_SetIntPulsDuration(IIM42652_t *imu, IIM42652_InterruptDuration duration);
uint8_t IIM42652_SetIntPulsDeassert(IIM42652_t *imu, uint8_t EnableState);
uint8_t IIM42652_SetIntPulsAsyncReset(IIM42652_t *imu, uint8_t EnableState);

uint8_t IIM42652_SetIntSource(IIM42652_t *imu, IIM42652_InterruptPinNumber InterruptPinNumber, IIM42652_InterruptSource source, uint8_t EnableState);

uint8_t IIM42652_SetWomXTh(IIM42652_t *imu, uint8_t threshold);
uint8_t IIM42652_GetWomXTh(IIM42652_t *imu, uint8_t* value);
uint8_t IIM42652_SetWomYTh(IIM42652_t *imu, uint8_t threshold);
uint8_t IIM42652_GetWomYTh(IIM42652_t *imu, uint8_t* value);
uint8_t IIM42652_SetWomZTh(IIM42652_t *imu, uint8_t threshold);
uint8_t IIM42652_GetWomZTh(IIM42652_t *imu, uint8_t* value);
uint8_t IIM42652_SetWOMIntMode(IIM42652_t *imu, IIM42652_WOMInterruptMode mode);
uint8_t IIM42652_GetWOMIntMode(IIM42652_t *imu, uint8_t* value);
uint8_t IIM42652_SetWOMMode(IIM42652_t *imu, IIM42652_WOMMode mode);
uint8_t IIM42652_GetWOMMode(IIM42652_t *imu, uint8_t* value);
uint8_t IIM42652_SetSMDMode(IIM42652_t *imu, IIM42652_SMDMode mode);
uint8_t IIM42652_GetSMDMode(IIM42652_t *imu, uint8_t* value);

uint8_t IIM42652_SetAccelMode(IIM42652_t *imu, IIM42652_AccelMode mode);
uint8_t IIM42652_SetGyroMode(IIM42652_t *imu, IIM42652_GyroMode mode);
uint8_t IIM42652_SetTempMode(IIM42652_t *imu, uint8_t EnableState);

uint8_t IIM42652_GetAccelAafBW(IIM42652_t *imu, uint16_t* value);
uint8_t IIM42652_GetGyroAafBW(IIM42652_t *imu, uint16_t* value);

uint8_t IIM42652_SetGyroFSR(IIM42652_t *imu, IIM42652_GyroFSRValues GyroFSR);
uint8_t IIM42652_GetGyroFSR(IIM42652_t *imu, uint8_t* value);
uint8_t IIM42652_SetAccelFSR(IIM42652_t *imu, IIM42652_AccFSRValues AccelFSR);
uint8_t IIM42652_GetAccelFSR(IIM42652_t *imu, uint8_t* value);
uint8_t IIM42652_SetGyroODR(IIM42652_t *imu, IIM42652_GyroODRValues GyroODR);
uint8_t IIM42652_GetGyroODR(IIM42652_t *imu, uint8_t* value);
uint8_t IIM42652_SetAccelODR(IIM42652_t *imu, IIM42652_AccODRValues AccelODR);
uint8_t IIM42652_GetAccelODR(IIM42652_t *imu, uint8_t* value);
uint8_t IIM42652_SetAccelBW(IIM42652_t *imu, IIM42652_AccBWValues AccelBW);
uint8_t IIM42652_GetAccelBW(IIM42652_t *imu, uint8_t* value);
uint8_t IIM42652_SetGyroBW(IIM42652_t *imu, IIM42652_GyroBWValues GyroBW);
uint8_t IIM42652_GetGyroBW(IIM42652_t *imu, uint8_t* value);
uint8_t IIM42652_SetAccelFilterOrder(IIM42652_t *imu, IIM42652_UIFilterOrder FO);
uint8_t IIM42652_GetAccelFilterOrder(IIM42652_t *imu, uint8_t* value);
uint8_t IIM42652_SetGyroFilterOrder(IIM42652_t *imu, IIM42652_UIFilterOrder FO);
uint8_t IIM42652_GetGyroFilterOrder(IIM42652_t *imu, uint8_t* value);
void IIM42652_SetAccelScaleFactor(IIM42652_t *imu, IIM42652_AccFSRValues AccelFSR);
void IIM42652_SetGyroScaleFactor(IIM42652_t *imu, IIM42652_GyroFSRValues GyroFSR);

uint8_t IIM42652_SetTMSReportEnable(IIM42652_t *imu, uint8_t EnableState);
uint8_t IIM42652_SetTMSFSyncEnable(IIM42652_t *imu, uint8_t EnableState);
uint8_t IIM42652_SetTMSDeltaMode(IIM42652_t *imu, uint8_t EnableState);
uint8_t IIM42652_SetTMSHighResolution(IIM42652_t *imu, uint8_t EnableState);
uint8_t IIM42652_SetTMStoRegEnable(IIM42652_t *imu, uint8_t EnableState);

uint8_t IIM42652_SetDMPPowerSaveMode(IIM42652_t *imu, uint8_t EnableState);
uint8_t IIM42652_SetDMPodr(IIM42652_t *imu, IIM42652_DMPODRValueS DmpODR);

uint8_t IIM42652_SetDMPInit(IIM42652_t *imu, uint8_t EnableState);
uint8_t IIM42652_SetDMPMemReset(IIM42652_t *imu);
uint8_t IIM42652_GetDMPMemReset(IIM42652_t *imu, uint8_t *state);

uint8_t IIM42652_DMPStart(IIM42652_t *imu);
uint8_t IIM42652_DMPResume(IIM42652_t *imu);
uint8_t IIM42652_GetDMPState(IIM42652_t *imu, uint8_t* state);

/* FIFO Management */
uint8_t IIM42652_FIFO_Flush(IIM42652_t *imu);
uint16_t IIM42652_GetFIFOCount(IIM42652_t *imu);
uint16_t IIM42652_GetFIFOCount_ISR(IIM42652_t *imu);
uint8_t IIM42652_GetFIFOData(IIM42652_t *imu);
uint8_t IIM42652_GetFIFOData_DMA(IIM42652_t *imu);
void IIM42652_GetFIFOData_DMA_Complete(IIM42652_t *imu);

/* Interrupt management */
uint8_t IIM42652_GetIntStatus(IIM42652_t *imu, uint8_t *value);
uint8_t IIM42652_GetIntStatus2(IIM42652_t *imu, uint8_t *value);
uint8_t IIM42652_GetIntStatus3(IIM42652_t *imu, uint8_t *value);

/* Raw-Data reading */
//Polling
uint8_t IIM42652_ReadIMUData(IIM42652_t *imu);

//DMA
uint8_t IIM42652_ReadGyroDMA(IIM42652_t *imu);
uint8_t IIM42652_ReadAccelDMA(IIM42652_t *imu);
void IIM42652_ReadGyroDMA_Complete(IIM42652_t *imu);
void IIM42652_ReadAccelDMA_Complete(IIM42652_t *imu);

/* Data elaboration */
float IIM42652_GetTempCelsius16(int16_t TemperatureHalfWord);
float IIM42652_GetTempCelsius8(int8_t FIFOTemperatureByte);
float IIM42652_GetAccelGravity(IIM42652_t *imu, int16_t AccelerationByte);
float IIM42652_GetGyroRps(IIM42652_t *imu, int16_t RotationByte);
uint16_t IIM42652_GetTimestampUS(uint16_t FIFOTimestampHalfWord);
void IIM42652_Format_Data(const uint8_t endian, const uint8_t *in, uint16_t *out);
uint16_t IIM42652_RegToAAFBW(uint8_t AAF_BS, uint8_t AAF_DELT, uint8_t AAF_DELTSQR);
void imu_TO_buffer(IIM42652_t *imu, uint8_t decimationFactor, IIM42652_FIFO_Packet16_t* imuData, uint8_t* buffer);
void buffer_TO_imu(IIM42652_FIFO_Packet16_t* imuData, uint8_t* buffer);
void imu_TO_vector(IIM42652_FIFO_Packet16_t* imuData, vc_vector* vector);
size_t imuUsbPacketsCreator(IIM42652_t *imu, IMU_Data_Buf_t* bufferIn, size_t bufferInLen, uint8_t* encBufferOut, size_t bufferOutLen);
#endif
