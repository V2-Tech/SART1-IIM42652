/*
 * USBCommProtocol.h
 *
 * 		Created on: Oct 25, 2021
 *      Author: Valerio Mazzoni
 *      Company: OMD
 *
 *      Version: 1.0.0
 *
 *      Changelog:
 *      	- 25/10/2021 - 1.0.0: start of the source code writing.
 */

#ifndef OMD_USBCOMMPROTOCOL_H_
#define OMD_USBCOMMPROTOCOL_H_

//--------------------------------------
// ------------ LIBRARYS ---------------
//--------------------------------------
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "common_def.h"
#include "IIM42652.h"
#include "RGB.h"
#include "USBCommProtocol_def.h"
#include "cobstranscoder.h"
#include "RingBuffer.h"
#include "vc_vector.h"

//--------------------------------------
// ------------ VARIABLES --------------
//--------------------------------------
extern osSemaphoreId usbBinarySemHandle;
extern RGB_t RGB;
extern osThreadId MainTaskHandle;
extern osThreadId UsbTxTaskHandle;
extern osThreadId UsbRxTaskHandle;
extern QueueHandle_t usb_RX_Queue;
extern QueueHandle_t usb_TX_Queue;
extern QueueHandle_t imu_Telemetry_Queue;

//--------------------------------------
// -------------- CLASS ----------------
//--------------------------------------
typedef struct {
	osMutexId m_TxQueueMutexHandle;

	uint8_t m_encodedRxBuf[APP_RX_DATA_SIZE];
	uint8_t m_encodedTxBuf[APP_TX_DATA_SIZE];

	RINGBUFFER(m_rxBuffer, APP_RX_DATA_SIZE, uint8_t);
} OMD_Protocol_t;

extern OMD_Protocol_t OMDCommProtocol;

//--------------------------------------
// ---------- CLASS METHODS ------------
//--------------------------------------

/****************************************
 * 			General
 ***************************************/
void OMDProtocolInit(OMD_Protocol_t *ProtocolStruct);

/****************************************
 * 			Processing
 ***************************************/
/* commandPacketsCreator
 *
 * @param	*decodedBuf Pointer to a omdDecode output buffer which contain AC1 commands
 * @param	decodedBufLen N-byte of the decoded input buffer
 * @param	*packetsBuf Pointer to a OMD_Command_Packet_t buffer where the packets will be stored
 *
 * @return	Number of command packets which have been assembled.
 */
/* Data retrieving */
void GiveRxData(OMD_Protocol_t *ProtocolStruct, vc_vector* rxDataVector);
void MoveRxDataInBuffer(OMD_Protocol_t *ProtocolStruct, vc_vector* rxDataVector, vc_vector* packetVector);

/* Data elaboration */
void executeCommand(OMD_Protocol_t *ProtocolStruct, vc_vector* command);
void sendCommand(OMD_Protocol_t *ProtocolStruct, uint8_t command, vc_vector* value);
void sendCommandBuf(OMD_Protocol_t *ProtocolStruct, uint8_t command, uint8_t value);
void sendCommandFromISR(OMD_Protocol_t *ProtocolStruct, uint8_t command, vc_vector* value);

/* Functions */
void ackCommand(OMD_Protocol_t *ProtocolStruct, vc_vector* command, uint8_t result);
void ackCommandBuf(OMD_Protocol_t *ProtocolStruct, uint8_t command, uint8_t result);

void saveUserConfig(void);
void setConfigMode(uint8_t EnableState);
uint8_t getConfigMode();
void setTelemetryEn(uint8_t EnableState);
uint8_t getTelemetryEn();
uint8_t setAccelODR(IIM42652_t *imu, uint8_t AccelODR);
uint8_t getAccelODR(IIM42652_t *imu, uint8_t* value);
uint8_t setGyroODR(IIM42652_t *imu, uint8_t GyroODR);
uint8_t getGyroODR(IIM42652_t *imu, uint8_t* value);
uint8_t setAccelFSR(IIM42652_t *imu, uint8_t AccelFSR);
uint8_t getAccelFSR(IIM42652_t *imu, uint8_t* value);
uint8_t setGyroFSR(IIM42652_t *imu, uint8_t GyroFSR);
uint8_t getGyroFSR(IIM42652_t *imu, uint8_t* value);
uint8_t setAccelBW(IIM42652_t *imu, uint8_t AccelBW);
uint8_t getAccelBW(IIM42652_t *imu, uint8_t* value);
uint8_t setGyroBW(IIM42652_t *imu, uint8_t GyroBW);
uint8_t getGyroBW(IIM42652_t *imu, uint8_t* value);
uint8_t setWomXth(IIM42652_t *imu, uint8_t threshold);
uint8_t getWomXth(IIM42652_t *imu, uint8_t* value);
uint8_t setWomYth(IIM42652_t *imu, uint8_t threshold);
uint8_t getWomYth(IIM42652_t *imu, uint8_t* value);
uint8_t setWomZth(IIM42652_t *imu, uint8_t threshold);
uint8_t getWomZth(IIM42652_t *imu, uint8_t* value);
uint8_t setWomIntMode(IIM42652_t *imu, uint8_t mode);
uint8_t getWomIntMode(IIM42652_t *imu, uint8_t* value);
uint8_t setWomMode(IIM42652_t *imu, uint8_t mode);
uint8_t getWomMode(IIM42652_t *imu, uint8_t* value);
uint8_t setSmdMode(IIM42652_t *imu, uint8_t mode);
uint8_t getSmdMode(IIM42652_t *imu, uint8_t* value);
#endif /* OMD_USBCOMMPROTOCOL_H_ */
