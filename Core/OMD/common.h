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
#ifndef _COMMON_H_
#define _COMMON_H_

//--------------------------------------
// ------------ LIBRARYS ---------------
//--------------------------------------
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "arm_math.h"
#include "math.h"
#include "string.h"
#include <stdbool.h>
#include "IIM42652.h"
#include "DAC7571.h"
#include "RGB.h"
#include "RingBuffer.h"
#include "USBCommProtocol.h"
#include "stream_buffer.h"
#include "vc_vector.h"
#include "flash_manager.h"

//--------------------------------------
//------------ DEFINES -----------------
//--------------------------------------
#define USER_DATA_SECTOR_START_ADDRESS 0x08004000

//--------------------------------------
//------- STRUCT / ENUMERATOR ----------
//--------------------------------------

//--------------------------------------
//------------ VARIABLES ---------------
//--------------------------------------
extern IIM42652_t IMU;
extern DAC7571_t DAC;
extern RGB_t RGB;
extern AC1_StatusWord_t AC1_StatusWord;
extern OMD_Protocol_t OMDCommProtocol;
extern USBD_HandleTypeDef hUsbDeviceFS;

extern RINGBUFFER(AccXBuffer, IMU_FFT_SIZE, float32_t);
extern RINGBUFFER(AccYBuffer, IMU_FFT_SIZE, float32_t);
extern RINGBUFFER(AccZBuffer, IMU_FFT_SIZE, float32_t);
extern RINGBUFFER(GyroXBuffer, IMU_FFT_SIZE, float32_t);
extern RINGBUFFER(GyroYBuffer, IMU_FFT_SIZE, float32_t);
extern RINGBUFFER(GyroZBuffer, IMU_FFT_SIZE, float32_t);

//--------------------------------------
//------------ FUNCTIONS ---------------
//--------------------------------------
void PrepareJumpToBootloader(void);
static void JumpToBootloader(void);
void CheckBootloaderJump();

uint8_t BitSetW(uint32_t *pWord, uint8_t bitPosition);
uint8_t BitSetHW(uint16_t *pHalfWord, uint8_t bitPosition);
uint8_t BitSetB(uint8_t *pByte, uint8_t bitPosition);

uint8_t BitResetW(uint32_t *pWord, uint8_t bitPosition);
uint8_t BitResetHW(uint16_t *pHalfWord, uint8_t bitPosition);
uint8_t BitResetB(uint8_t *pByte, uint8_t bitPosition);

uint8_t BitCheckW(uint32_t Word, uint8_t bitPosition);
uint8_t BitCheckHW(uint16_t HalfWord, uint8_t bitPosition);
uint8_t BitCheckB(uint8_t Byte, uint8_t bitPosition);

uint8_t _SAVE_AC1_SETTINGS(AC1_Config_t actConfig);
uint8_t _LOAD_AC1_SETTINGS(AC1_Config_t *actConfig);

/*
 * @brief
 *
 * Funzione di invio dati tramite VCP ottimizzata:
 * inizio la transazione dei dati solamente quando il buffer d'invio e pieno.
 * Questo poichè in FS abbiamo un sync packet solamente ogni 1ms.
 */
void _USB_SEND_OPT(uint8_t* byteBuf, size_t byteBufLen);
#endif
