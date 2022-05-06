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
#ifndef _RGB_H_
#define _RGB_H_

//--------------------------------------
// ------------ LIBRARYS ---------------
//--------------------------------------
#include "stm32f4xx_hal.h"

//--------------------------------------
//-------- CONFIGURATION VAL -----------
//--------------------------------------
#define CYCLE_TIME_MS_LED  500


//--------------------------------------
// -------- CLASS VARIABLES ------------
//--------------------------------------
typedef struct {
	uint16_t R;
	uint16_t G;
	uint16_t B;
} Color;

typedef struct {
	/* PWM */
	TIM_HandleTypeDef *m_timerHandle;

	/* Status */
	Color m_ActColor;
	uint8_t blinkState;

	/* Color */
	Color rgbBLACK, rgbBLUE, rgbRED, rgbGREEN, rgbCYAN, rgbYELLOW, rgbVIOLET;

} RGB_t;


//--------------------------------------
//------------ FUNCTIONS ---------------
//--------------------------------------
void RGB_Init(RGB_t *pRGBClass,TIM_HandleTypeDef *timerHandle);
void RGB_SetActColor(RGB_t *pRGBClass, Color colorValues, uint8_t EnableMemory);
Color RGB_GetActColor(RGB_t *pRGBClass);
void RGB_Blink(RGB_t *pRGBClass);

#endif
