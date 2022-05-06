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

#include "RGB.h"

RGB_t RGB;

void RGB_Init(RGB_t *pRGBClass,TIM_HandleTypeDef *timerHandle)
{
	/* Init class variables */
	pRGBClass->m_timerHandle = timerHandle;
	pRGBClass->blinkState = 0;

	/* Start RGB LED Timer */
	HAL_TIM_Base_Start_IT(pRGBClass->m_timerHandle);

	/* Start RGB LED PWM channels */
	HAL_TIM_PWM_Start(pRGBClass->m_timerHandle, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(pRGBClass->m_timerHandle, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(pRGBClass->m_timerHandle, TIM_CHANNEL_3);

	/* Create standard color */
	//BLACK
	pRGBClass->rgbBLACK.R = 0;
	pRGBClass->rgbBLACK.G = 0;
	pRGBClass->rgbBLACK.B = 0;
	//BLUE
	pRGBClass->rgbBLUE.R = 0;
	pRGBClass->rgbBLUE.G = 0;
	pRGBClass->rgbBLUE.B = 1024;
	//RED
	pRGBClass->rgbRED.R = 512;
	pRGBClass->rgbRED.G = 0;
	pRGBClass->rgbRED.B = 0;
	//GREEN
	pRGBClass->rgbGREEN.R = 0;
	pRGBClass->rgbGREEN.G = 256;
	pRGBClass->rgbGREEN.B = 0;
	//CYAN
	pRGBClass->rgbCYAN.R = 0;
	pRGBClass->rgbCYAN.G = 256;
	pRGBClass->rgbCYAN.B = 1024;
	//YELLOW
	pRGBClass->rgbYELLOW.R = 512;
	pRGBClass->rgbYELLOW.G = 256;
	pRGBClass->rgbYELLOW.B = 0;
	//VIOLET
	pRGBClass->rgbVIOLET.R = 512;
	pRGBClass->rgbVIOLET.G = 0;
	pRGBClass->rgbVIOLET.B = 1024;
}

void RGB_SetActColor(RGB_t *pRGBClass, Color colorValues, uint8_t EnableMemory)
{
	pRGBClass->m_timerHandle->Instance->CCR1 = 1024 - colorValues.B;
	pRGBClass->m_timerHandle->Instance->CCR2 = 1024 - colorValues.R;
	pRGBClass->m_timerHandle->Instance->CCR3 = 1024 - colorValues.G;

	if (EnableMemory)
	{
		pRGBClass->m_ActColor = colorValues;
	}
}

Color RGB_GetActColor(RGB_t *pRGBClass)
{
	return pRGBClass->m_ActColor;
}

void RGB_Blink(RGB_t *pRGBClass)
{
	if (pRGBClass->blinkState)
	{
		RGB_SetActColor(pRGBClass, pRGBClass->rgbBLACK, DISABLE);
		pRGBClass->blinkState = 0;
	}
	else
	{
		RGB_SetActColor(pRGBClass, pRGBClass->m_ActColor, DISABLE);
		pRGBClass->blinkState = 1;
	}
}
