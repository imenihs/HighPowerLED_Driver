/*
 * File:   ssd1306.h
 * Author: AkiraShimizu
 *
 * Created on October 26, 2023, 9:45 PM
 */

#ifndef SSD1306_H
#define SSD1306_H

#ifdef __cplusplus
extern "C"
{
#endif

	// 描画を都度行うか、RAMバッファに貯めて一気に行うか
	// #define OLED_ONDEMAND_RENDERING		//有効にすると都度描画

#ifndef OLED_ONDEMAND_RENDERING
#define OLED_RAM_RENDRING
#endif

#include "stm32f1xx_hal.h"
// フォントROMをインクルード
#include "font.h"
	typedef enum
	{
		OLED_NOERR = 0,
		OLED_ERROR,
	} OLED_Error_t;

	OLED_Error_t OLED_Init(I2C_HandleTypeDef *hi2c);
	OLED_Error_t OLED_Clear(void);
	OLED_Error_t OLED_SetCursor(uint8_t x, uint8_t y);
	OLED_Error_t OLED_WriteChar(uint8_t code);
	OLED_Error_t OLED_WriteString(char *str);

	void intToString(int32_t value, char *result);
#ifdef OLED_RAM_RENDRING
	OLED_Error_t OLED_Update(I2C_HandleTypeDef *hi2c);
#endif

#ifdef __cplusplus
}
#endif

#endif /* SSD1306_H */
