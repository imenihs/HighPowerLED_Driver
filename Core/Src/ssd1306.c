#include "stm32f1xx_hal.h"
#include <string.h>
#include "ssd1306.h"
// フォントROMをインクルード
#include "font.h"
// SSD1306 OLEDディスプレイのアドレス
#define OLED_ADDRESS (0x3c)
#define OLED_COLUMN (128)
#define OLED_ROW (32)

#ifdef OLED_RAM_RENDRING
typedef struct
{
	uint8_t cmd;
	uint8_t buf[OLED_COLUMN * OLED_ROW / 8];
	uint8_t reserve[10];
} OLED_t;
static OLED_t OLEDs = {0x00};
static uint8_t CurX = 0x00;
static uint8_t CurY = 0x00;
#endif

OLED_Error_t OLED_Init(I2C_HandleTypeDef *hi2c)
{
	static const uint8_t OLED_InitData[] = {
		0x00,			  // control command continuance
		0xae,			  // ディスプレイOFF
		0xa8, 0x1f,		  // 縦32dot★
		0xd3, 0x00,		  // ディスプレイ開始行を0に設定
		0x40,			  // ディスプレイ開始行を0に設定
		0xa1,			  // 描画　左→右
		0xc8,			  // 描画　上→下
		0xda, 0x02,		  // 表示モード★
		0x81, 0x7f,		  // コントラスト設定
		0xd5, 0x80,		  // クロック設定
		0xdb, 0x40,		  // VCOMHレベル設定
		0x8d, 0x14,		  // チャージポンプ電圧
		0x20, 0x02,		  // Set Memory Addressing Mode = Page addressing mode
		0x21, 0x00, 0x7f, // set Column Address, Column Start Address=0,Column Stop Address=127
		0xa4,			  // 全画面表示
		0xa6,			  // ノーマル表示モード
		0xaf,			  // ディスプレイON
	};
	HAL_I2C_Master_Transmit(hi2c, OLED_ADDRESS << 1, (uint8_t *)OLED_InitData, sizeof(OLED_InitData), 100);

#ifdef OLED_RAM_RENDRING
	OLEDs.cmd = 0x40;
	memset(OLEDs.buf, 0x00, sizeof(OLEDs.buf));
#endif
	return OLED_NOERR;
}

#ifdef OLED_ONDEMAND_RENDERING

OLED_Error_t OLED_Clear(void)
{
	const uint8_t oled_cmd[] = {
		0x00, // control command continuance
		0x20, // Set memory addressing mode
		0x00, // Horizontal Addressing Mode
		0x21, // Set column address
		0x00, // Column start address 0
		0x7f, // Column end address 127
		0x22, // Set page address
		0x00, // Page start address 0
		0x03, // Page end address 3★
	};
	I2C_WriteNBytes(OLED_ADDRESS, (uint8_t *)oled_cmd, sizeof(oled_cmd));

	const uint8_t fill[9] = {
		0x40,
		0x00,
	};
	for (uint16_t cnt = 0; cnt < 64; cnt++)
	{
		I2C_WriteNBytes(OLED_ADDRESS, (uint8_t *)fill, sizeof(fill));
	}
	return OLED_NOERR;
}
#endif

#ifdef OLED_RAM_RENDRING

OLED_Error_t OLED_Clear(void)
{
	memset(OLEDs.buf, 0x00, sizeof(OLEDs.buf));
	CurX = 0;
	CurY = 0;
	return OLED_NOERR;
}
#endif

#ifdef OLED_ONDEMAND_RENDERING

OLED_Error_t OLED_SetCursor(uint8_t x, uint8_t y)
{
	uint8_t oled_cmd[] = {
		0x00, // control command continuance
		0x20, // Set memory addressing mode
		0x02, // Page addressing mode
		0x00, // Page
		0x00, // Column low(4bit)
		0x00, // Column high(4bit)
	};
	if (x >= OLED_COLUMN)
		return OLED_ERROR;
	if (y >= (OLED_ROW / 8))
		return OLED_ERROR;
	oled_cmd[3] = 0xb0 + (y & 0x03);
	oled_cmd[4] = x & 0x0f;
	oled_cmd[5] = 0x10 + ((x >> 4) & 0x0f);

	I2C_WriteNBytes(OLED_ADDRESS, (uint8_t *)oled_cmd, sizeof(oled_cmd));

	return OLED_NOERR;
}

OLED_Error_t OLED_WriteChar(uint8_t code)
{
	code = code - 0x20;				// フォントROMを0x20オフセットして削除しているのに合わせる
	code = (code > 91) ? 91 : code; // 配列範囲外予防
	uint8_t oled_cmd[] = {
		0x40, // control command continuance
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
	};

	memcpy(&oled_cmd[1], &font6x8[code][0], 6);
	I2C_WriteNBytes(OLED_ADDRESS, (uint8_t *)oled_cmd, sizeof(oled_cmd));

	return OLED_NOERR;
}
#endif

#ifdef OLED_RAM_RENDRING

OLED_Error_t OLED_SetCursor(uint8_t x, uint8_t y)
{
	if (x >= OLED_COLUMN)
		return OLED_ERROR;
	if (y >= (OLED_ROW / 8))
		return OLED_ERROR;
	CurX = x;
	CurY = y;
	return OLED_NOERR;
}

OLED_Error_t OLED_WriteChar(uint8_t code)
{
	code = code - 0x20;				// フォントROMを0x20オフセットして削除しているのに合わせる
	code = (code > 91) ? 91 : code; // 配列範囲外予防
	uint16_t offset;
	offset = (uint16_t)CurX + (uint16_t)CurY * OLED_COLUMN;
	uint8_t *ptr;
	ptr = OLEDs.buf + offset;
	memcpy(ptr, font6x8[code], 6);
	CurX += 6;
	if (CurX >= OLED_COLUMN)
	{
		CurX -= OLED_COLUMN;
		CurY++;
		if (CurY >= (OLED_ROW / 8))
		{
			CurY = 0;
			memcpy(OLEDs.buf, OLEDs.reserve, CurX);
		}
	}
	return OLED_NOERR;
}
#endif

OLED_Error_t OLED_WriteString(char *str)
{
	while (*str)
	{
		OLED_WriteChar(*str);
		str++;
	}

	return OLED_NOERR;
}

#ifdef OLED_RAM_RENDRING

OLED_Error_t OLED_Update(I2C_HandleTypeDef *hi2c)
{
	static const uint8_t oled_cmd[] = {
		0x00, // control command continuance
		0x20, // Set memory addressing mode
		0x00, // Horizontal Addressing Mode
		0x21, // Set column address
		0x00, // Column start address 0
		0x7f, // Column end address 127
		0x22, // Set page address
		0x00, // Page start address 0
		0x03, // Page end address 3★
	};
	HAL_I2C_Master_Transmit(hi2c, OLED_ADDRESS << 1, (uint8_t *)oled_cmd, sizeof(oled_cmd), 10);

	OLED_t *ptr;
	ptr = &OLEDs;
	OLEDs.cmd = 0x40;
	HAL_I2C_Master_Transmit_DMA(hi2c, OLED_ADDRESS << 1, (uint8_t *)ptr, sizeof(OLEDs.cmd) + sizeof(OLEDs.buf));

	return OLED_NOERR;
}
#endif

void intToString(int32_t value, char *result)
{
	uint8_t i = 0;
	uint8_t isNegative = 0;

	// 負の値の場合、符号を記録し、正の値に変換
	if (value < 0)
	{
		isNegative = 1;
		value = -value;
	}

	// 整数を文字列に変換
	do
	{
		result[i++] = value % 10 + '0';
		value /= 10;
	} while (value);

	// 負の値の場合、マイナス符号を追加
	if (isNegative)
	{
		result[i++] = '-';
	}

	result[i] = '\0'; // 文字列の終端を設定
	// 文字列を逆順にする（必要に応じて）
	uint8_t length = i;
	for (uint8_t j = 0; j < length / 2; j++)
	{
		char temp = result[j];
		result[j] = result[length - j - 1];
		result[length - j - 1] = temp;
	}
}
