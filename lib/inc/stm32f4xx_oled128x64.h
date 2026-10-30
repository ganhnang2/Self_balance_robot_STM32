/**
 * @file    oled128x64.h
 * @author  Nguyen Hai Nam
 * @brief   Header file for SSD1306 OLED 128x64 driver on STM32F4 (Bare-Metal).
 *
 * @note
 *   - Compatible with STM32F4 I2C peripheral driver (stm32f4xx_i2c.h).
 *   - Supports both Direct/Page mode and 1024-byte RAM buffer mode.
 *   - OLED I2C 7-bit address is 0x3C (0x78 in 8-bit format).
 *   - The OLED_Init() function takes a pointer to an already-initialized I2C_Handle_t instance:
 *     + If NULL is passed: The driver automatically initializes and uses the default I2C2 peripheral 
 *       (Standard Mode 100 kHz). Note that GPIO PB10/PB3 must still be configured beforehand (e.g. via Pin_Config_OLED()).
 *     + If non-NULL is passed: The user MUST configure GPIO pins and initialize the I2C peripheral 
 *       (via I2C_Init()) externally beforehand. The driver will NOT call I2C_Init() internally, 
 *       allowing shared-bus I2C configurations.
 */

#ifndef OLED_128X64_H_
#define OLED_128X64_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/*                             OLED SPECIFICATIONS                            */
/******************************************************************************/

#define OLED_I2C_ADDR               0x3C   	/*!< 7-bit I2C Slave Address */
#define OLED_WIDTH                  128    	/*!< OLED display width in pixels */
#define OLED_HEIGHT                 64     	/*!< OLED display height in pixels */
#define OLED_PAGES                  8      	/*!< Total pages (64 / 8) */
#define OLED_BUFFER_SIZE            1024   	/*!< 128 * 64 / 8 bytes */

/* Font & Text Parameters */
#define OLED_CHAR_HEIGHT            7U      /*!< Height of font 5x7 */
#define OLED_CHAR_WIDTH             5U      /*!< Width of font 5x7 */
#define OLED_SEPARATE_SPACING       1U      /*!< Spacing between consecutive characters */

/* Backwards compatibility macros */
#define OLED_ADDRESS                OLED_I2C_ADDR
#define OLED_MAX_COLUMN             OLED_WIDTH
#define OLED_MAX_ROW                OLED_HEIGHT
#define OLED_MAX_PAGE               OLED_PAGES
#define OLED_MAX_ROW_INDEX          (OLED_HEIGHT - 1U)
#define OLED_MAX_COLUMN_INDEX       (OLED_WIDTH - 1U)
#define OLED_MAX_PAGE_INDEX         (OLED_PAGES - 1U)

#define OLED_WRITE_COMMAND			0x80
#define OLED_WRITE_DATA_TO_RAM		0xC0
#define OLED_WRITE_ALL_COMMANDS		0x00
#define OLED_WRITE_ALL_DATAS		0x40

#define OLED_DMA_STATE_IDLE			0
#define OLED_DMA_STATE_BUSY			1

/******************************************************************************/
/*                              COLOR ENUM                                    */
/******************************************************************************/

typedef enum {
    OLED_COLOR_BLACK = 0,   /*!< Pixel turned OFF */
    OLED_COLOR_WHITE = 1,   /*!< Pixel turned ON */
    OLED_COLOR_INVERT = 2   /*!< Pixel state inverted */
} OLED_Color_t;

/******************************************************************************/
/*                        PROTOTYPE FUNCTIONS                                 */
/******************************************************************************/

/* Fundamental Management Functions */
void OLED_Init(I2C_Handle_t *pI2CHandle);
void OLED_DMA_Init(I2C_Handle_t *pI2CHandle);
void OLED_Set_Brightness(uint8_t brightness);
void OLED_Display_Mode(uint8_t mode);
void OLED_Sleep(void);
void OLED_Wakeup(void);
void OLED_Clear_Display(void);
void OLED_Set_Cursor(uint8_t page, uint8_t col);
void OLED_Draw_Bit_Map(const uint8_t *image);

/* Framebuffer Drawing Functions */
void OLED_Clear_Buffer(void);
void OLED_Fill_Buffer(uint8_t pattern);
void OLED_Draw_Pixel(uint8_t x, uint8_t y, OLED_Color_t color);
void OLED_Print_Character(char ch, uint8_t x, uint8_t y, OLED_Color_t color);
void OLED_Print_String(const char *str, uint8_t x, uint8_t y, OLED_Color_t color);

void OLED_Update_Display(void);
void OLED_DMA_Init(I2C_Handle_t *pI2CHandle);
void OLED_Update_Display_DMA(void);
uint8_t OLED_Get_State_DMA(void);
void OLED_DMA_IRQ_Handler(void);

/* Scrolling Functions */
void OLED_Horizontal_Scroll(uint8_t startPage, uint8_t endPage, uint8_t isLeft);
void OLED_Vertical_And_Horizontal_Scroll(uint8_t startPage, uint8_t endPage, uint8_t verticalOffset, uint8_t isLeft);
void OLED_Deactivate_Scroll(void);
void OLED_Scroll_Area(uint8_t fixedRows, uint8_t scrollingRows);

/* Low-level Communication Helpers */
void OLED_Write_Command(uint8_t cmd);
void OLED_Write_Data(uint8_t data);
void OLED_Write_Data_Stream(const uint8_t *pData, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* OLED_128X64_H_ */