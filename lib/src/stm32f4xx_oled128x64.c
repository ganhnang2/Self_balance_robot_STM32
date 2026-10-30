/**
 * @file    oled128x64.c
 * @author  Nguyen Hai Nam
 * @brief   Source file for SSD1306 OLED 128x64 driver on STM32F4 (Bare-Metal).
 *
 * @note
 *   - Compatible with STM32F4 I2C peripheral driver (stm32f4xx_i2c.h).
 *   - Framebuffer of 1024 bytes (128x64 monochrome).
 *   - Font 5x7 defined in Character_Definition.h.
 *	 - All scroll functions here have a fixed interval of 5 frames between two scrolls, and the default scroll direction is to the right.
 *	 - Only apply to Page Addressing Mode.
 *   - The OLED_Init() function takes a pointer to an already-initialized I2C_Handle_t instance:
 *     + If NULL is passed: The driver automatically initializes and uses the default I2C2 peripheral 
 *       (Standard Mode 100 kHz). Note that GPIO PB10/PB3 must still be configured beforehand (e.g. via Pin_Config_OLED()).
 *     + If non-NULL is passed: The user MUST configure GPIO pins and initialize the I2C peripheral 
 *       (via I2C_Init()) externally beforehand. The driver will NOT call I2C_Init() internally, 
 *       allowing shared-bus I2C configurations.
 */

#include <string.h>
#include "stm32f4xx_oled128x64.h"
#include "stm32f4xx_font5x7.h"

// Internal handle for I2C2 peripheral dedicated to OLED 
static I2C_Handle_t g_oledI2cHandle;
static I2C_Handle_t *pI2cOled = NULL;

// Framebuffer stored in internal SRAM: 128 columns x 8 pages (1024 bytes)
static uint8_t bufferRAM[OLED_BUFFER_SIZE];

// Flag indicating background DMA transfer status: 1 = Busy, 0 = Idle/Complete
static volatile uint8_t g_oledDmaBusy = OLED_DMA_STATE_IDLE;

// Temporary transmit buffer for packetized I2C transfers: [Control_Byte, Data...]
static uint8_t i2cTxBuf[OLED_WIDTH + 1U];

// DMA buffer used for transmitting data over the I2C bus
static uint8_t dmaBuf[OLED_BUFFER_SIZE + 1];

/**
 * @brief  Configure GPIO pins (PB10 - SCL, PB3 - SDA) and initialize I2C2 peripheral for OLED.
 * @note   - PB10: I2C2_SCL (Alternate Function AF4)
 *         - PB3:  I2C2_SDA (Alternate Function AF9 on STM32F401)
 *         - Both pins configured as Open-Drain with no pull-up and pull-down.
 */
static void OLED_I2C_Init(void)
{
	/* Initialize I2C2 peripheral (GPIO PB10/PB3 configured via Pin_Config_OLED) */
	g_oledI2cHandle.pI2Cx                  = I2C2;
	g_oledI2cHandle.I2C_Config.SCLSpeed    = 200000UL;   /* 200 kHz */
	g_oledI2cHandle.I2C_Config.ACKControl  = I2C_ACK_ENABLE;
	g_oledI2cHandle.I2C_Config.FMDutyCycle = I2C_FM_DUTY_2;
	I2C_Init(&g_oledI2cHandle);
	
	/* Link internal I2C handle */
	pI2cOled = &g_oledI2cHandle;
}

/*----------------------------------------------------------------------------*/
/*                    Low-level Communication Helpers                         */
/*----------------------------------------------------------------------------*/
/**
 * @brief  	- Send a single command byte to SSD1306.
 * @param  	- cmd: Command byte.
 * @note	- Bit Co = 1, D/C# = 0: One data byte + data byte is interpreted as a command
 */
void OLED_Write_Command(uint8_t cmd)
{
    if (pI2cOled == NULL) 
		return;

    uint8_t packet[2];
    packet[0] = 0x80; // 0x80: Co=1, D/C#=0 (Single command byte)
    packet[1] = cmd;

    I2C_Master_Send_Data(pI2cOled, packet, 2, OLED_I2C_ADDR, DISABLE);
}

/**
 * @brief  	- Send a single data byte to SSD1306 GDDRAM.
 * @param  	- data: Data byte.
 * @note	- Bit Co = 1, D/C# = 1: One data byte + data byte is interpreted as a display data om GDRAM
 */
void OLED_Write_Data(uint8_t data)
{
    if (pI2cOled == NULL) 
		return;

    uint8_t packet[2];
    packet[0] = 0xC0; // 0xC0: Co=1, D/C#=1 (Single data byte)
    packet[1] = data;

    I2C_Master_Send_Data(pI2cOled, packet, 2, OLED_I2C_ADDR, DISABLE);
}

/**
 * @brief  	- Send a stream of data bytes to SSD1306 GDDRAM in a single I2C burst.
 * @param  	- pData: Pointer to raw display bytes.
 * @param  	- len:   Number of bytes (up to OLED_WIDTH = 128 bytes).
 * @note	- Bit Co = 0, D/C# = 1: Mutiple data byte + data byte is interpreted as a display data om GDRAM
 */
void OLED_Write_Data_Stream(const uint8_t *pData, uint16_t len)
{
    if (pI2cOled == NULL || pData == NULL || len == 0) 
		return;

    while (len > 0)
    {
        uint16_t chunk = (len > OLED_WIDTH) ? OLED_WIDTH : len;
        i2cTxBuf[0] = 0x40; 		// 0x40: Co=0, D/C#=1 (Stream of display data)
		
		// i2cTxBuf = [Control_Byte, Data...]
		// Control byte is stored into first index of i2cTxBuf, remaining byte is data bytes. 
        memcpy(&i2cTxBuf[1], pData, chunk);

        I2C_Master_Send_Data(pI2cOled, i2cTxBuf, chunk + 1U, OLED_I2C_ADDR, DISABLE);

        pData += chunk;
        len -= chunk;
    }
}

/*----------------------------------------------------------------------------*/
/*                       Fundamental Management Functions                     */
/*----------------------------------------------------------------------------*/

/**
 * @brief  Initialize SSD1306 OLED 128x64 display in the Polling mode.
 * @param  pI2CHandle: Pointer to ALREADY-INITIALIZED I2C_Handle_t, or NULL for default I2C2.
 * @note   - If NULL is passed: The driver automatically initializes default I2C2 (PB10/PB3 configured beforehand).
 *         - If non-NULL is passed: The user MUST configure GPIO pins and call I2C_Init() externally beforehand.
 *           The driver will NOT re-initialize the I2C peripheral.
 * @return None
 */
void OLED_Init(I2C_Handle_t *pI2CHandle)
{
    if (pI2CHandle != NULL)
    {
        pI2cOled = pI2CHandle;
    }
	else
	{
		/* Configure I2C peripheral and GPIO pins (PB10, PB3) which is used for OLED128x64 */
		OLED_I2C_Init();
	}

    /* SSD1306 Initialization sequence for 128x64 display */
    OLED_Write_Command(0xAE);          // Step 1: 0xAE: Display OFF

    OLED_Write_Command(0xD5);          // 0xD5: Set Display Clock Divide Ratio / Osc Freq
    OLED_Write_Command(0x80);          // Suggested ratio 0x80

    OLED_Write_Command(0xA8);          // 0xA8: Set Multiplex Ratio
    OLED_Write_Command(0x3F);          // 1/64 duty (64 - 1 = 63 = 0x3F)

    OLED_Write_Command(0xD3);          // 0xD3: Set Display Offset
    OLED_Write_Command(0x00);          // No offset

    OLED_Write_Command(0x40);          // 0x40: Set Display Start Line to 0

    OLED_Write_Command(0x8D);          // 0x8D: Charge Pump Setting
    OLED_Write_Command(0x14);          // Enable Charge Pump (0x14)

    OLED_Write_Command(0x20);          // 0x20: Set Memory Addressing Mode
    OLED_Write_Command(0x02);          // 0x02: Page Addressing Mode (default)

    OLED_Write_Command(0xA1);          // 0xA1: Column address 127 mapped to SEG0 (Horiz flip)
    OLED_Write_Command(0xC8);          // 0xC8: Scan from COM[N-1] to COM0 (Vert flip)

    OLED_Write_Command(0xDA);          // 0xDA: Set COM Pins Hardware Config
    OLED_Write_Command(0x12);          // Alternative COM pin config for 128x64

    OLED_Write_Command(0x81);          // 0x81: Set Contrast Control
    OLED_Write_Command(0xCF);          // Contrast value

    OLED_Write_Command(0xD9);          // 0xD9: Set Pre-charge Period
    OLED_Write_Command(0xF1);          // Phase 1: 1 DCLK, Phase 2: 15 DCLK

    OLED_Write_Command(0xDB);          // 0xDB: Set VCOMH Deselect Level
    OLED_Write_Command(0x40);          // ~0.77 x Vcc

    OLED_Write_Command(0xA4);          // 0xA4: Resume to RAM content display
    OLED_Write_Command(0xA6);          // 0xA6: Normal Display (non-inverted)

    OLED_Deactivate_Scroll();          // 0x2E: Deactivate any scrolling

    OLED_Clear_Display();              // Clear GDDRAM
    OLED_Clear_Buffer();               // Clear RAM buffer

    OLED_Write_Command(0xAF);          // 0xAF: Display ON
}

/**
 * @brief  Initialize SSD1306 OLED 128x64 display in the DMA mode.
 * @param  pI2CHandle: Pointer to ALREADY-INITIALIZED I2C_Handle_t, or NULL for default I2C2.
 * @note   - If NULL is passed: The driver automatically initializes default I2C2 (PB10/PB3 configured beforehand).
 *         - If non-NULL is passed: The user MUST configure GPIO pins and call I2C_Init() externally beforehand.
 *           The driver will NOT re-initialize the I2C peripheral.
 *		   - Horizontal Addressing Mode has been configured in this initialization because data streams transmit
 *		   	 continuously over the I2C bus
 * @return None
 */
void OLED_DMA_Init(I2C_Handle_t *pI2CHandle)
{
    /*************************Horizontal Addressing Mode*********************/
    OLED_Write_Command(0x20);          // Set Memory Addressing Mode
    OLED_Write_Command(0x00);          // 0x00: Horizontal Addressing Mode

    // Set column address window (0 -> 127)
    OLED_Write_Command(0x21);          // Set Column Address
    OLED_Write_Command(0x00);          // Start Column = 0
    OLED_Write_Command(0x7F);          // End Column = 127

    // Set page address window (0 -> 7)
    OLED_Write_Command(0x22);          // Set Page Address
    OLED_Write_Command(0x00);          // Start Page = 0
    OLED_Write_Command(0x07);          // End Page = 7

    OLED_Write_Command(0xAF);          // Display ON

    /*******************************DMA Peripheral****************************/
    // Enable DMA1 clock
    RCC->AHB1ENR |= (1 << RCC_AHB1ENR_DMA1EN_Pos);

    // Disable Stream 7 before configuring
    DMA1_Stream7->CR &= ~(1 << DMA_SxCR_EN_Pos);
    while (DMA1_Stream7->CR & (1 << DMA_SxCR_EN_Pos));

    // Clear all interrupt flags for Stream 7
    DMA1->HIFCR = (1UL << DMA_HIFCR_CTCIF7_Pos) | (1UL << DMA_HIFCR_CHTIF7_Pos) | (1UL << DMA_HIFCR_CTEIF7_Pos);

    // Peripheral data size: 8 bits
    DMA1_Stream7->CR &= ~(0x03 << DMA_SxCR_PSIZE_Pos);

    // Memory data size: 8 bits
    DMA1_Stream7->CR &= ~(0x03 << DMA_SxCR_MSIZE_Pos);

    // Direction: Memory to peripheral
    DMA1_Stream7->CR |= (1 << DMA_SxCR_DIR_Pos);

    // Disable peripheral increment mode
    DMA1_Stream7->CR &= ~(1 << DMA_SxCR_PINC_Pos);

    // Enable memory increment mode
    DMA1_Stream7->CR |= (1 << DMA_SxCR_MINC_Pos);

    // Channel 7 = I2C2_TX (DMA1 Stream7 Channel7)
    DMA1_Stream7->CR &= ~(0x07 << DMA_SxCR_CHSEL_Pos);
    DMA1_Stream7->CR |= (0x07 << DMA_SxCR_CHSEL_Pos);

    // Configure peripheral and memory addresses
    DMA1_Stream7->PAR  = (uint32_t)&(I2C2->DR);
    DMA1_Stream7->M0AR = (uint32_t)i2cTxBuf;
}

/**
 * @brief  	- Move cursor to specific location (page, column) in Page Addressing Mode.
 * @param  	- page: Page index (0 to 7).
 * @param  	- col:  Column index (0 to 127).
 */
void OLED_Set_Cursor(uint8_t page, uint8_t col)
{
    if (page > OLED_MAX_PAGE_INDEX) 
		page = OLED_MAX_PAGE_INDEX;
    if (col > OLED_MAX_COLUMN_INDEX) 
		col = OLED_MAX_COLUMN_INDEX;
	
	/*	Set GDDRAM Page Start Address  */
    OLED_Write_Command(0xB0 | page);              	
	
	/*	Set column start address	*/
    OLED_Write_Command(0x00 | (col & 0x0F));       	// Set the lower nibble of the column start address register (Only for Page Adressing mode)
    OLED_Write_Command(0x10 | ((col >> 4) & 0x0F));	// Set the higher nibble of the column start address register (Only for Page Adressing mode)
}

/**
 * @brief  	- Set contrast / brightness.
 * @param  	- brightness: 0 to 255.
 */
void OLED_Set_Brightness(uint8_t brightness)
{
    OLED_Write_Command(0x81); 		// 0x81: Set Contrast Control
    OLED_Write_Command(brightness);	
}

/**
 * @brief  	- Set display orientation mode.
 * @param  	- mode: 0: Normal mode (Top to bottom), 1: 180-degree inverted mode (Bottom to Top).
 * @note	- Top: Header in OLED 128x64, Bottom: The bottom of the OLED
 */
void OLED_Display_Mode(uint8_t mode)
{
    if (mode == 0)
    {
        OLED_Write_Command(0xA0);     	// 0xA0: Column address 0 mapped to SEG0
        OLED_Write_Command(0xC0); 		// 0xC0: Scan from COM0 to COM[N-1]
    }
    else
    {
        OLED_Write_Command(0xA1);      	// 0xA1: Column address 127 mapped to SEG0
        OLED_Write_Command(0xC8);  		// 0xC8: Scan from COM[N-1] to COM0
    }
}

/**
 * @brief  Put OLED into sleep mode (display off, charge pump disabled).
 */
void OLED_Sleep(void)
{
    OLED_Write_Command(0x8D);	// 0x8D: Charge Pump Setting
    OLED_Write_Command(0x10);	// 0x10: Disable charge pump
    OLED_Write_Command(0xAE);	// 0xAE: Display OFF
}

/**
 * @brief  Wakeup OLED from sleep mode.
 */
void OLED_Wakeup(void)
{
    OLED_Write_Command(0x8D); // 0x8D: Charge Pump Setting
    OLED_Write_Command(0x14); // 0x14: Enable charge pump
    OLED_Write_Command(0xAF); // 0xAF: Display ON
}

/**
 * @brief  Directly clear all data in OLED GDDRAM.
 */
void OLED_Clear_Display(void)
{
    uint8_t zeroPage[OLED_WIDTH];
    memset(zeroPage, 0x00, OLED_WIDTH);

    for (uint8_t page = 0; page < OLED_PAGES; page++)
    {
        OLED_Set_Cursor(page, 0);
        OLED_Write_Data_Stream(zeroPage, OLED_WIDTH);
    }
}

/**
 * @brief  	- Directly draw a 1024-byte full-screen bitmap image to GDDRAM.
 * @param  	- image: Pointer to 1024-byte bitmap array.
 * @note	- A bitmap image must be configured proprely with Vertical Orientation and Little-endian mode
 */
void OLED_Draw_Bit_Map(const uint8_t *image)
{
    if (image == NULL) 
		return;

    for (uint8_t page = 0; page < OLED_PAGES; page++)
    {
        OLED_Set_Cursor(page, 0);
        OLED_Write_Data_Stream(&image[page * OLED_WIDTH], OLED_WIDTH);
    }
}

/*----------------------------------------------------------------------------*/
/*                       Framebuffer Drawing Functions                        */
/*----------------------------------------------------------------------------*/

/**
 * @brief  Clear all pixels in the local framebuffer.
 */
void OLED_Clear_Buffer(void)
{
    memset(bufferRAM, 0x00, sizeof(bufferRAM));
}

/**
 * @brief  	- Fill framebuffer with a specific pattern.
 * @param  	- pattern: 8-bit vertical strip pattern.
 */
void OLED_Fill_Buffer(uint8_t pattern)
{
    memset(bufferRAM, pattern, sizeof(bufferRAM));
}

/**
 * @brief  	- Draw, erase, or invert a single pixel in the framebuffer.
 * @param  	- x:     X coordinate (0 to 127).
 * @param  	- y:     Y coordinate (0 to 63).
 * @param  	- color: OLED_COLOR_WHITE, OLED_COLOR_BLACK, or OLED_COLOR_INVERT.
 */
void OLED_Draw_Pixel(uint8_t x, uint8_t y, OLED_Color_t color)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) 
		return;

    uint16_t index = (uint16_t)((y / 8U) * OLED_WIDTH) + x;
    uint8_t bitMask = (1U << (y % 8U));

    switch (color)
    {
        case OLED_COLOR_WHITE:
            bufferRAM[index] |= bitMask;
            break;
        case OLED_COLOR_BLACK:
            bufferRAM[index] &= ~bitMask;
            break;
        case OLED_COLOR_INVERT:
            bufferRAM[index] ^= bitMask;
            break;
        default:
            break;
    }
}

/**
 * @brief  	- Print a single character onto the framebuffer using 5x7 font.
 * @param  	- ch:    Character to display.
 * @param  	- x:     Starting X column coordinate.
 * @param  	- y:     Starting Y row coordinate.
 * @param  	- color: Pixel color.
 */
void OLED_Print_Character(char ch, uint8_t x, uint8_t y, OLED_Color_t color)
{
    if (ch < 32 || ch > 127) 
		ch = ' ';

    const uint8_t *charData = &font_data_5x7[(ch - 32) * OLED_CHAR_WIDTH];

    for (uint8_t col = 0; col < OLED_CHAR_WIDTH; col++)
    {
        uint8_t line = charData[col];
        for (uint8_t row = 0; row < OLED_CHAR_HEIGHT; row++)
        {
            if (line & (1U << row))
            {
                OLED_Draw_Pixel(x + col, y + row, color);
            }
            else
            {
                OLED_Color_t bg = (color == OLED_COLOR_WHITE) ? OLED_COLOR_BLACK :
                                  (color == OLED_COLOR_BLACK) ? OLED_COLOR_WHITE : OLED_COLOR_INVERT;
                OLED_Draw_Pixel(x + col, y + row, bg);
            }
        }
    }
}

/**
 * @brief  	- Print a string onto the framebuffer with auto-wrapping.
 * @param  	- str:   Null-terminated string.
 * @param  	- x:     Starting X column.
 * @param  	- y:     Starting Y row.
 * @param  	- color: Pixel color.
 */
void OLED_Print_String(const char *str, uint8_t x, uint8_t y, OLED_Color_t color)
{
    if (str == NULL) return;

    uint8_t curX = x;
    uint8_t curY = y;

    while (*str != '\0')
    {
        if (*str == '\n')
        {
            curY += (OLED_CHAR_HEIGHT + OLED_SEPARATE_SPACING);
            curX = x;
            str++;
            continue;
        }

        if (curX + OLED_CHAR_WIDTH > OLED_WIDTH)
        {
            curY += (OLED_CHAR_HEIGHT + OLED_SEPARATE_SPACING);
            curX = x;
        }

        if (curY + OLED_CHAR_HEIGHT > OLED_HEIGHT)
        {
            break; // Exceeded screen height
        }

        OLED_Print_Character(*str, curX, curY, color);
        curX += (OLED_CHAR_WIDTH + OLED_SEPARATE_SPACING);
        str++;
    }
}

/**
 * @brief  Flush the entire 1024-byte framebuffer to OLED via polling (blocking mode).
 */
void OLED_Update_Display(void)
{
    for (uint8_t page = 0; page < OLED_PAGES; page++)
    {
        OLED_Set_Cursor(page, 0);
        OLED_Write_Data_Stream(&bufferRAM[page * OLED_WIDTH], OLED_WIDTH);
    }
}

/**
 * @brief  Flush the entire 1024-byte framebuffer to OLED via I2C2 using DMA controller (Non-blocking).
 *         Releases CPU in ~20 microseconds while DMA1 Stream 7 Channel 7 transfers all 1025 bytes in the background.
 *         DMA Transfer Complete Interrupt (TCIE) is enabled to conclude the transaction and generate STOP condition.
 */
void OLED_Update_Display_DMA(void)
{
    if (g_oledDmaBusy != OLED_DMA_STATE_IDLE)
    {
        return;
    }
	
	// Wait until I2C bus is free (avoid conflicts with an unfinished previous STOP condition)
	uint32_t timeout = I2C_TIMEOUT_MAX;
    while (I2C2->SR2 & I2C_SR2_BUSY)
    {
        if (--timeout == 0U) 
		{
			g_oledDmaBusy = OLED_DMA_STATE_IDLE;
			return;
		}
    }
	
    g_oledDmaBusy = OLED_DMA_STATE_BUSY;
	
	/**************************************CONFIGURATION*******************************/
    // Packet buffer: 1 Control byte (0x40) + 1024 data bytes (OLED_BUFFER_SIZE)
    dmaBuf[0] = 0x40;
    memcpy(&dmaBuf[1], bufferRAM, OLED_BUFFER_SIZE);

	timeout = I2C_TIMEOUT_MAX;
    DMA1_Stream7->CR &= ~DMA_SxCR_EN;
	
	// Wait until DMA is disabled 
    while (DMA1_Stream7->CR & DMA_SxCR_EN)
    {
        if (--timeout == 0U) 
		{
			g_oledDmaBusy = OLED_DMA_STATE_IDLE;
			return;
		}
    }
	
	// Clear all interrupt flags for Stream 7
	DMA1->HIFCR = (1UL << DMA_HIFCR_CTCIF7_Pos) | (1UL << DMA_HIFCR_CHTIF7_Pos) | (1UL << DMA_HIFCR_CTEIF7_Pos);

	// Number of data to transfer: 1 Control Byte (0x40) + 1024 data bytes
	DMA1_Stream7->NDTR = 1U + OLED_BUFFER_SIZE;
	
	// Peripheral address 
	DMA1_Stream7->PAR = (uint32_t)&(I2C2->DR);
	
	// Transmit buffer address 
	DMA1_Stream7->M0AR = (uint32_t)dmaBuf;
	
	// Transfer direction: to peripheral
	DMA1_Stream7->CR &= ~(0x03 << DMA_SxCR_DIR_Pos);
	DMA1_Stream7->CR |= (0x01 << DMA_SxCR_DIR_Pos);
	
	// Enable Transfer Complete and Transfer Error interrupts only.
	// HTIE (half-transfer) is NOT needed and wastes an ISR entry per page.
	DMA1_Stream7->CR &= ~(1 << DMA_SxCR_HTIE_Pos);
	DMA1_Stream7->CR |= (1 << DMA_SxCR_TCIE_Pos) | (1 << DMA_SxCR_TEIE_Pos);
	
	// Enable DMA1 Stream 7
	DMA1_Stream7->CR |= (1 << DMA_SxCR_EN_Pos);
	
	/********************************DATA TRANSFER************************************/
	// NOTE: The DMAEN bit must be set in the I2C_CR2 register before the ADDR event. (From the Reference Manual)
	I2C2->CR2 |= (1 << I2C_CR2_DMAEN_Pos);
	
	// Generate START Condition on I2C2 bus
    I2C2->CR1 |= I2C_CR1_START;
    timeout = I2C_TIMEOUT_MAX;
    while (!(I2C2->SR1 & I2C_SR1_SB))
    {
        if (--timeout == 0U)
        {
            I2C2->CR1 |= I2C_CR1_STOP;
            DMA1_Stream7->CR &= ~DMA_SxCR_EN;
			g_oledDmaBusy = OLED_DMA_STATE_IDLE;
            return;
        }
    }

    // Address Phase (Address + Bit R/W (Write))
    I2C2->DR = (uint8_t)(OLED_I2C_ADDR << 1U);
    timeout = I2C_TIMEOUT_MAX;
	
	// Wait until ADDR flag is set
    while (!(I2C2->SR1 & I2C_SR1_ADDR))
    {
        if ((I2C2->SR1 & I2C_SR1_AF) || (--timeout == 0U))
        {
            I2C2->SR1 &= ~I2C_SR1_AF;
            I2C2->CR2 &= ~I2C_CR2_DMAEN;
            DMA1_Stream7->CR &= ~DMA_SxCR_EN;
            I2C2->CR1 |= I2C_CR1_STOP;
            g_oledDmaBusy = OLED_DMA_STATE_IDLE;
            return;
        }
    }

    // CLear ADDR flag by read SR1 and SR2 sequencely
    uint32_t dummy = I2C2->SR1;
    dummy          = I2C2->SR2;
    (void)dummy;
}

/**
 * @brief  Check if the background OLED DMA transfer is currently in progress.
 * @return 1 if DMA is still busy transmitting, 0 if transmission complete and bus is idle.
 */
uint8_t OLED_Get_State_DMA(void)
{
    return g_oledDmaBusy;
}

/**
 * @brief  Trình phục vụ ngắt của DMA1 Stream 7 (I2C2 TX).
 *         Tự động được phần cứng gọi khi DMA đã gửi xong toàn bộ 1025 bytes cho OLED.
 */
 
void OLED_DMA_IRQ_Handler(void)
{
	uint32_t temp1, temp2;
	/*********************************TRANSFER COMPLETE*******************************/
	// Let's check Transfer complete (TCIE)
	temp1 = DMA1->HISR & (1 << DMA_HISR_TCIF7_Pos);
	temp2 = DMA1_Stream7->CR & (1 << DMA_SxCR_TCIE_Pos); 
    if (temp1 && temp2)
    {
		// 1. Clear TCIF flag by writing 1 to the corresponding bit in the DMA_HIFCR register 
		DMA1->HIFCR |= 1 << DMA_HIFCR_CTCIF7_Pos;	

		// 2. Disable DMA request on I2C2
		I2C2->CR2 &= ~(1UL << I2C_CR2_DMAEN_Pos);		

		// 3. Wait until the last byte is received completely (BTF = 1)
        uint32_t timeout = I2C_TIMEOUT_MAX;
        while (!(I2C2->SR1 & I2C_SR1_BTF))
        {
            if (--timeout == 0U) 
				break;
        }

		// 4. Generate STOP condition to finish the transaction on I2C bus
        I2C2->CR1 |= I2C_CR1_STOP;

		// 5. Disable Stream 7
        DMA1_Stream7->CR &= ~DMA_SxCR_EN;
		
		// 6. Update the transaction state
		g_oledDmaBusy = OLED_DMA_STATE_IDLE;
    }
	
	/*********************************HALF TRANSFER COMPLETE*******************************/
	// HTIE is disabled in OLED_DMA_Init() — clear the flag defensively if set
	temp1 = DMA1->HISR  & (1 << DMA_HISR_HTIF7_Pos);
    temp2 = DMA1_Stream7->CR & (1 << DMA_SxCR_HTIE_Pos);  
    if (temp1 && temp2)
	{
		// writing 1 to the corresponding bit in the DMA_HIFCR register to clear the corresponding CHTIF flag 
		DMA1->HIFCR |= 1 << DMA_HIFCR_CHTIF7_Pos;
	}
	
	/*********************************ERROR**********************************************/
	// Let's check Transfer Error (HTIIE)
	temp1 = DMA1->HISR & (1 << DMA_HISR_TEIF7_Pos);
	temp2 = DMA1_Stream7->CR & (1 << DMA_SxCR_TEIE_Pos); 
	if (temp1 && temp2)
	{
		// Writing 1 to the corresponding bit in the DMA_HIFCR register to clear the corresponding CTEIF flag 
		DMA1->HIFCR |= 1 << DMA_HIFCR_CTEIF7_Pos;		
		
		// Generate STOP condition to finish the transaction on I2C buss
        I2C2->CR1 |= I2C_CR1_STOP;
		
		// Disable DMA request on I2C2 and disable Stream 7
		I2C2->CR2 &= ~I2C_CR2_DMAEN;
        DMA1_Stream7->CR &= ~DMA_SxCR_EN;
		
		g_oledDmaBusy = OLED_DMA_STATE_IDLE;			// Clear the flag TxState to indicate that TX is free
	}
}

/*----------------------------------------------------------------------------*/
/*                            Scrolling Functions                             */
/*----------------------------------------------------------------------------*/

/**
 * @brief  	- Configure and activate continuous horizontal scroll.
 * @param  	- startPage: 0 to 7.
 * @param  	- endPage:   0 to 7.
 * @param  	- isLeft:    1 for Left scroll, 0 for Right scroll.
 */
void OLED_Horizontal_Scroll(uint8_t startPage, uint8_t endPage, uint8_t isLeft)
{
    if (startPage > 7) 
		startPage = 0;
    if (endPage > 7) 
		endPage = 7;

    OLED_Deactivate_Scroll();

    OLED_Write_Command(isLeft ? 0x27 : 0x26); // 0x27: Left Horizontal Scroll, 0x26: Right
    OLED_Write_Command(0x00);      // Dummy byte
    OLED_Write_Command(startPage); // Start page
    OLED_Write_Command(0x06);      // Interval: 25 frames
    OLED_Write_Command(endPage);   // End page
    OLED_Write_Command(0x00);      // Dummy byte
    OLED_Write_Command(0xFF);      // Dummy byte

    OLED_Write_Command(0x2F);      // 0x2F: Activate scroll
}

/**
 * @brief  	- Configure continuous vertical and horizontal scroll.
 * @param  	- startPage:      0 to 7.
 * @param  	- endPage:        0 to 7.
 * @param  	- verticalOffset: Vertical scroll offset (1 to 63 rows).
 * @param  	- isLeft:         1 for Left, 0 for Right.
 */
void OLED_Vertical_And_Horizontal_Scroll(uint8_t startPage, uint8_t endPage, uint8_t verticalOffset, uint8_t isLeft)
{
    if (verticalOffset > 63 || verticalOffset == 0) 
		verticalOffset = 1;
    if (startPage > 7) 
		startPage = 0;
    if (endPage > 7) 
		endPage = 7;

    OLED_Deactivate_Scroll();

    OLED_Write_Command(isLeft ? 0x2A : 0x29); // 0x2A: Vertical & Left, 0x29: Vertical & Right
    OLED_Write_Command(0x00);           // Dummy byte
    OLED_Write_Command(startPage);      // Start page
    OLED_Write_Command(0x07);           // Interval: 2 frames
    OLED_Write_Command(endPage);        // End page
    OLED_Write_Command(verticalOffset); // Vertical offset

    OLED_Write_Command(0x2F);           // 0x2F: Activate scroll
}

/**
 * @brief  	- Deactivate active scrolling.
 */
void OLED_Deactivate_Scroll(void)
{
    OLED_Write_Command(0x2E); // 0x2E: Deactivate scroll
}

/**
 * @brief  	- Define vertical scrolling area.
 * @param  	- fixedRows:     Number of rows in top fixed area (0 to 63).
 * @param  	- scrollingRows: Number of scrolling rows (1 to 64).
 */
void OLED_Scroll_Area(uint8_t fixedRows, uint8_t scrollingRows)
{
    if ((uint16_t)fixedRows + scrollingRows > OLED_HEIGHT)
    {
        fixedRows = 0;
        scrollingRows = OLED_HEIGHT;
    }

    OLED_Deactivate_Scroll();

    OLED_Write_Command(0xA3); // 0xA3: Set Vertical Scroll Area
    OLED_Write_Command(fixedRows);
    OLED_Write_Command(scrollingRows);
}
	