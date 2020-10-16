
#ifndef __OLED_INCLUDED__
#define __OLED_INCLUDED__

#include <stdint.h>
#include "fonts.h"
#include "ssd1306.h"

// Select custom I2C address
//#define SSD1306_I2C_ADDR		0x3C
#define SSD1306_I2C_ADDR		0x78
#define SSD1306_I2C_HAL_ADDR	SSD1306_I2C_ADDR
//#define SSD1306_I2C_HAL_ADDR	(SSD1306_I2C_ADDR << 1)

//Select custom width and height if your LCD differs in size
#define SSD1306_WIDTH            128
#define SSD1306_HEIGHT           64

// HAL defines
//#define HW_SPI_PORT_NSS			GPIOB, GPIO_PIN_6
//#define HW_SPI_PORT_SCK			GPIOA, GPIO_PIN_7
//#define HW_SPI_PORT_MOSI		GPIOA, GPIO_PIN_6
////#define HW_SPI_PORT_MISO		GPIOB, GPIO_PIN_4
//#define HW_PORT_NRESET			GPIOA, GPIO_PIN_9
//#define HW_PORT_D_OR_C			GPIOC, GPIO_PIN_7
//#define HW_PORT_SET(port)		HAL_GPIO_WritePin( port, GPIO_PIN_SET )
//#define HW_PORT_RESET(port)		HAL_GPIO_WritePin( port, GPIO_PIN_RESET )
//#define HW_PORT_READ(port)		HAL_GPIO_ReadPin( port )
#define HW_DELAY_MS(ms)			CyDelay( ms )
//#define HW_GET_SYS_TICKS()		HAL_GetTick()

/**
 * @brief  SSD1306 color enumeration
 */
//typedef enum {
//	SSD1306_COLOR_BLACK = 0x00, /*!< Black color, no pixel */
//	SSD1306_COLOR_WHITE = 0x01  /*!< Pixel is set. Color depends on LCD */
//} SSD1306_COLOR_t;

int SSD1306_WRITECOMMAND( uint8_t command );
int SSD1306_WRITEDATA( uint8_t data );
//int SSD1306_check_device( void );
void SSD1306_buf_write( uint8_t *pbuf, int length, uint32_t m);

uint8_t TM_SSD1306_Init(void);
void TM_SSD1306_UpdateScreen(void);
void TM_SSD1306_ToggleInvert(void);
void TM_SSD1306_Fill(SSD1306_COLOR_t Color);
void TM_SSD1306_DrawPixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color);
void TM_SSD1306_GotoXY(uint16_t x, uint16_t y);
char TM_SSD1306_Putc(char ch, TM_FontDef_t* Font, SSD1306_COLOR_t color);
char TM_SSD1306_Puts(char* str, TM_FontDef_t* Font, SSD1306_COLOR_t color);
void TM_SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_COLOR_t c);
void TM_SSD1306_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR_t c);
void TM_SSD1306_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR_t c);
void TM_SSD1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR_t color);
void TM_SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c);
void TM_SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c);

int I2C_Mem_Write_a8( uint8_t slave_Addr, uint8_t Address, uint8_t *pData, int length, uint32_t m);
int I2C_Mem_Read_a8( uint8_t slave_Addr, uint8_t Address, uint8_t *pData, int length);

/*
// HAL defines
#define HW_SPI_PORT_NSS			GPIOB, GPIO_PIN_6
#define HW_SPI_PORT_SCK			GPIOA, GPIO_PIN_7
#define HW_SPI_PORT_MOSI		GPIOA, GPIO_PIN_6
//#define HW_SPI_PORT_MISO		GPIOB, GPIO_PIN_4
#define HW_PORT_NRESET			GPIOA, GPIO_PIN_9
#define HW_PORT_D_OR_C			GPIOC, GPIO_PIN_7
#define HW_PORT_SET(port)		HAL_GPIO_WritePin( port, GPIO_PIN_SET )
#define HW_PORT_RESET(port)		HAL_GPIO_WritePin( port, GPIO_PIN_RESET )
#define HW_PORT_READ(port)		HAL_GPIO_ReadPin( port )
#define HW_DELAY_MS(ms)			HAL_Delay( ms )
#define HW_GET_SYS_TICKS()		HAL_GetTick()

// Timing Delays
#define SSD1331_DELAYS_HWFILL		(3)
#define SSD1331_DELAYS_HWLINE       (1)

// SSD1331 Commands
#define SSD1331_CMD_DRAWLINE 		0x21
#define SSD1331_CMD_DRAWRECT 		0x22
#define SSD1331_CMD_FILL 			0x26
#define SSD1331_CMD_SETCOLUMN 		0x15
#define SSD1331_CMD_SETROW    		0x75
#define SSD1331_CMD_CONTRASTA 		0x81
#define SSD1331_CMD_CONTRASTB 		0x82
#define SSD1331_CMD_CONTRASTC		0x83
#define SSD1331_CMD_MASTERCURRENT 	0x87
#define SSD1331_CMD_SETREMAP 		0xA0
#define SSD1331_CMD_STARTLINE 		0xA1
#define SSD1331_CMD_DISPLAYOFFSET 	0xA2
#define SSD1331_CMD_NORMALDISPLAY 	0xA4
#define SSD1331_CMD_DISPLAYALLON  	0xA5
#define SSD1331_CMD_DISPLAYALLOFF 	0xA6
#define SSD1331_CMD_INVERTDISPLAY 	0xA7
#define SSD1331_CMD_SETMULTIPLEX  	0xA8
#define SSD1331_CMD_SETMASTER 		0xAD
#define SSD1331_CMD_DISPLAYOFF 		0xAE
#define SSD1331_CMD_DISPLAYON     	0xAF
#define SSD1331_CMD_POWERMODE 		0xB0
#define SSD1331_CMD_PRECHARGE 		0xB1
#define SSD1331_CMD_CLOCKDIV 		0xB3
#define SSD1331_CMD_PRECHARGEA 		0x8A
#define SSD1331_CMD_PRECHARGEB 		0x8B
#define SSD1331_CMD_PRECHARGEC 		0x8C
#define SSD1331_CMD_PRECHARGELEVEL 	0xBB
#define SSD1331_CMD_VCOMH 			0xBE

uint16_t Color565(uint8_t r, uint8_t g, uint8_t b);
void drawPixel(int16_t x, int16_t y, uint16_t color);
void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
//void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t fillcolor);
void pushColor(uint16_t c);

// commands
void begin(void);
void goHome(void);
void goTo(int x, int y);
void reset(void);

// low level
void writeData(uint8_t d);
void writeCommand(uint8_t c);
//void writeData_unsafe(uint16_t d);
//void setWriteDir(void);
//void write8(uint8_t d);
void spiwrite(uint8_t);

void spi_csh( void );
void spi_csl( void );
char spi_xfer_byte( char data );
void device_reset( char state );
void device_set_dc( char state );
int getRotation( void );
void swap( int16_t *px, int16_t *py );
*/

#endif // __OLED_INCLUDED__

