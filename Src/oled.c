
//#include "gpio.h"
//#include "i2c.h"
//#include <project.h>
#include <string.h>
#include "fonts.h"
#include "oled.h"
#include "ssd1306.h"

/* Absolute value */
#define ABS(x)   ((x) > 0 ? (x) : -(x))

/* Write command */
#define SSD1306_WRITECOMMAND(command)      ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, (command))

/* SSD1306 data buffer */
static uint8_t SSD1306_Buffer[(SSD1306_WIDTH * SSD1306_HEIGHT / 8)];

/*=== Function for milliseconds ===*/
//#define DWT_CONTROL *(volatile unsigned long *)0xE0001000
//#define SCB_DEMCR   *(volatile unsigned long *)0xE000EDFC
//
//void DWT_Init(void)
//{
//    SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Allow to use counter
//    DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk;   // Launch the counter
//}
//
//uint32_t us_count_tic;
//
//void delay_millis(uint32_t us)
//{
//    us_count_tic =  us * (SystemCoreClock / 10000); // Obtain amount of ticks for 1usec and multiply on our value
//    DWT->CYCCNT = 0U; // Counter to zero
//    while(DWT->CYCCNT < us_count_tic);
//}


/* Private SSD1306 structure */
typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
} SSD1306_t;

/* Private variable */
static SSD1306_t SSD1306;

uint8_t TM_SSD1306_Init(void) {

	// Check if LCD connected to I2C
//	if( !SSD1306_check_device() ){
//			// Return false
//			return 0;
//	}
	// A little delay
//	HW_DELAY_MS( 100 );
        HAL_Delay(300);

	/* Init LCD */
	SSD1306_WRITECOMMAND(0xAE); //display off
	SSD1306_WRITECOMMAND(0x20); //Set Memory Addressing Mode
	SSD1306_WRITECOMMAND(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	SSD1306_WRITECOMMAND(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	SSD1306_WRITECOMMAND(0xC8); //Set COM Output Scan Direction
	SSD1306_WRITECOMMAND(0x00); //---set low column address
	SSD1306_WRITECOMMAND(0x10); //---set high column address
	SSD1306_WRITECOMMAND(0x40); //--set start line address
	SSD1306_WRITECOMMAND(0x81); //--set contrast control register
	SSD1306_WRITECOMMAND(0xFF);
	SSD1306_WRITECOMMAND(0xA1); //--set segment re-map 0 to 127
	SSD1306_WRITECOMMAND(0xA6); //--set normal display
	SSD1306_WRITECOMMAND(0xA8); //--set multiplex ratio(1 to 64)
	SSD1306_WRITECOMMAND(0x3F); //
	SSD1306_WRITECOMMAND(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	SSD1306_WRITECOMMAND(0xD3); //-set display offset
	SSD1306_WRITECOMMAND(0x00); //-not offset
	SSD1306_WRITECOMMAND(0xD5); //--set display clock divide ratio/oscillator frequency
	SSD1306_WRITECOMMAND(0xF0); //--set divide ratio
	SSD1306_WRITECOMMAND(0xD9); //--set pre-charge period
	SSD1306_WRITECOMMAND(0x22); //
	SSD1306_WRITECOMMAND(0xDA); //--set com pins hardware configuration
	SSD1306_WRITECOMMAND(0x12);
	SSD1306_WRITECOMMAND(0xDB); //--set vcomh
	SSD1306_WRITECOMMAND(0x20); //0x20,0.77xVcc
	SSD1306_WRITECOMMAND(0x8D); //--set DC-DC enable
	SSD1306_WRITECOMMAND(0x14); //
	SSD1306_WRITECOMMAND(0xAF); //--turn on SSD1306 panel

	/* Clear screen */
//	TM_SSD1306_Fill(SSD1306_COLOR_BLACK);
        TM_SSD1306_Fill(1);

	/* Update screen */
        HAL_Delay(1);
	TM_SSD1306_UpdateScreen();
        
        HAL_Delay(100);
        
        TM_SSD1306_Fill(0);
	/* Update screen */
        HAL_Delay(1);
	TM_SSD1306_UpdateScreen();
        
        HAL_Delay(100);
        
//        TM_SSD1306_Fill(1);
//	/* Update screen */
//        HAL_Delay(1);
//	TM_SSD1306_UpdateScreen();
//        
//        HAL_Delay(100);
//        
//        TM_SSD1306_Fill(0);
//	/* Update screen */
//        HAL_Delay(1);
//	TM_SSD1306_UpdateScreen();

	/* Set default values */
	SSD1306.CurrentX = 0;
	SSD1306.CurrentY = 0;

	/* Initialized OK */
	SSD1306.Initialized = 1;

	/* Return OK */
	return 1;
}

void TM_SSD1306_UpdateScreen(void) {
	uint8_t m = 0;
        for (m = 0; m < 8; m++) {
//          HAL_Delay(100);
//          printf("m = %d", m);
//          printf("\n");
//		SSD1306_WRITECOMMAND(0xB0 + m);
//		SSD1306_WRITECOMMAND(0x00);
//		SSD1306_WRITECOMMAND(0x10);

          /* Write multi data */
          SSD1306_buf_write( &SSD1306_Buffer[SSD1306_WIDTH * m], SSD1306_WIDTH, m); 
	}
}

void TM_SSD1306_ToggleInvert(void) {
	uint16_t i;

	/* Toggle invert */
	SSD1306.Inverted = !SSD1306.Inverted;

	/* Do memory toggle */
	for (i = 0; i < sizeof(SSD1306_Buffer); i++) {
		SSD1306_Buffer[i] = ~SSD1306_Buffer[i];
	}
}

void TM_SSD1306_Fill(SSD1306_COLOR_t color) {
	/* Set memory */
	memset(SSD1306_Buffer, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer));
}

void TM_SSD1306_DrawPixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color) {
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Error */
		return;
	}

	/* Check if pixels are inverted */
	if (SSD1306.Inverted) {
		color = (SSD1306_COLOR_t)!color;
	}

	/* Set color */
	if (color == SSD1306_COLOR_WHITE) {
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	} else {
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}

void TM_SSD1306_GotoXY(uint16_t x, uint16_t y) {
	/* Set write pointers */
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}

char TM_SSD1306_Putc(char ch, TM_FontDef_t* Font, SSD1306_COLOR_t color) {
	uint32_t i, b, j;

	// Check available space in LCD
	if (
		SSD1306_WIDTH <= (SSD1306.CurrentX + Font->FontWidth) ||
		SSD1306_HEIGHT <= (SSD1306.CurrentY + Font->FontHeight)
	) {
		// Error
		return 0;
	}

	// Go through font
	for (i = 0; i < Font->FontHeight; i++) {
		b = Font->data[(ch - 32) * Font->FontHeight + i];
		for (j = 0; j < Font->FontWidth; j++) {
			if ((b << j) & 0x8000) {
				TM_SSD1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR_t) color);
			} else {
				TM_SSD1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR_t)!color);
			}
		}
	}

	// Increase pointer
	SSD1306.CurrentX += Font->FontWidth;

	// Return character written
	return ch;
}

char TM_SSD1306_Puts(char* str, TM_FontDef_t* Font, SSD1306_COLOR_t color) {
	// Write characters
	while (*str) {
		// Write character by character
		if (TM_SSD1306_Putc(*str, Font, color) != *str) {
			// Return error
			return *str;
		}

		// Increase string pointer
		str++;
	}

	// Everything OK, zero should be returned
	return *str;
}

void TM_SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_COLOR_t c) {
	int16_t dx, dy, sx, sy, err, e2, i, tmp;

	/* Check for overflow */
	if (x0 >= SSD1306_WIDTH) {
		x0 = SSD1306_WIDTH - 1;
	}
	if (x1 >= SSD1306_WIDTH) {
		x1 = SSD1306_WIDTH - 1;
	}
	if (y0 >= SSD1306_HEIGHT) {
		y0 = SSD1306_HEIGHT - 1;
	}
	if (y1 >= SSD1306_HEIGHT) {
		y1 = SSD1306_HEIGHT - 1;
	}

	dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
	dy = (y0 < y1) ? (y1 - y0) : (y0 - y1);
	sx = (x0 < x1) ? 1 : -1;
	sy = (y0 < y1) ? 1 : -1;
	err = ((dx > dy) ? dx : -dy) / 2;

	if (dx == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Vertical line */
		for (i = y0; i <= y1; i++) {
			TM_SSD1306_DrawPixel(x0, i, c);
		}

		/* Return from function */
		return;
	}

	if (dy == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Horizontal line */
		for (i = x0; i <= x1; i++) {
			TM_SSD1306_DrawPixel(i, y0, c);
		}

		/* Return from function */
		return;
	}

	while (1) {
		TM_SSD1306_DrawPixel(x0, y0, c);
		if (x0 == x1 && y0 == y1) {
			break;
		}
		e2 = err;
		if (e2 > -dx) {
			err -= dy;
			x0 += sx;
		}
		if (e2 < dy) {
			err += dx;
			y0 += sy;
		}
	}
}

void TM_SSD1306_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR_t c) {
	/* Check input parameters */
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= SSD1306_WIDTH) {
		w = SSD1306_WIDTH - x;
	}
	if ((y + h) >= SSD1306_HEIGHT) {
		h = SSD1306_HEIGHT - y;
	}

	/* Draw 4 lines */
	TM_SSD1306_DrawLine(x, y, x + w, y, c);         /* Top line */
	TM_SSD1306_DrawLine(x, y + h, x + w, y + h, c); /* Bottom line */
	TM_SSD1306_DrawLine(x, y, x, y + h, c);         /* Left line */
	TM_SSD1306_DrawLine(x + w, y, x + w, y + h, c); /* Right line */
}

void TM_SSD1306_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR_t c) {
	uint8_t i;

	/* Check input parameters */
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= SSD1306_WIDTH) {
		w = SSD1306_WIDTH - x;
	}
	if ((y + h) >= SSD1306_HEIGHT) {
		h = SSD1306_HEIGHT - y;
	}

	/* Draw lines */
	for (i = 0; i <= h; i++) {
		/* Draw lines */
		TM_SSD1306_DrawLine(x, y + i, x + w, y + i, c);
	}
}

void TM_SSD1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR_t color) {
	/* Draw lines */
	TM_SSD1306_DrawLine(x1, y1, x2, y2, color);
	TM_SSD1306_DrawLine(x2, y2, x3, y3, color);
	TM_SSD1306_DrawLine(x3, y3, x1, y1, color);
}

void TM_SSD1306_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR_t color) {
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
	curpixel = 0;

	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	} else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	} else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay){
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	} else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		TM_SSD1306_DrawLine(x, y, x3, y3, color);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}

void TM_SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    TM_SSD1306_DrawPixel(x0, y0 + r, c);
    TM_SSD1306_DrawPixel(x0, y0 - r, c);
    TM_SSD1306_DrawPixel(x0 + r, y0, c);
    TM_SSD1306_DrawPixel(x0 - r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        TM_SSD1306_DrawPixel(x0 + x, y0 + y, c);
        TM_SSD1306_DrawPixel(x0 - x, y0 + y, c);
        TM_SSD1306_DrawPixel(x0 + x, y0 - y, c);
        TM_SSD1306_DrawPixel(x0 - x, y0 - y, c);

        TM_SSD1306_DrawPixel(x0 + y, y0 + x, c);
        TM_SSD1306_DrawPixel(x0 - y, y0 + x, c);
        TM_SSD1306_DrawPixel(x0 + y, y0 - x, c);
        TM_SSD1306_DrawPixel(x0 - y, y0 - x, c);
    }
}

void TM_SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    TM_SSD1306_DrawPixel(x0, y0 + r, c);
    TM_SSD1306_DrawPixel(x0, y0 - r, c);
    TM_SSD1306_DrawPixel(x0 + r, y0, c);
    TM_SSD1306_DrawPixel(x0 - r, y0, c);
    TM_SSD1306_DrawLine(x0 - r, y0, x0 + r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        TM_SSD1306_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, c);
        TM_SSD1306_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, c);

        TM_SSD1306_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, c);
        TM_SSD1306_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, c);
    }
}

//-----------------------------------------------------------------------------
/// OLED display checking function
//-----------------------------------------------------------------------------
//int SSD1306_check_device( void )
//{
//	uint32_t stat;
//	
//	// 3C << 1 => stat = I2C_I2C_MSTR_ERR_LB_NAK
//	// 3C => stat = I2C_I2C_MSTR_NO_ERROR
//	stat = I2C_I2CMasterSendStart( SSD1306_I2C_HAL_ADDR, I2C_I2C_READ_XFER_MODE );
//	if( stat != I2C_I2C_MSTR_NO_ERROR ){
//		return 0;
//	}
//	I2C_I2CMasterSendStop();
//	return 1;
//}

//-----------------------------------------------------------------------------
/// OLED display write command function
//-----------------------------------------------------------------------------
/*int SSD1306_WRITECOMMAND( uint8_t command )
{
	uint8_t tbuf[2];
	uint32_t stat;
	
	tbuf[0] = 0x00;
	tbuf[1] = command;
	stat = I2C_I2CMasterWriteBuf( SSD1306_I2C_HAL_ADDR, tbuf, 2, I2C_I2C_MODE_COMPLETE_XFER );
	if( stat != I2C_I2C_MSTR_NO_ERROR ){
		I2C_I2CMasterSendStop();
		return 0;
	}
	while( !(I2C_I2CMasterStatus() & I2C_I2C_MSTAT_WR_CMPLT) ); // Wait till all bits are transmitted 
	//I2C_Mem_Write_a8( SSD1306_I2C_HAL_ADDR, 0x00, &command, 1 );
	return 1;
}
*/

//-----------------------------------------------------------------------------
/// OLED display write data function
//-----------------------------------------------------------------------------
/*int SSD1306_WRITEDATA( uint8_t data )
{
	uint8_t tbuf[2];
	uint32 stat;
	
	tbuf[0] = 0x40;
	tbuf[1] = data;
	stat = I2C_I2CMasterWriteBuf( SSD1306_I2C_HAL_ADDR, tbuf, 2, I2C_I2C_MODE_COMPLETE_XFER );
	if( stat != I2C_I2C_MSTR_NO_ERROR ){
		I2C_I2CMasterSendStop();
		return 0;
	}
	while( !(I2C_I2CMasterStatus() & I2C_I2C_MSTAT_WR_CMPLT) ); // Wait till all bits are transmitted 
	//I2C_Mem_Write_a8( SSD1306_I2C_HAL_ADDR, 0x40, &data, 1 );
	return 1;
}
*/

//-----------------------------------------------------------------------------
/// OLED display buffer write function
//-----------------------------------------------------------------------------
void SSD1306_buf_write( uint8_t *pbuf, int length, uint32_t m)
{
	/*
	uint32 stat;
	
	memmove( &pbuf[1], pbuf, length );
	pbuf[0] = 0x40;
	stat = I2C_I2CMasterWriteBuf( SSD1306_I2C_HAL_ADDR, pbuf, length+1, I2C_I2C_MODE_COMPLETE_XFER );
	//	if( stat != I2C_I2C_MSTR_NO_ERROR ){
	//		I2C_I2CMasterSendStop();
	//		return 0;
	//	}
	while( !(I2C_I2CMasterStatus() & I2C_I2C_MSTAT_WR_CMPLT) ); // Wait till all bits are transmitted 
	*/
	I2C_Mem_Write_a8( SSD1306_I2C_HAL_ADDR, 0x40, pbuf, length, m);
}

//-----------------------------------------------------------------------------
/// I2C transfer with address function
//-----------------------------------------------------------------------------
int I2C_Mem_Write_a8( uint8_t slave_Addr, uint8_t Address, uint8_t *pData, int length, uint32_t m)
{  
//  printf("Length: %d\n", length);
  
  uint8_t median = length / 2;
//  printf("Median: %d\n", median);
  uint8_t dt_0[SSD1306_WIDTH + 1];
  uint8_t dt_1[SSD1306_WIDTH / 2 + 2];
  uint8_t dt_2[SSD1306_WIDTH / 2 + 2];
  
  dt_0[0] = Address;
  dt_1[0] = Address;
  dt_2[0] = Address;
  
  uint8_t i;
  for (i = 1; i <= length/2; i++) {
    dt_0[i] = pData[i-1];
    dt_1[i] = pData[i-1];
    dt_2[i] = pData[median+i-1];
  }
  
  SSD1306_WRITECOMMAND(0xB0 + m);
  SSD1306_WRITECOMMAND(0x00);
  SSD1306_WRITECOMMAND(0x10);
  
  HAL_I2C_Master_Transmit(&hi2c1, slave_Addr, dt_1, median+1, 15);
  
  
  SSD1306_WRITECOMMAND(0xB0 + m);
  SSD1306_WRITECOMMAND(0x00);
  SSD1306_WRITECOMMAND(0x14);
  HAL_I2C_Master_Transmit(&hi2c1, slave_Addr, dt_2, median+1, 15);

        /*
	int i;
	uint32_t stat;
	
	I2C_I2CMasterClearStatus(); // Clear the status register of I2C master
	stat = I2C_I2CMasterSendStart( slave_Addr, I2C_I2C_WRITE_XFER_MODE ); // Returns a non zero if slave device ACKs
	if( stat != I2C_I2C_MSTR_NO_ERROR ){
		return 0;
	}
	stat = I2C_I2CMasterWriteByte( Address );
	if( stat != I2C_I2C_MSTR_NO_ERROR ){
		I2C_I2CMasterSendStop();
		return 0;
	}
	//	stat = I2C_I2CMasterWriteBuf( slave_Addr, pData, length, I2C_I2C_MODE_COMPLETE_XFER );
	//	if( stat != I2C_I2C_MSTR_NO_ERROR ){
	//		I2C_I2CMasterSendStop();
	//		return 0;
	//	}
	//	while( !(I2C_I2CMasterStatus() & I2C_I2C_MSTAT_WR_CMPLT) ); // Wait till all bits are transmitted 
	for( i=0; i<length; i++ ){
		stat = I2C_I2CMasterWriteByte( *pData++ );
		if( stat != I2C_I2C_MSTR_NO_ERROR ){
			I2C_I2CMasterSendStop();
			return 0;
		}
	}
	I2C_I2CMasterSendStop();
	return 1;
	//I2C_I2CMasterReadBuf( I2C_ADDRESS, Current_time, NO_OF_BYTES, (I2C_I2C_MODE_REPEAT_START | I2C_I2C_MODE_COMPLETE_XFER) );	
	//I2C_I2CMasterWriteBuf( slave_Addr, (uint8 *)pData, length, (I2C_I2C_MODE_REPEAT_START | I2C_I2C_MODE_COMPLETE_XFER) );
	//I2C_I2CMasterWriteBuf( slave_Addr, &Address, 1, I2C_I2C_MODE_NO_STOP );
	//while( !(I2C_I2CMasterStatus() & I2C_I2C_MSTAT_WR_CMPLT) ); // Wait till all bits are transmitted 
	//I2C_I2CMasterClearStatus(); // Clear the status register of I2C master
	//I2C_I2CMasterWriteBuf( slave_Addr, (uint8 *)pData, length, (I2C_I2C_MODE_REPEAT_START | I2C_I2C_MODE_COMPLETE_XFER) );
	//while( !(I2C_I2CMasterStatus() & I2C_I2C_MSTAT_WR_CMPLT) ); // Wait till all bits are transmitted 
        */
} 

//-----------------------------------------------------------------------------
/// I2C address read function
//-----------------------------------------------------------------------------
//int I2C_Mem_Read_a8( uint8_t slave_Addr, uint8_t Address, uint8_t *pData, int length )
//{
//	uint32_t stat;
//
//	I2C_I2CMasterClearStatus(); // Clear the status register of I2C master
//	stat = I2C_I2CMasterSendStart( slave_Addr, I2C_I2C_WRITE_XFER_MODE ); // Returns a non zero if slave device ACKs
//	if( stat != I2C_I2C_MSTR_NO_ERROR ){
//		return 0;
//	}
//	stat = I2C_I2CMasterWriteByte( Address );
//	if( stat != I2C_I2C_MSTR_NO_ERROR ){
//		I2C_I2CMasterSendStop();
//		return 0;
//	}
//	stat = I2C_I2CMasterReadBuf( slave_Addr, pData, length, (I2C_I2C_MODE_REPEAT_START | I2C_I2C_MODE_COMPLETE_XFER) );	
//	return 1;
//}

void SSD1306_ON(void) {
	SSD1306_WRITECOMMAND(0x8D);
	SSD1306_WRITECOMMAND(0x14);
	SSD1306_WRITECOMMAND(0xAF);
}

void SSD1306_OFF(void) {
	SSD1306_WRITECOMMAND(0x8D);
	SSD1306_WRITECOMMAND(0x10);
	SSD1306_WRITECOMMAND(0xAE);
}

/********************************** low level pin interface */
/*
//------------------------------------------------------------------------------
/// Set NSS HIGH
//------------------------------------------------------------------------------
void spi_csh( void )
{
	HW_PORT_SET( HW_SPI_PORT_NSS );
}

//------------------------------------------------------------------------------
/// Set NSS LOW
//------------------------------------------------------------------------------
void spi_csl( void )
{
	HW_PORT_RESET( HW_SPI_PORT_NSS );
}

//------------------------------------------------------------------------------
/// Transfer 1 byte via SPI
//------------------------------------------------------------------------------
char spi_xfer_byte( char data )
{
	int i;
	char rxbyte;

	HW_PORT_RESET( HW_SPI_PORT_SCK );
	rxbyte = 0;
	for( i=0; i<8; i++ ){
		// Move rezult by 1 bit
		rxbyte <<= 1;
		// Form SCK falling edge
		HW_PORT_RESET( HW_SPI_PORT_SCK );
		// Change data pin
		if( data & 0x80 ){
			HW_PORT_SET( HW_SPI_PORT_MOSI );
		} else {
			HW_PORT_RESET( HW_SPI_PORT_MOSI );
		}
		// Form SCK rising edge
		HW_PORT_SET( HW_SPI_PORT_SCK );
		// Get data from input pin
#ifdef HW_SPI_PORT_MISO
		rxbyte |= HW_PORT_READ( HW_SPI_PORT_MISO ) & 0x01;
#else
		rxbyte = 0;
#endif
		// Move by 1 bit
		data <<= 1;
	}
	// Return received value
	return rxbyte;
}

//------------------------------------------------------------------------------
/// Set ~RESET signal low or high
/// state = 1 - ~RESET=0, state = 0 - ~RESET = 1
//------------------------------------------------------------------------------
void device_reset( char state )
{
	if( state ){
		HW_PORT_RESET( HW_PORT_NRESET );
	} else {
		HW_PORT_SET( HW_PORT_NRESET );
	}
}

//------------------------------------------------------------------------------
/// Set D/C (or RS) signal low or high
/// state = 0 - D/C=0, state = 1 - D/C = 1
//------------------------------------------------------------------------------
void device_set_dc( char state )
{
	if( !state ){
		HW_PORT_RESET( HW_PORT_D_OR_C );
	} else {
		HW_PORT_SET( HW_PORT_D_OR_C );
	}
}

void spiwrite(uint8_t c)
{
	spi_xfer_byte( c );
}

void writeCommand(uint8_t c)
{
	device_set_dc(0);
	spi_csl();
	spiwrite(c);
	spi_csh();
}


void writeData(uint8_t c)
{
	device_set_dc(1);
	spi_csl();
	spiwrite(c);
	spi_csh();
}

void goHome(void)
{
	goTo(0,0);
}

void goTo(int x, int y)
{
	if( (x >= OLED_MAXWIDTH) || (y >= OLED_MAXHEIGHT) ){
		return;
	}

	// set x and y coordinate
	writeCommand(SSD1331_CMD_SETCOLUMN);
	writeCommand(x);
	writeCommand(OLED_MAXWIDTH-1);

	writeCommand(SSD1331_CMD_SETROW);
	writeCommand(y);
	writeCommand(OLED_MAXHEIGHT-1);
}

uint16_t Color565( uint8_t r, uint8_t g, uint8_t b )
{
	uint16_t c;

	c = r >> 3;
	c <<= 6;
	c |= g >> 2;
	c <<= 5;
	c |= b >> 3;
	return c;
}

///    @brief  Draws a filled rectangle using HW acceleration
void Adafruit_SSD1331::fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t fillcolor)
{
//Serial.println("fillRect");
  // check rotation, move rect around if necessary
  switch (getRotation()) {
  case 1:
    swap(x, y);
    swap(w, h);
    x = WIDTH - x - 1;
    break;
  case 2:
    x = WIDTH - x - 1;
    y = HEIGHT - y - 1;
    break;
  case 3:
    swap(x, y);
    swap(w, h);
    y = HEIGHT - y - 1;
    break;
  }
  // Bounds check
  if ((x >= TFTWIDTH) || (y >= TFTHEIGHT))
	return;
  // Y bounds check
  if (y+h > TFTHEIGHT)
  {
    h = TFTHEIGHT - y;
  }
  // X bounds check
  if (x+w > TFTWIDTH)
  {
    w = TFTWIDTH - x;
  }

  // fill!
  writeCommand(SSD1331_CMD_FILL);
  writeCommand(0x01);
  writeCommand(SSD1331_CMD_DRAWRECT);
  writeCommand(x & 0xFF);							// Starting column
  writeCommand(y & 0xFF);							// Starting row
  writeCommand((x+w-1) & 0xFF);	// End column
  writeCommand((y+h-1) & 0xFF);	// End row

  // Outline color
  writeCommand((uint8_t)((fillcolor >> 11) << 1));
  writeCommand((uint8_t)((fillcolor >> 5) & 0x3F));
  writeCommand((uint8_t)((fillcolor << 1) & 0x3F));
  // Fill color
  writeCommand((uint8_t)((fillcolor >> 11) << 1));
  writeCommand((uint8_t)((fillcolor >> 5) & 0x3F));
  writeCommand((uint8_t)((fillcolor << 1) & 0x3F));

  // Delay while the fill completes
  HW_DELAY_MS(SSD1331_DELAYS_HWFILL);
}

void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
	// check rotation, move pixel around if necessary
	switch (getRotation()) {
	case 1:
		swap(&x0, &y0);
		swap(&x1, &y1);
		x0 = OLED_MAXWIDTH - x0 - 1;
		x1 = OLED_MAXWIDTH - x1 - 1;
		break;
	case 2:
		x0 = OLED_MAXWIDTH - x0 - 1;
		y0 = OLED_MAXHEIGHT - y0 - 1;
		x1 = OLED_MAXWIDTH - x1 - 1;
		y1 = OLED_MAXHEIGHT - y1 - 1;
		break;
	case 3:
		swap(&x0, &y0);
		swap(&x1, &y1);
		y0 = OLED_MAXHEIGHT - y0 - 1;
		y1 = OLED_MAXHEIGHT - y1 - 1;
		break;
	}

	// Boundary check
	if ((y0 >= OLED_MAXHEIGHT) && (y1 >= OLED_MAXHEIGHT))
		return;
	if ((x0 >= OLED_MAXWIDTH) && (x1 >= OLED_MAXWIDTH))
		return;
	if (x0 >= OLED_MAXWIDTH)
		x0 = OLED_MAXWIDTH - 1;
	if (y0 >= OLED_MAXHEIGHT)
		y0 = OLED_MAXHEIGHT - 1;
	if (x1 >= OLED_MAXWIDTH)
		x1 = OLED_MAXWIDTH - 1;
	if (y1 >= OLED_MAXHEIGHT)
		y1 = OLED_MAXHEIGHT - 1;

	writeCommand(SSD1331_CMD_DRAWLINE);
	writeCommand(x0);
	writeCommand(y0);
	writeCommand(x1);
	writeCommand(y1);
	HW_DELAY_MS(SSD1331_DELAYS_HWLINE);
	writeCommand((uint8_t)((color >> 11) << 1));
	writeCommand((uint8_t)((color >> 5) & 0x3F));
	writeCommand((uint8_t)((color << 1) & 0x3F));
	HW_DELAY_MS(SSD1331_DELAYS_HWLINE);
}

void drawPixel(int16_t x, int16_t y, uint16_t color)
{
	if ((x < 0) || (x >= OLED_MAXWIDTH) || (y < 0) || (y >= OLED_MAXHEIGHT)) return;

	// check rotation, move pixel around if necessary
	switch (getRotation()) {
	case 1:
		swap(&x, &y);
		x = OLED_MAXWIDTH - x - 1;
		break;
	case 2:
		x = OLED_MAXWIDTH - x - 1;
		y = OLED_MAXHEIGHT - y - 1;
		break;
	case 3:
		swap(&x, &y);
		y = OLED_MAXHEIGHT - y - 1;
		break;
	}

	goTo(x, y);
	pushColor(color);
}

void pushColor(uint16_t color)
{
	// setup for data
	device_set_dc( 1 );
	spi_csl();
	spiwrite(color >> 8);
	spiwrite(color);
	spi_csh();
}


void begin(void)
{
	// Toggle RST low to reset; CS low so it'll listen to us
	spi_csl();
	device_reset( 0 );
	HW_DELAY_MS( 500 );
	device_reset( 1 );
	HW_DELAY_MS( 500 );
	device_reset( 0 );
	HW_DELAY_MS( 500 );

	// Initialization Sequence
	writeCommand(SSD1331_CMD_DISPLAYOFF);  	// 0xAE
	writeCommand(SSD1331_CMD_SETREMAP); 	// 0xA0
#if defined SSD1331_COLORORDER_RGB
	writeCommand(0x72);				// RGB Color
#else
	writeCommand(0x76);				// BGR Color
#endif
	writeCommand(SSD1331_CMD_STARTLINE); 	// 0xA1
	writeCommand(0x0);
	writeCommand(SSD1331_CMD_DISPLAYOFFSET); 	// 0xA2
	writeCommand(0x0);
	writeCommand(SSD1331_CMD_NORMALDISPLAY);  	// 0xA4
	writeCommand(SSD1331_CMD_SETMULTIPLEX); 	// 0xA8
	writeCommand(0x3F);  			// 0x3F 1/64 duty
	writeCommand(SSD1331_CMD_SETMASTER);  	// 0xAD
	writeCommand(0x8E);
	writeCommand(SSD1331_CMD_POWERMODE);  	// 0xB0
	writeCommand(0x0B);
	writeCommand(SSD1331_CMD_PRECHARGE);  	// 0xB1
	writeCommand(0x31);
	writeCommand(SSD1331_CMD_CLOCKDIV);  	// 0xB3
	writeCommand(0xF0);  // 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(SSD1331_CMD_PRECHARGEA);  	// 0x8A
	writeCommand(0x64);
	writeCommand(SSD1331_CMD_PRECHARGEB);  	// 0x8B
	writeCommand(0x78);
	writeCommand(SSD1331_CMD_PRECHARGEA);  	// 0x8C
	writeCommand(0x64);
	writeCommand(SSD1331_CMD_PRECHARGELEVEL);  	// 0xBB
	writeCommand(0x3A);
	writeCommand(SSD1331_CMD_VCOMH);  		// 0xBE
	writeCommand(0x3E);
	writeCommand(SSD1331_CMD_MASTERCURRENT);  	// 0x87
	writeCommand(0x06);
	writeCommand(SSD1331_CMD_CONTRASTA);  	// 0x81
	writeCommand(0x91);
	writeCommand(SSD1331_CMD_CONTRASTB);  	// 0x82
	writeCommand(0x50);
	writeCommand(SSD1331_CMD_CONTRASTC);  	// 0x83
	writeCommand(0x7D);
	writeCommand(SSD1331_CMD_DISPLAYON);	//--turn on oled panel
}

int getRotation( void )
{
	return 2;
}

void swap( int16_t *px, int16_t *py )
{
	int16_t z;

	z = *px;
	*px = *py;
	*py = z;
}
*/
