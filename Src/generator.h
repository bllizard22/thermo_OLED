
#ifndef __GENERATOR_INCLUDED__
#define __GENERATOR_INCLUDED__

//--- Configuration Defines ----------------------------------------------------
#define GENERATOR_MIN_FREQUENCY		5000
#define GENERATOR_MAX_FREQUENCY		9000

#define GENERATOR_MIN_AMPLITUDE		3000

//--- Basic Defines ------------------------------------------------------------
#define GENERATOR_TIMER_HANDLE		htim1
#define GENERATOR_TIMER_CHANNEL		TIM_CHANNEL_1
#define ADC_TIMER_HANDLE			htim2

typedef enum {
	STAT_NO_INIT 		= 0x00,
	STAT_RUN 			= 0x01,
	STAT_STOP 			= 0x02,
	STAT_ERR_NO_PLUS	= 0x11,
	STAT_ERR_NO_MINUS	= 0x12,
	STAT_ERR_NO_OUT 	= 0x13
} TGenStatus;

//--- Public Variables ---------------------------------------------------------
extern volatile uint32_t frequency;

//--- Function Definitions -----------------------------------------------------
void gen_init( void );
void gen_process( void );
void gen_start( uint32_t freq_hz );
void gen_stop( void );
void gen_set_frequency( uint32_t freq_hz );
uint32_t	gen_get_frequency( void );
uint8_t		gen_get_status( void );

#endif // __GENERATOR_INCLUDED__