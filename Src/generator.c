
#include <string.h>

#include "iwdg.h"
#include "adc.h"
#include "tim.h"

#include "inbus.h"
#include "led_stripe.h"
#include "generator.h"

//#define USE_TEST_MODE		1

#define ADC_NUM_CHANNELS	3
#define ADC_MAX_SAMPLES		150
#define FREQ_AVERAGING		480//1000
#define FREQ_HCLK_HZ		72000000
#define FREQ_MAX_ERROR_HZ	5
#define STARTING_WAIT_MS	5000

volatile __no_init uint32_t frequency;
uint32_t out_frequency;
volatile char is_freq_updated = 0;
volatile int smp_idx;
uint32_t mfreqa, mfreqb;
int mfreq_avg;
int16_t adc_data[ADC_NUM_CHANNELS][ADC_MAX_SAMPLES];
volatile char adc_ready_flag;
uint32_t starter;
uint8_t gen_status;
uint32_t ma, mb;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void gen_init( void )
{
	// Init the variables
	adc_ready_flag = 0;
	smp_idx = 0;
	mfreqa = mfreqb = 0;
	mfreq_avg = 0;
	out_frequency = 0;
	ma = mb = 0;
	gen_status = STAT_NO_INIT;
	memset( (void *)adc_data, 0, sizeof(adc_data) );
	// Set the initial state of pins
	//	HAL_GPIO_WritePin( GEN_Q_SET_GPIO_Port, GEN_Q_SET_Pin, GPIO_PIN_RESET );	// = 0
	//	HAL_GPIO_WritePin( GEN_I_SET_GPIO_Port, GEN_I_SET_Pin, GPIO_PIN_SET );		// = 1
	starter = HAL_GetTick() + STARTING_WAIT_MS;
	// Init LED
	led_init();

	// Check if the system has resumed from WDG reset
	if( __HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET || __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET ){
		// WDG reset flags is on
		if( frequency != 0 && frequency < GENERATOR_MIN_FREQUENCY ) frequency = GENERATOR_MIN_FREQUENCY;
		if( frequency > GENERATOR_MAX_FREQUENCY ) frequency = GENERATOR_MAX_FREQUENCY;
		// Setup the output frequency
		gen_start( frequency );
		led_set_state( 60, 32 );		// Yellow
		// Clear reset flags
		__HAL_RCC_CLEAR_RESET_FLAGS();
	} else {
		// First start: reset the frequency variable
		gen_set_frequency( 0 );
		led_set_state( 0, 32 );		// Red
	}
	// Start the ADC works
	HAL_ADCEx_InjectedStart_IT( &hadc1 );
	HAL_TIM_Base_Start( &ADC_TIMER_HANDLE );
	// Start the IWDG
	//HAL_IWDG_Start( &hiwdg );
	HAL_IWDG_Refresh( &hiwdg );
	// Start CAN receiving
	inbus_init();
#ifdef	USE_TEST_MODE
	gen_set_frequency( 5000 );
#endif	// USE_TEST_MODE
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void gen_process( void )
{
	int i, imaxa, demaxa, cnta;
	int imaxb, demaxb, cntb;
	char growa, growb;

	if( is_freq_updated ){
		// Update the frequency value and restart the generator
		is_freq_updated = 0;
		if( frequency ){
			gen_start( frequency );
		} else {
			// Stop the generator if the frequency is NULL
			gen_stop();
		}
	}
	if( adc_ready_flag ){
		// Calculate the frequency by both channels
		adc_ready_flag = 0;
		imaxa = imaxb = 0;
		cnta = cntb = 0;
		growa = growb = 0;
		demaxa = demaxb = 0;
		for( i=0; i<ADC_MAX_SAMPLES-1; i++ ){
			if( adc_data[0][i] <= adc_data[0][i+1] ){
				// Function is growing up
				growa = 1;
			}
			if( adc_data[1][i] <= adc_data[1][i+1] ){
				// Function is growing up
				growb = 1;
			}
			if( adc_data[0][i] > adc_data[0][i+1] ){
				// Function is falling down
				if( growa ){
					growa = 0;
					if( imaxa ){
						demaxa += i - imaxa;
						++cnta;
					}
					imaxa = i;
				}
			}
			if( adc_data[1][i] > adc_data[1][i+1] ){
				// Function is falling down
				if( growb ){
					growb = 0;
					if( imaxb ){
						demaxb += i - imaxb;
						++cntb;
					}
					imaxb = i;
				}
			}
		}
		// Calculate the single frequency measure
		demaxa *= 1000;
		demaxa /= cnta;
		demaxb *= 1000;
		demaxb /= cntb;
		// Calculate the averaged frequency
		mfreqa += demaxa;
		mfreqb += demaxb;
		// Calculate the averaged amplitude
		ma += adc_data[0][imaxa];
		mb += adc_data[1][imaxb];
		// Increment the counter
		++mfreq_avg;
		if( mfreq_avg >= FREQ_AVERAGING ){
			//printf( "t=%d\n", HAL_GetTick() );
			//printf( "ma = %d, mb = %d\n", ma / mfreq_avg, mb / mfreq_avg );
			ma /= mfreq_avg;
			mb /= mfreq_avg;
			if( ma < GENERATOR_MIN_AMPLITUDE ){
				gen_status = STAT_ERR_NO_PLUS;
			}
			if( mb < GENERATOR_MIN_AMPLITUDE ){
				gen_status &= 0xFD;
				gen_status |= STAT_ERR_NO_MINUS;
			}
			ma = mb = 0;
			mfreqa /= mfreq_avg;
			mfreqb /= mfreq_avg;
			mfreqa = FREQ_HCLK_HZ / mfreqa;
			mfreqb = FREQ_HCLK_HZ / mfreqb;
			out_frequency = mfreqa + mfreqb;
			out_frequency /= 2;
			mfreqa = mfreqb = 0;
			mfreq_avg = 0;
			// Check for correct frequency value
			if( frequency ){
				i = out_frequency - frequency;
				if( i < -FREQ_MAX_ERROR_HZ || i > FREQ_MAX_ERROR_HZ ){
					// Out of error or too big error
					led_set_state( 0, 32 );	// Red
				} else {
					// Normal value
					led_set_state( 120, 32 );	// Green
				}
			} else {
				// Generator is off
				led_set_state( 240, 32 );	// Blue
			}
		}
	}
	inbus_process();
	HAL_IWDG_Refresh( &hiwdg );
	if( starter ){
		if( starter < HAL_GetTick() ){
			starter = 0;
			HAL_GPIO_WritePin( GEN_Q_SET_GPIO_Port, GEN_Q_SET_Pin, GPIO_PIN_SET );	// = 1
			HAL_GPIO_WritePin( GEN_I_SET_GPIO_Port, GEN_I_SET_Pin, GPIO_PIN_RESET );// = 0
		}
	}
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void gen_start( uint32_t freq_hz )
{
	TIM_OC_InitTypeDef sConfigOC;
	uint32_t period;
	int32_t error;

	// Stop the timer
	gen_stop();
	// Calculate the timer period and the error
	if( freq_hz < GENERATOR_MIN_FREQUENCY ) freq_hz = GENERATOR_MIN_FREQUENCY;
	if( freq_hz > GENERATOR_MAX_FREQUENCY ) freq_hz = GENERATOR_MAX_FREQUENCY;
	period = error = HAL_RCC_GetHCLKFreq();
	period /= freq_hz;
	error *= 10;
	error /= period;
	error -= freq_hz * 10;
	// Reconfigure the timer
	GENERATOR_TIMER_HANDLE.Init.Prescaler = 0;
	GENERATOR_TIMER_HANDLE.Init.Period = period - 1;
	if( HAL_TIM_Base_Init(&GENERATOR_TIMER_HANDLE) != HAL_OK ){
		Error_Handler();
	}
	// Reconfigure the PWM output
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = period / 2;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if( HAL_TIM_PWM_ConfigChannel(&GENERATOR_TIMER_HANDLE, &sConfigOC, GENERATOR_TIMER_CHANNEL) != HAL_OK ){
		Error_Handler();
	}
	// Start the timer
	HAL_TIM_PWM_Start( &GENERATOR_TIMER_HANDLE, GENERATOR_TIMER_CHANNEL );
	HAL_TIMEx_PWMN_Start( &GENERATOR_TIMER_HANDLE, GENERATOR_TIMER_CHANNEL );
	gen_status = STAT_RUN;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void gen_stop( void )
{
	HAL_TIMEx_PWMN_Stop( &GENERATOR_TIMER_HANDLE, GENERATOR_TIMER_CHANNEL );
	HAL_TIM_PWM_Stop( &GENERATOR_TIMER_HANDLE, GENERATOR_TIMER_CHANNEL );
	gen_status = STAT_STOP;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void gen_set_frequency( uint32_t freq_hz )
{
	if( freq_hz != 0 && freq_hz < GENERATOR_MIN_FREQUENCY ) freq_hz = GENERATOR_MIN_FREQUENCY;
	if( freq_hz > GENERATOR_MAX_FREQUENCY ) freq_hz = GENERATOR_MAX_FREQUENCY;
	frequency = freq_hz;
	is_freq_updated = 1;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
uint32_t gen_get_frequency( void )
{
	return out_frequency;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
uint8_t		gen_get_status( void )
{
	return gen_status;
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
//{
//	adc_data[chan_idx++][smp_idx] = HAL_ADC_GetValue( AdcHandle );
//	if( chan_idx >= ADC_NUM_CHANNELS ){
//		chan_idx = 0;
//		++smp_idx;
//		if( smp_idx >= ADC_MAX_SAMPLES ){
//			smp_idx = 0;
//		}
//	}
//}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void  HAL_ADCEx_InjectedConvCpltCallback( ADC_HandleTypeDef* hadc )
{
	adc_data[0][smp_idx] = HAL_ADCEx_InjectedGetValue( hadc, ADC_INJECTED_RANK_1 );
	adc_data[1][smp_idx] = HAL_ADCEx_InjectedGetValue( hadc, ADC_INJECTED_RANK_2 );
	adc_data[2][smp_idx] = HAL_ADCEx_InjectedGetValue( hadc, ADC_INJECTED_RANK_3 );
	++smp_idx;
	if( smp_idx >= ADC_MAX_SAMPLES ){
		smp_idx = 0;
		adc_ready_flag = 1;
	}
	HAL_ADCEx_InjectedStart_IT( &hadc1 );
}

