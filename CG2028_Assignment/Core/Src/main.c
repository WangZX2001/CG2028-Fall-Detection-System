/******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * (c) CG2028 Teaching Team
 ******************************************************************************/

/*--------------------------- Includes ---------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"

#include "stdio.h"
#include "string.h"
#include <sys/stat.h>
#include <math.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"

static void UART1_Init(void);
static void Buzzer_Init(void);
static void I2C1_Init(void);

extern void initialise_monitor_handles(void); // for semi-hosting support (printf). Will not be required if transmitting via UART
extern int mov_avg(int N, int *accel_buff);	  // asm implementation
int mov_avg_C(int N, int *accel_buff);		  // Reference C implementation

typedef enum
{
	STATE_IDLE = 0,
	STATE_FREE_FALL = 1,
	STATE_IMPACT_WAIT = 2,
	STATE_FALLEN_INACTIVE = 3

} FallState_t;

const char *get_fall_state_name(FallState_t state);
float altitude_rate(float altitude, uint32_t tick_ms);

UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1; // Handle for the I2C bus

int main(void)
{
	/*--------------------------- Constants ---------------------------------------*/
	const int N = 4;
	int delay_ms = 10;			// change delay time to suit your code
	int INACTIVE_PERIOD = 5000; // gyro waiting time

	const float GYRO__INACTIVE_THRESHOLD = 30.0f;
	const uint32_t FAST_BLINK_MS = 150;	 // 150ms toggle for fast
	const uint32_t SLOW_BLINK_MS = 1500; // 1500ms toggle for slow
	const float FREE_FALL_THRESHOLD = 3.0f;
	const float ACCEL_IMPACT_THRESHOLD = 12.5f;
	const float FALL_HEIGHT_THRESHOLD = 0.4f; // Minimum 0.4 meters drop required
	const float MOVING_THRESHOLD = 10.7f;

	/*--------------------------- INIT ---------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the SYSTICK. */
	HAL_Init();

	/* UART initialization  */
	UART1_Init();
	/* Peripheral initializations using BSP functions */
	BSP_LED_Init(LED2);
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();

	Buzzer_Init();
	I2C1_Init();
	ssd1306_Init();
	ssd1306_Fill(Black); // Clear the screen
	ssd1306_UpdateScreen();
	BSP_PSENSOR_Init();
	/*Set the initial LED state to off*/
	BSP_LED_Off(LED2);

	/*--------------------------- Buffers/Index ---------------------------------------*/
	int accel_buff_x[4] = {0};
	int accel_buff_y[4] = {0};
	int accel_buff_z[4] = {0};
	int i = 0;

	/*--------------------------- State Machine and Time Constant --------------------*/
	uint32_t free_fall_start_time = 0;
	uint32_t impact_time = 0;
	uint32_t last_led_toggle_time = 0;
	uint32_t current_time = 0;
	uint32_t blink_interval = 0;

	FallState_t fall_state = STATE_IDLE;
	int tumble_detected = 0; // 0 = stable, 1 = chaotic rotation detected
	const float TUMBLE_THRESHOLD = 150.0f; // Degrees per second (dps)

	int rapid_alt_event = 0;
	const float ALT_RATE_THRESHOLD = 0.3f; // m/s (tune)

	/*--------------------------- Sensors and Computed Values --------------------*/
	float accel_mag = 0.0f;
	float g_mag = 0.0f;

	float start_altitude = 0.0f;
	float height_dropped_at_impact = 0.0f;

	static float p_filt = 1013.25f;
	const float alpha = 0.95f;

	while (1)
	{
		float p_raw = BSP_PSENSOR_ReadPressure();
		p_filt = alpha * p_filt + (1.0f - alpha) * p_raw;

		float altitude = 44330.0f * (1.0f - powf(p_filt / 1013.25f, 0.1903f));
		float rate_of_change_alt = altitude_rate(altitude, HAL_GetTick());

		// BSP_LED_Toggle(LED2);		// This function helps to toggle the current LED state

		int16_t accel_data_i16[3] = {0}; // array to store the x, y and z readings of accelerometer
		/********Function call to read accelerometer values*********/
		BSP_ACCELERO_AccGetXYZ(accel_data_i16);

		// Copy the values over to a circular style buffer
		accel_buff_x[i % 4] = accel_data_i16[0]; // acceleration along X-Axis
		accel_buff_y[i % 4] = accel_data_i16[1]; // acceleration along Y-Axis
		accel_buff_z[i % 4] = accel_data_i16[2]; // acceleration along Z-Axis

		// ********* Read gyroscope values *********/
		float gyro_data[3] = {0.0};
		float *ptr_gyro = gyro_data;
		BSP_GYRO_GetXYZ(ptr_gyro);

		// The output of gyro has been made to display in dps(degree per second)
		float gyro_velocity[3] = {0.0};
		gyro_velocity[0] = (gyro_data[0] / (1000));
		gyro_velocity[1] = (gyro_data[1] / (1000));
		gyro_velocity[2] = (gyro_data[2] / (1000));

		// Preprocessing the filtered outputs  The same needs to be done for the output from the C program as well
		float accel_filt_asm[3] = {0}; // final value of filtered acceleration values

		accel_filt_asm[0] = (float)mov_avg(N, accel_buff_x) * (9.8 / 1000.0f);
		accel_filt_asm[1] = (float)mov_avg(N, accel_buff_y) * (9.8 / 1000.0f);
		accel_filt_asm[2] = (float)mov_avg(N, accel_buff_z) * (9.8 / 1000.0f);

		// Preprocessing the filtered outputs  The same needs to be done for the output from the assembly program as well
		float accel_filt_c[3] = {0};

		accel_filt_c[0] = (float)mov_avg_C(N, accel_buff_x) * (9.8 / 1000.0f);
		accel_filt_c[1] = (float)mov_avg_C(N, accel_buff_y) * (9.8 / 1000.0f);
		accel_filt_c[2] = (float)mov_avg_C(N, accel_buff_z) * (9.8 / 1000.0f);

		/***************************UART transmission*******************************************/
		char buffer[150]; // Create a buffer large enough to hold the text

		/******Transmitting results of C execution over UART*********/
		if (i >= 3)
		{
		    sprintf(buffer,
		        "Accel: %.2f | g_mag: %.2f | Alt: %.2f m | dAlt/dt: %.2f m/s\r\n"
		        "State: %s | Tumble: %s | RapidAlt: %s | HeightDrop: %.2f m\r\n\r\n",
		        accel_mag,
		        g_mag,
		        altitude,
		        rate_of_change_alt,
		        get_fall_state_name(fall_state),
		        tumble_detected ? "YES" : "NO",
		        rapid_alt_event ? "YES" : "NO",
		        height_dropped_at_impact);

		    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
		}

		HAL_Delay(delay_ms); // 20 millisecond delay

		i++;

		// ********* Fall detection *********/
		// write your program from here:
		// ---- GYRO SQUARED MAGNITUDE ----
		float gx = gyro_velocity[0];
		float gy = gyro_velocity[1];
		float gz = gyro_velocity[2];

		g_mag = sqrtf((gx * gx) + (gy * gy) + (gz * gz));

		// 1. Calculate the overall magnitude of acceleration
		// Equation: Amag = sqrt(x^2 + y^2 + z^2)
		accel_mag = sqrtf((accel_filt_c[0] * accel_filt_c[0]) +
						  (accel_filt_c[1] * accel_filt_c[1]) +
						  (accel_filt_c[2] * accel_filt_c[2]));

		// 2. State Machine for Fall Detection
		switch (fall_state)
		{
		case STATE_IDLE:
			// Looking for free-fall (a drop in acceleration)
			tumble_detected = 0; // Reset for new detection
			rapid_alt_event = 0;   // reset
			height_dropped_at_impact = 0.0f; //reset
			start_altitude = altitude;
			blink_interval = SLOW_BLINK_MS;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // Ensure Buzzer is OFF
			if (accel_mag < FREE_FALL_THRESHOLD)
			{
				fall_state = STATE_FREE_FALL;
				start_altitude = altitude;
				height_dropped_at_impact = 0.0f;
				free_fall_start_time = HAL_GetTick();
			}
			break;
		case STATE_FREE_FALL:

			// Check for "Tumble" (Chaotic rotation)

			if (fabsf(rate_of_change_alt) > ALT_RATE_THRESHOLD)
			{
			    rapid_alt_event = 1;
			}
			if (accel_mag > ACCEL_IMPACT_THRESHOLD)
			{
				impact_time = HAL_GetTick();
				height_dropped_at_impact = 0.0f;
				fall_state = STATE_IMPACT_WAIT;
				break;
			}

			// timeout: if "impact" never comes, cancel
			if ((HAL_GetTick() - free_fall_start_time) > 2000) // 2s example
			{
				fall_state = STATE_IDLE;
			}
			break;

		case STATE_IMPACT_WAIT:
			blink_interval = FAST_BLINK_MS;
			if (g_mag > TUMBLE_THRESHOLD)
			{
				tumble_detected = 1;
			}
			if (fabsf(rate_of_change_alt) > ALT_RATE_THRESHOLD)
			{
			    rapid_alt_event = 1;
			}
		    current_time = HAL_GetTick();
		    uint32_t time_since_impact = current_time - impact_time;

		    // Height tracking (Keep your existing altitude logic here)
		    float current_drop = start_altitude - altitude;
		    if (current_drop > height_dropped_at_impact)
		    {
		        height_dropped_at_impact = current_drop;
		    }

		    // FINAL DECISION: Check after 5 seconds
		    if (time_since_impact >= INACTIVE_PERIOD)
		    {
		        // Combined conditions:
		        // 1. Must be still now (Gyro low)
		        // 2. Must have rotated during fall (Tumble)
		        // 3. Must have dropped far enough (Height)
		    	// 4. Must not be moving
		        if (g_mag < GYRO__INACTIVE_THRESHOLD && tumble_detected == 1 && height_dropped_at_impact >= FALL_HEIGHT_THRESHOLD &&
		        	    rapid_alt_event == 1 && accel_mag < MOVING_THRESHOLD)
		        {
		            fall_state = STATE_FALLEN_INACTIVE;
		        }
		        else
		        {
		            fall_state = STATE_IDLE; // False alarm or user recovered
		        }
		    }
		    break;
		case STATE_FALLEN_INACTIVE:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // Turn Buzzer ON
			ssd1306_Fill(Black);
			ssd1306_SetCursor(2, 10); // X: 2, Y: 10
			ssd1306_WriteString("Help me,", Font_11x18, White);
			ssd1306_SetCursor(2, 35); // Drop down to next line
			ssd1306_WriteString("I fell down!", Font_7x10, White);
			ssd1306_UpdateScreen();
			break;
		}

		// 2. Check if enough time has passed to toggle the LED
		current_time = HAL_GetTick();
		if ((current_time - last_led_toggle_time) >= blink_interval)
		{
			BSP_LED_Toggle(LED2);
			last_led_toggle_time = current_time; // Reset the timer
		}
	}
}

int mov_avg_C(int N, int *accel_buff)
{ // The implementation below is inefficient and meant only for verifying your results.
	int result = 0;
	for (int i = 0; i < N; i++)
	{
		result += accel_buff[i];
	}

	result = result / 4;

	return result;
}

static void UART1_Init(void)
{
	/* Pin configuration for UART. BSP_COM_Init() can do this automatically */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Configuring UART1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		while (1)
			;
	}
}

static void Buzzer_Init(void)
{
	// 1. Enable the clock for GPIO Port B
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// 2. Configure the specific pin (PB2)
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	// 3. Apply the initialization to Port B
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// 4. Ensure the buzzer starts turned OFF
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
}

const char *get_fall_state_name(FallState_t state)
{
	switch (state)
	{
	case STATE_IDLE:
		return "IDLE";
	case STATE_FREE_FALL:
		return "FREE_FALL";
	case STATE_IMPACT_WAIT:
		return "IMPACT_WAIT";
	case STATE_FALLEN_INACTIVE:
		return "FALLEN_INACTIVE";
	default:
		return "UNKNOWN";
	}
}

float altitude_rate(float altitude, uint32_t tick_ms)
{
    const uint32_t WINDOW_MS = 500;      // 10 Hz rate update
    static float alt_prev = 0.0f;
    static uint32_t t_prev = 0;
    static float rate_hold = 0.0f;

    if (t_prev == 0) {
        alt_prev = altitude;
        t_prev = tick_ms;
        return 0.0f;
    }

    uint32_t dt_ms = tick_ms - t_prev;

    // Not enough time -> keep previous rate
    if (dt_ms < WINDOW_MS) {
        return rate_hold;
    }

    float dt_s = dt_ms / 1000.0f;
    rate_hold = (altitude - alt_prev) / dt_s;

    alt_prev = altitude;
    t_prev = tick_ms;

    return rate_hold;
}

static void I2C1_Init(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();

	// 1. Configure PB8 (SCL) and PB9 (SDA) for I2C
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD; // Open Drain is required for I2C!
	GPIO_InitStruct.Pull = GPIO_PULLUP;		// I2C needs pull-up resistors
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1; // AF4 maps these pins to I2C1
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// 2. Configure the I2C Peripheral
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x10909CEC; // Standard 100kHz timing
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	HAL_I2C_Init(&hi2c1);
}

// Do not modify these lines of code. They are written to supress UART related warnings
int _read(int file, char *ptr, int len) { return 0; }
int _fstat(int file, struct stat *st) { return 0; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _isatty(int file) { return 1; }
int _close(int file) { return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { return -1; }
