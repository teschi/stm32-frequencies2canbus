/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/******************************************************************************
  *   This notice applies to any and all portions of this file that
  *   *are* between comment pairs USER CODE BEGIN and USER CODE END.
  *
  * COPYRIGHT(c) 2018 Marcus Prinz
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of the copyright holder nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************/


		#include <stdlib.h>
		#include <math.h>
		#include <stdio.h>
		#include <stdarg.h>
		#include <string.h>
		#include <stdbool.h>


		// Maximum Buffer size for writing to serial console
		#define PRINT_BUFFER_SIZE								300

		// Maximum Buffer size for reading from serial console
		#define RX_BUFFER_SIZE									100

		// Both ABP1 & ABP2 clocks run on 72Mhz so all timers have the same base clock
		#define TIMER_FREQ										72000000

		// Suitable range for capture value to check if before or after overflow (for more detail see where it is used)
		#define ACCEPTABLE_CAPTURE_RANGE_DURING_OVERFLOW		4096

		// This should allways be 4 (do not change)
		#define MAX_CHANNELS_PER_TIMER							4

		// We use EXTI10 as software interrupt for pending CAN transmissions (GPIO pin gets disabled immediately)
		#define SEND_CAN_SOFTWARE_INTERRUPT						UNUSED_EXTI10_Pin



		// Prescaler for TIM2&3 (referenced from CubeXM settings). In case of 100: (65536*100/72Mhz = 91ms overflow time)
		#define INPUT_CAPTURE_PRESCALER							100

		// Time after which we treat an input-signal as "invalid" when we didn't get a capture event (in seconds!)
		#define SIGNAL_INVALID_SECONDS							10

		// CAN-send interval in milliseconds
		#define DEFAULT_CAN_SEND_INTERVAL						20

		// Frequency multiplier for the CAN-Frame output.
		// (e.g. if multiplier is 100, a frequency of 25Hz, will be reflected in the CAN-frame as 2500 (or rather "0xC4 0x09")
		#define CAN_REPORT_FREQ_MULTIPLIER						100

		// Interval at which we report debug information (if debug is on)
		#define UART_DEBUG_REPORT_INTERVAL						1000

		// Common filter setting for all input-captures (see: IC1F in the datasheet)  (referenced from CubeXM settings)
		// if necessary this value could be increased using CKD in TIMx_CR1
		// prescalers (higher ICxF values and CKD) will cause slight jitter,
		// (although that might not have any effect on the reported freqs)
		// 5 == 0b0101 means 16 samples @ 72MHz (=0.22µs)
		#define INPUT_CAPTURE_FILTER_VALUE						5

		// Enable pin-out function for pin A0-4 to track time spent in main and interrupts.
		// Comment out to disable the set/reset functions completely (he pins are normally not activated anyway though)
		#define DEBUG_PINS_FOR_PERFORMANCE_MEASURE_ENABLED


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/



		// *************************** Type definition ***************************

		/**
		 * Input-Capture state for every channel.
		 * - keeps track of all status that the incoming timer events need to be processed
		 */
		struct InputCaptuteState_t
		{
			int isValid;						// don't clear on snapshot
			uint32_t lastRegisterValue;			// don't clear on snapshot
			int32_t overflowsSinceLastCapture;  // don't clear on snapshot
			int32_t lastCapture;
			int32_t sumCaptures;
			int32_t countCaptures;

			// data for statistics
			int32_t countCapturesSiceLastDebugReport;
			int32_t minCapture;
			int32_t maxCapture;
			int32_t maxOverflowsPerCapture;
			int32_t countOverCaptureErrors;
			int32_t countAmbigousOverflowErrors;
		};
		typedef struct InputCaptuteState_t InputCaptuteState;
		#define DEFAULT_CAPTURE_STATE 		{0,0,0,0,0,0,0,0,0,0,0,0}


		/**
		 * The Timer-Channel-State mainly consists of a reference to a Input-Capture state, where to get frequency information from,
		 * but also stores the last measured and the last reported frequency.
		 */
		struct TimerChannelState_t
		{
			InputCaptuteState *inputState;
			uint32_t lastMeasuredFreq;
			uint32_t lastReportedFreq;
		};
		typedef struct TimerChannelState_t TimerChannelState;


		/**
		 * The the Timer-State consists of all channels to be reported for that Timer,
		 * as well as the CAN-Frame where the reports will be written into.
		 */
		struct TimerState_t
		{
			int numChannels;
			bool scheduledForSending;
			CanTxMsgTypeDef *reportCanFrame;
			TimerChannelState channelStates[MAX_CHANNELS_PER_TIMER];
		};
		typedef struct TimerState_t TimerState;



		// *************************** Global Variables ***************************
		// (the volatile ones are used in main() as well as interrupts.)


		volatile uint32_t resetFlagsBackup = 0;
		volatile bool isSerialOutputEnabled = true;
		volatile uint32_t lastErrorEventTime = 0;
		volatile uint32_t currentInputCapturePrescaler = INPUT_CAPTURE_PRESCALER;
		volatile uint32_t overflowsUntilSignalInvalid = ((SIGNAL_INVALID_SECONDS * TIMER_FREQ)/(65536 * INPUT_CAPTURE_PRESCALER));
		volatile uint32_t canReportSendInterval = DEFAULT_CAN_SEND_INTERVAL;

		volatile uint32_t totalOverCaptureErros = 0;
		volatile uint32_t totalAmbiguousOverflowErros = 0;
		volatile uint32_t totalCanErrors = 0;
		volatile uint32_t totalHalErrors = 0;

		int usartRxCurrentIndex = 0;
		char usartRxInputBuffer[RX_BUFFER_SIZE];
		char usartRxBufferCopy[RX_BUFFER_SIZE];


		// CAN-Buffers - CAN_ID_STD = 11-bit-Addr / CAN_RTR_DATA =  normal data frame
		CanTxMsgTypeDef txMessageTim2 = {0x302, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0,0,0,0,0,0,0,0}};
		CanTxMsgTypeDef	txMessageTim3 = {0x303, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0,0,0,0,0,0,0,0}};


		// Raw list of Input-Capture-States by Timer and Channel. Used by the interrupt-handlers to write capture-events into
		InputCaptuteState captureStatesTim2[] = {DEFAULT_CAPTURE_STATE, DEFAULT_CAPTURE_STATE, DEFAULT_CAPTURE_STATE, DEFAULT_CAPTURE_STATE};
		InputCaptuteState captureStatesTim3[] = {DEFAULT_CAPTURE_STATE, DEFAULT_CAPTURE_STATE, DEFAULT_CAPTURE_STATE, DEFAULT_CAPTURE_STATE};


		// Storage for the reports and references to the raw structures to get the data from
		TimerState reportStateTim2 = {3, false, &txMessageTim2, {
						{&captureStatesTim2[0], 0, 0}, // CH1 / PA_0  / D2
						{&captureStatesTim2[2], 0, 0}, // CH3 / PA_2  / D1
						{&captureStatesTim2[3], 0, 0}  // CH4 / PA_3  / D0
				}
		};

		TimerState reportStateTim3 = {3, false, &txMessageTim3, {
						{&captureStatesTim3[0], 0, 0}, // CH1 / PC_6 / D35 (EXT-13)
						{&captureStatesTim3[1], 0, 0}, // CH2 / PC_7 / D36 (EXT-14)
						{&captureStatesTim3[2], 0, 0}  // CH3 / PC_8 / D37 (EXT-15)
				}
		};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_IWDG_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


		// *************************** Helper Macros ***************************

		#define min(X,Y) ((X) < (Y) ? (X) : (Y))
		#define max(X,Y) ((X) > (Y) ? (X) : (Y))
		// checks if certain bits are present in a value
		#define checkFlag(X, __FLAG__)          (((X) & (__FLAG__)) == (__FLAG__))
		// set the field of a register to a specific value (using _Pos and _Msk definitions in HAL)
		#define SetRegisterField(REG, FLD, VAL) ( (REG) = ( ((REG) & (~(FLD ## _Msk))) | (((VAL) << FLD ## _Pos) & FLD ## _Msk) ) )


		// *************************** GPIO Helper Macros ***************************

		// shorthand: pinSet(OUT_LED_GREEN) -> HAL_GPIO_WritePin(OUT_LED_GREEN_GPIO_Port, OUT_LED_GREEN_Pin, GPIO_PIN_SET);
		#define pinSet(X) ( HAL_GPIO_WritePin( X ## _GPIO_Port,  X ## _Pin, GPIO_PIN_SET))
		#define pinReset(X) ( HAL_GPIO_WritePin( X ## _GPIO_Port,  X ## _Pin, GPIO_PIN_RESET))
		#define pinToggle(X) ( HAL_GPIO_TogglePin( X ## _GPIO_Port, X ## _Pin))
		#define pinRead(X) ( HAL_GPIO_ReadPin( X ## _GPIO_Port, X ## _Pin))

		// checks whether the DebugEnable-Pin is pulled to GND
		#define isDebugEnablePinSet() ( !pinRead(IN_DEBUG_ENABLED))

		// those pins are used to measure the timing of interrupt handlers (to be used with a logic analyzer)
		#ifdef DEBUG_PINS_FOR_PERFORMANCE_MEASURE_ENABLED
			#define debugOutPinSet(X) 		pinSet(X)
			#define debugOutPinReset(X) 	pinReset(X)
			#define debugOutPinToggle(X) 	pinToggle(X)
		#else
			// replace debug-pin setting with empty statements
			#define debugOutPinSet(X) 		{}
			#define debugOutPinReset(X) 	{}
			#define debugOutPinToggle(X) 	{}
		#endif


		// *************************** LED helper functions ***************************


		/** Flash the Green LED to indicate an error.
		 * - flashes with a maximum frequency, so it is actually visible
		 * - Led will be cleared in main() function if there was no error for some time
		 */
		void flashErrorLed()
		{
			static volatile uint32_t lastErrorBlinkTime = 0;

			uint32_t tick = HAL_GetTick();
			lastErrorEventTime = tick;
			// lastErrorBlinkTime should always be smaller than lastErrorEventTime, otherwise we had an overflow
			lastErrorBlinkTime = min(lastErrorBlinkTime, lastErrorEventTime);

			// don't flash so quick that we don't see it
			if (HAL_GetTick() > lastErrorBlinkTime + 30)
			{
				pinToggle(OUT_LED_GREEN);
				lastErrorBlinkTime = tick;
			}
		}

		/**
		 * toggle the Yellow LED to indicate an frequency measurement.
		 *  - frequency will be limited to get rid of weird moiré effects at high frequencies
		 */
		void toggleFrequencyLed()
		{
			static uint32_t lastYellowLedToggleTime = 0;

			// limit yellow led frequency to 25Hz, to reduce weird interference effects at high frequencies
			uint32_t tick = HAL_GetTick();
			if (tick - lastYellowLedToggleTime > 20 || tick < lastYellowLedToggleTime)
			{
				pinToggle(OUT_LED_YELLOW);
				lastYellowLedToggleTime = tick;
			}
		}


		// *************************** USART Transmit functions ***************************

		/**
		 * wrapper that writes a plain string to USART.
		 */
		void writeString(const char *str, int len)
		{
			// in theory we could also redirect to an USB CDC device,
			// but we CANNOT because on an STM32F103 CAN and USB MUST NOT be used at the same time
			HAL_UART_Transmit(&huart1, (uint8_t *)str, len, HAL_MAX_DELAY); // send message via UART
		}

		/**
		 * Custom printf helper that writes to USART a plain string.
		 *
		 * @param fmt String to print
		 * @param argp Parameters list
		 */
		void HAL_printf_valist(const char *fmt, va_list argp)
		{
			char string[PRINT_BUFFER_SIZE];
			if (vsnprintf(string, PRINT_BUFFER_SIZE, fmt, argp) > 0)
			{
				writeString(string, strlen(string));
			}
			else
			{
				writeString("Printf-Error!\r\n", 15);
			}
		}

		/**
		 * Custom printf function, that translates ... to va_list.
		 *
		 * inspired by: https://notes.iopush.net/stm32-log-and-printf/.
		 * @param *fmt String to print
		 * @param ... Data
		 */
		void HAL_printf(const char *fmt, ...) {
			if (isSerialOutputEnabled)
			{
				va_list argp;
				va_start(argp, fmt);
				HAL_printf_valist(fmt, argp);
				va_end(argp);
			}
		}


		// *************************** UART receive and debug-console functions  ***************************


		/**
		 * Calculates prescaler- & period-values to match a given frequency.
		 *
		 * This is rather slow because of all the float calculations. It should only be used for debugging
		 * We're counting down the prescaler and count up the interval and remember the closest match we got.
		 * @param targetFreq[in] the frequency that should me matched
		 * @param foundFreq[out] the frequency we calculated
		 * @param foundDelta[out] the difference between the requested and the calculated frequency
		 * @param foundPrescaler[out], foundPeriod[out] the register-values that produce the frequency
		 */
		void calculateFrequency(float targetFreq, float *foundFreq, float *foundDelta, uint16_t *foundPrescaler, uint16_t *foundPeriod)
		{
			uint32_t iCurrPresc = 0xFFFF, iBestPresc = 0xFFFF;  // wer're starting with the maximum prescaler value
			uint32_t iCurrPeriod, iBestPeriod;
			float fCurrDelta = 999999, fBestDelta = 999999;
			float fCurrFreq;

			iCurrPeriod = (TIMER_FREQ / (iCurrPresc * targetFreq));
			iCurrPeriod = max( 4, (iCurrPeriod/2)*2 );		// make sure we begin with an even number which is at least 4
			iBestPeriod = iCurrPeriod;

			while (iCurrPresc >= 1 && iCurrPeriod <= 0xFFFF)
			{
				// calculate Frequency and Delta
				fCurrFreq = (float)TIMER_FREQ / (iCurrPresc * iCurrPeriod);
				fCurrDelta = fabsf(targetFreq - fCurrFreq);

				// remember the best match
				if (fCurrDelta < fBestDelta)
				{
					// HAL_printf("better match: Delta=%f, freq=%f, presc=%d, period=%d\r\n", fCurrDelta, fCurrFreq, iCurrPresc, iCurrPeriod);
					fBestDelta = fCurrDelta;
					iBestPresc = iCurrPresc;
					iBestPeriod = iCurrPeriod;
				}

				// check whether match is good enough
				if (fCurrDelta == 0)
				{
					// HAL_printf("EXACT match: freq=%f, presc=%d, period=%d\r\n", fCurrFreq, iCurrPresc, iCurrPeriod);
					break;
				}

				if (fCurrFreq < targetFreq)
				{
					iCurrPresc--;
				}
				else
				{
					iCurrPeriod += 2;
				}
			}

			// calculate the final frequency again and return
			fCurrFreq = (float)TIMER_FREQ / (iBestPresc * iBestPeriod);
			// HAL_printf("Final (best) match: delta=%f, freq=%f, presc=%d, period=%d\r\n", fBestDelta, fCurrFreq, iBestPresc, iBestPeriod);

			(*foundFreq) = fCurrFreq;
			(*foundDelta) = fBestDelta;
			(*foundPrescaler) = iBestPresc;
			(*foundPeriod) = iBestPeriod;
		}

		/**
		 * Set a PWM interval for a TIM channel, to produces a square-wave.
		 *
		 * Even though we specify the channel to set the "half-point", this changes "autoreload" for the complete timer
		 */
		void setPwmInterval(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t interval)
		{
			// set the middle point that we change the level
			switch (channel)
			{
				case TIM_CHANNEL_1:
					htim->Instance->CCR1 = interval/2;
					break;
				case TIM_CHANNEL_2:
					htim->Instance->CCR2 = interval/2;
					break;
				case TIM_CHANNEL_3:
					htim->Instance->CCR3 = interval/2;
					break;
				case TIM_CHANNEL_4:
					htim->Instance->CCR4 = interval/2;
					break;
				default:
					{
						HAL_printf("Illegal Channel in adjustPwmInterval!\r\n");
						return;
					}
			}

			// set the full interval at which we reset
			__HAL_TIM_SET_AUTORELOAD(htim, interval-1);

			// if we reduce the interval, we might already be past it, in that that case:  reset it immediately
			// otherwise we could wait a long time here.
			if (__HAL_TIM_GET_COUNTER(htim) >= interval-1)
			{
				__HAL_TIM_SET_COUNTER(htim, 0);
			}
		}

		/**
		 * Sets the prescaler of a timer.
		 *
		 * And make sure it gets used immediately.
		 * @param value[in] the actual length without the offset of the PSC register
		 */
		void setTimerPrescaler(TIM_HandleTypeDef *htim, uint32_t value)
		{
			// prescaler value is offset by 1
			__HAL_TIM_SET_PRESCALER(htim, value-1);

			// manually trigger update event, so the changed prescaler-value immediately gets loaded
			HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE);
		}

		/**
		 * Change the frequency of one of the debug-PWM signals.
		 */
		void adjustDebugPwmFrequency(TIM_HandleTypeDef *htim, uint32_t channel, float targetFreq)
		{
			float foundFreq, foundDelta;
			uint16_t foundPrescaler, foundPeriod;

			calculateFrequency(targetFreq, &foundFreq, &foundDelta, &foundPrescaler, &foundPeriod);
			HAL_printf("result: freq=%f, delta=%f%%, prescaler=%d, period=%d\r\n", foundFreq, 100*foundDelta/foundFreq, foundPrescaler, foundPeriod);

			// set frequency
			setPwmInterval(htim, channel, foundPeriod);
			setTimerPrescaler(htim, foundPrescaler);
		}

		/**
		 * De-initialize all output-signals used for debugging, and make them floating.
		 *
		 * Primarily to prevent any accidental connections in the live environment.
		 * They might still be set in the code, but that wouldn't do anything anymore
		 */
		void deinitAllDebugOutputPins()
		{
			// deinitialize all Debug GPIO outputs (makes them floating)
			HAL_GPIO_DeInit(OUT_DEBUG_SIGNAL0_GPIO_Port, OUT_DEBUG_SIGNAL0_Pin);
			HAL_GPIO_DeInit(OUT_DEBUG_SIGNAL1_GPIO_Port, OUT_DEBUG_SIGNAL1_Pin);
			HAL_GPIO_DeInit(OUT_DEBUG_SIGNAL2_GPIO_Port, OUT_DEBUG_SIGNAL2_Pin);
			HAL_GPIO_DeInit(OUT_DEBUG_SIGNAL3_GPIO_Port, OUT_DEBUG_SIGNAL3_Pin);
			HAL_GPIO_DeInit(OUT_DEBUG_SIGNAL4_GPIO_Port, OUT_DEBUG_SIGNAL4_Pin);

			// stop frequency output on D6 (make pin floating)
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_Base_DeInit(&htim1);
			HAL_GPIO_DeInit(PWM_DEBUG_TIM1_CH1_GPIO_Port, PWM_DEBUG_TIM1_CH1_Pin);

			// stop frequency output on D5 (make pin floating)
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
			HAL_TIM_Base_DeInit(&htim4);
			HAL_GPIO_DeInit(PWM_DEBUG_TIM4_CH1_GPIO_Port, PWM_DEBUG_TIM4_CH1_Pin);
		}

		/**
		 * Re-enables the debugging pins on-demand (in case the debug-jumper was not set).
		 */
		void reenableAllDebugOutputPins()
		{
			// re-initialize all GPIOs
			MX_GPIO_Init();

			// re-initialize both PWM timers
			MX_TIM1_Init();
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

			MX_TIM4_Init();
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		}

		/**
		 * Handles an incoming debug-command-string coming in on the USART.
		 *
		 * First char is the command, following chars might be used as parameter.
		 */
		void handleDebugCommand(char* message)
		{
			HAL_printf("Debug-Command: %s\r\n", message);

			char cmd = message[0];
			uint32_t intValue;
			float floatValue;

			switch (cmd)
			{
				// help
				case 'h':
					HAL_printf("Help_Usage\r\n");
					HAL_printf("  p<uint>  - change prescaler for InputCapture (default: 100)\r\n");
					HAL_printf("  F<float> - change frequency on test-signal from D5\r\n");
					HAL_printf("  f<float> - change frequency on test-signal from D6\r\n");
					HAL_printf("  i<uint>  - change input-filter value [IC1F] (-1) for input-captures (allowed: 1-16, default: 6) \r\n");
					HAL_printf("  c<uint>  - change input-filter Clock-division [CKD] (-1) for input-captures (allowed: 1-3, default: 1) \r\n");
					HAL_printf("  d        - disable debug GPIO-PINS (D5, D6 and A0-4)\r\n");
					HAL_printf("  e        - enable debug GPIO-PINS (D5, D6 and A0-4)\r\n");
					HAL_printf("  u        - toggle USART debugging on/off\r\n");
					HAL_printf("  w        - trigger a Watchdog reset\r\n");
					HAL_printf("  s        - execute a Sofware reset\r\n");
					HAL_printf("  r<uint>  - CAN report rate (default: 20)\r\n");
					HAL_printf("  n        - Print information\r\n");
					HAL_printf("  l        - Print license info\r\n");
					break;

				// set prescaler for Input-Capture Timers
				case 'p':
					intValue = strtol(message+1, NULL, 10);
					if (intValue == 0 || intValue > 0x10000)
						HAL_printf("cannot convert to integer: %s\r\n", message+1);
					else
					{
						HAL_printf("Setting TIM2&3 prescaler to %d (0x%04X)\r\n", intValue, intValue);
						currentInputCapturePrescaler = intValue;
						overflowsUntilSignalInvalid = ((SIGNAL_INVALID_SECONDS * TIMER_FREQ)/(65536 * currentInputCapturePrescaler));
						setTimerPrescaler(&htim2, currentInputCapturePrescaler);
						setTimerPrescaler(&htim3, currentInputCapturePrescaler);
						__HAL_TIM_SET_COUNTER(&htim3, 0x7fff); // offset the 2 counters, so they don't overflow at the same time
					}
					break;

				// change Debug-PWM frequency on Pin D5
				case 'F':
					floatValue = strtof(message+1, NULL);
					if (floatValue == 0)
						HAL_printf("cannot convert to double: %s\r\n", message+1);
					else
					{
						// for float printing to work i had to add the linker-flag "-u _printf_float" (http://www.openstm32.org/forumthread2108)
						HAL_printf("Calculating Frequency for %f ...\r\n", floatValue);
						adjustDebugPwmFrequency(&htim4, TIM_CHANNEL_1, floatValue);
					}
					break;

				// change Debug-PWM frequency on Pin D6
				case 'f':
					floatValue = strtof(message+1, NULL);
					if (floatValue == 0)
						HAL_printf("cannot convert to double: %s\r\n", message+1);
					else
					{
						// for float printing to work i had to add the linker-flag "-u _printf_float" (http://www.openstm32.org/forumthread2108)
						HAL_printf("Calculating Frequency for %f ...\r\n", floatValue);
						adjustDebugPwmFrequency(&htim1, TIM_CHANNEL_1, floatValue);
					}
					break;

				// change capture-filter value for all channels
				case 'i':
					intValue = strtol(message+1, NULL, 10);
					if (intValue < 1 || intValue > 16)
						HAL_printf("cannot convert to integer: %s\r\n", message+1);
					else
					{
						intValue--;
						HAL_printf("setting capture-filters to %d\r\n", intValue);

						SetRegisterField(htim2.Instance->CCMR1, TIM_CCMR1_IC1F, intValue);
						SetRegisterField(htim2.Instance->CCMR1, TIM_CCMR1_IC2F, intValue);
						SetRegisterField(htim2.Instance->CCMR2, TIM_CCMR2_IC3F, intValue);
						SetRegisterField(htim2.Instance->CCMR2, TIM_CCMR2_IC4F, intValue);
						SetRegisterField(htim3.Instance->CCMR1, TIM_CCMR1_IC1F, intValue);
						SetRegisterField(htim3.Instance->CCMR1, TIM_CCMR1_IC2F, intValue);
						SetRegisterField(htim3.Instance->CCMR2, TIM_CCMR2_IC3F, intValue);
						SetRegisterField(htim3.Instance->CCMR2, TIM_CCMR2_IC4F, intValue);
					}
					break;

				// change input-filter clock-division for the timers used for input-capture
				case 'c':
					intValue = strtol(message+1, NULL, 10);
					if (intValue < 1 || intValue > 3)
						HAL_printf("cannot convert to integer: %s\r\n", message+1);
					else
					{
						intValue--;
						HAL_printf("setting CKD to %d\r\n", intValue);

						SetRegisterField(htim2.Instance->CR1, TIM_CR1_CKD, intValue);
						SetRegisterField(htim3.Instance->CR1, TIM_CR1_CKD, intValue);
					}
					break;

				// disable all debug-out pins
				case 'd':
					HAL_printf("de-initialize debug outputs\r\n");
					deinitAllDebugOutputPins();
					break;

				// re-enable all debug-out pins
				case 'e':
					HAL_printf("re-enable debug outputs\r\n");
					reenableAllDebugOutputPins();
					break;

				// toggle USART-output on/off
				case 'u':
					isSerialOutputEnabled = !isSerialOutputEnabled;
					HAL_printf("set enabled Serial output = %d\r\n", isSerialOutputEnabled);
					break;

				// trigger a watchdog reset (the bootloader should not wait for new firmware but boot immediately)
				case 'w':
					HAL_printf("Triggering Watchdog reset\r\n");
					HAL_Delay(5000);
					HAL_printf("ERROR - should have reset by now\r\n");
					break;

				// trigger a software reset (the bootloader should do it's normal thing and wait a bit for new firmware)
				case 's':
					HAL_printf("Doing a Software reset\r\n");
					NVIC_SystemReset();
					break;

				// change the the interval of when can-frames will be sent
				case 'r':
					intValue = strtol(message+1, NULL, 10);
					if (intValue < 1)
						HAL_printf("cannot convert to integer: %s\r\n", message+1);
					else
					{
						HAL_printf("setting CAN Report interval to %d\r\n", intValue);
						canReportSendInterval = intValue;
					}
					break;

				// report system-information
				case 'n':
					HAL_printf("Displaying info\r\n");
					HAL_printf(" - Uptime: %d seconds\r\n", HAL_GetTick()/1000);
					HAL_printf(" - Reset-Flags: ");
					if (checkFlag(resetFlagsBackup, RCC_CSR_PINRSTF))
						HAL_printf("Reset-PIN (PIN), ");
					if (checkFlag(resetFlagsBackup, RCC_CSR_PORRSTF))
						HAL_printf("Power-ON (POR), ");
					if (checkFlag(resetFlagsBackup, RCC_CSR_SFTRSTF))
						HAL_printf("Software (SFT), ");
					if (checkFlag(resetFlagsBackup, RCC_CSR_IWDGRSTF))
						HAL_printf("Independent watchdog (IWDG), ");
					if (checkFlag(resetFlagsBackup, RCC_CSR_WWDGRSTF))
						HAL_printf("Window watchdog (WWDG), ");
					if (checkFlag(resetFlagsBackup, RCC_CSR_LPWRRSTF))
						HAL_printf("Low-power (LPWR), ");
					HAL_printf("\r\n");
					HAL_printf(" - OverCapture errors: %d\r\n", totalOverCaptureErros);
					HAL_printf(" - AmbiguousOverflow errors: %d\r\n", totalAmbiguousOverflowErros);
					HAL_printf(" - CAN errors: %d\r\n", totalCanErrors);
					HAL_printf(" - HAL errors: %d\r\n", totalHalErrors);
					break;

				// print license text
				case 'l':
					HAL_printf("\r\n---License info---\r\nfor more information see: https://github.com/teschi/stm32-frequencies2canbus\r\n");
					HAL_printf("* HAL-Framework\r\n    COPYRIGHT(c) 2018 STMicroelectronics\r\n    License: BSD-3-Clause\r\n");
					HAL_printf("* USER CODE\r\n    COPYRIGHT(c) 2018 Marcus Prinz\r\n    License: BSD-3-Clause\r\n\r\n");
					break;

				default:
					HAL_printf("Unknown Command\r\n");
					break;

			}
			HAL_printf("-----\r\n");
		}

		/**
		 * Interrupt-handler for incoming characters on USART.
		 *
		 * We add the char to a buffer-string and handle the string when we see NewLine.
		 * (we use this instead of "HAL_UART_IRQHandler", because the default way of using TxCpltCallback is unusable for our case,
		 *  because we would have to know the string-length in advance)
		 */
		void MY_UART_IRQHandler(UART_HandleTypeDef *huart)
		{
			// check if there is a new byte that was received (unsure if there can be multiple, so we use a loop)
			while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE))
			{
				// get the byte out of the receive buffer (will automatically clear the interrupt)
				char ch = __HAL_UART_FLUSH_DRREGISTER(huart);

				// if we have space in the buffer add the character
				if (ch != '\r' && ch != '\n' && usartRxCurrentIndex < RX_BUFFER_SIZE-2)
					usartRxInputBuffer[usartRxCurrentIndex++] = ch;

				// if it was a newline, copy the string, reset the buffer and handle the copied data
				if (ch == '\n')
				{
					usartRxInputBuffer[usartRxCurrentIndex++] = '\0';
					strncpy(usartRxBufferCopy, usartRxInputBuffer, RX_BUFFER_SIZE);
					usartRxCurrentIndex = 0;

					handleDebugCommand(usartRxBufferCopy);
				}
			}

			// not sure if we really need to do this. Datasheet mentions that ORE will also trigger this Interrupt
			// it might be auto-cleared by the lines above, but to be on the safe side clear it
			if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
				__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
		}

		/**
		 * Used own IRQ handler for USART (code-generation for it turned off in CubeMX (in NVIC-config)).
		 */
		void USART1_IRQHandler(void)
		{
			debugOutPinSet(OUT_DEBUG_SIGNAL4);
			MY_UART_IRQHandler(&huart1);
			debugOutPinReset(OUT_DEBUG_SIGNAL4);
		}


		// *************************** TIMER functions & interrupt handlers ***************************


		/**
		 * Handle the Input-Capture-Event that occurred on a channel.
		 *
		 * This is the heart of the measurement. In here we:
		 * - keep track of the register-values to measure the intervals
		 * - keep track of overflow events (even multiple ones) to measure long intervals
		 * - keep track of overcapture-events (in case we missed an event), and discard measurements accordingly
		 * - maintain a "valid" state of the channel (we need at least 2 correct capture-events to get a frequency)
		 * - correct ambiguous capture events (when overflow and Capture happens in the same event, and we don't know what was first)
		 * - keep some statistics
		 */
		void handleTimerInputCaptureEvent(TIM_HandleTypeDef *htim, InputCaptuteState *state, uint32_t channel, uint32_t interruptFlag, uint32_t overCaptureFlag, uint32_t eventIncludesOverflow)
		{
			// get the InputCapture value
			uint32_t ic = __HAL_TIM_GET_COMPARE(htim, channel);

			// check whether we had an OverCapture condition (2nd capture before we cleared the 1st)
			uint32_t oc = __HAL_TIM_GET_FLAG(htim, overCaptureFlag);
			if (oc)
			{
				// clear the OC flag and remember it for logging
				__HAL_TIM_CLEAR_FLAG(htim, overCaptureFlag);
				state->countOverCaptureErrors++;
				totalOverCaptureErros++;
			}

			// we can now clear the timer InputCapture interrupt (after we handled OC)
			__HAL_TIM_CLEAR_IT(htim, interruptFlag);

			// calculate the capture-value (which might span over multiple Overflows)
			state->lastCapture = (int)ic - state->lastRegisterValue + state->overflowsSinceLastCapture*0x10000;
			// remember the current capture-value as baseline for the next capture
			state->lastRegisterValue = ic;

			// do some statistics and clear the overflow-counter
			state->maxOverflowsPerCapture = max(state->maxOverflowsPerCapture, state->overflowsSinceLastCapture);
			state->overflowsSinceLastCapture = 0;

			// if we had an overflow in the same interrupt as the capture, we don't know what came first from the event itself.
			// So we check if the captured value is a very low value (which implies that it happened AFTER the overflow)
			// If this is the case, we add the additional overflow value, and remove it from the count for the next run
			if (eventIncludesOverflow)
			{
				if (ic > (0x10000 - ACCEPTABLE_CAPTURE_RANGE_DURING_OVERFLOW))
				{
					// capture-event was right before the overflow --> Do nothing
				}
				else if (ic < ACCEPTABLE_CAPTURE_RANGE_DURING_OVERFLOW)
				{
					// capture-event was right after the overflow --> fix overflow
					state->lastCapture += 0x10000;
					state->overflowsSinceLastCapture--;
				}
				else
				{
					// if the capture-value is neither very small nor very big, we can't make the choice.
					// We treat this as ERROR, as the event handle took WAAAAY to long to react.
					// At this point we don't trust "overflowsSinceLastCapture" anymore and we're invalidating the signal
					// which means we get a signal back after 2 capture-events
					totalAmbiguousOverflowErros++;
					state->countAmbigousOverflowErrors++;
					state->isValid = false;
					return;
				}
			}

			// only count a measurement value if we didn't run into overcapture, and if the signal is valid
			if (!oc && state->isValid)
			{
				state->countCaptures++;
				state->sumCaptures += state->lastCapture;

				state->maxCapture = (state->countCapturesSiceLastDebugReport == 0) ? state->lastCapture : max(state->maxCapture, state->lastCapture);
				state->minCapture = (state->countCapturesSiceLastDebugReport == 0) ? state->lastCapture : min(state->minCapture, state->lastCapture);
				state->countCapturesSiceLastDebugReport++;
			}

			state->isValid = true;
		}

		/**
		 * Handle the overflow-event of a timer, and increase the overflow-counter of all its capture-states.
		 * We also set a channel to "invalid" if we didn't get an event for a defined amount of time
		 */
		void handleTimerOverflowEvent(TIM_HandleTypeDef *htim, TimerState *report_states)
		{
			// clear Interrupt
			__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);

			// send all channels that are reported an overflow-event
			for (int i=0; i<report_states->numChannels; i++)
			{
				InputCaptuteState *state = report_states->channelStates[i].inputState;
				if (state->isValid)
				{
					state->overflowsSinceLastCapture++;

					// invalidate because we didn't get a capture-event for a long time (hence overflow counter got big)
					if (state->overflowsSinceLastCapture > overflowsUntilSignalInvalid)
					{
						state->isValid = false;
					}
				}
			}
		}

		/**
		 * Copy the current channel status (to be reported via CAN) and clear the measuerment state for next measurement.
		 *
		 * We leave some state-information intact
		 *  - stuff we need for the next measurement (last register-value, overflow-count, invalid-state)
		 *  - debug statistics
		 */
		void takeTimerChannelSnapshot(InputCaptuteState *target, InputCaptuteState *state)
		{
			*target = *state;
			state->lastCapture = 0;
			state->sumCaptures = 0;
			state->countCaptures = 0;
		}

		/**
		 * clears the debug statistics (e.g. for after we reported on USART)
		 */
		void clearTimerChannelDebugStats(InputCaptuteState *state)
		{
			state->countCapturesSiceLastDebugReport = 0;
			state->maxCapture = 0;
			state->minCapture = 0;
			state->countOverCaptureErrors = 0;
			state->countAmbigousOverflowErrors = 0;
			state->maxOverflowsPerCapture = 0;
		}

		/**
		 * Calculate the Frequencies to report on CAN and update the CAN-Frame buffer.
		 * Note: this is called from a different interrupt (lower-prio interrupt than the timers)
		 *
		 * This function:
		 *  - takes a snapshot of all channels of a timer (in an atomic manner)
		 *  - calculates the frequency (either the measured one or a fallback value based the amount of overflows)
		 *  - updates the CAN-Frame buffer
		 */
		void calculateCanFrequencyReport(TimerState *report)
		{
			static InputCaptuteState snapshots[4] = {DEFAULT_CAPTURE_STATE, DEFAULT_CAPTURE_STATE, DEFAULT_CAPTURE_STATE, DEFAULT_CAPTURE_STATE};

			// take snapshots of all states for this report (atomic to make sure interrupts don't get in between the copying)
			__disable_irq();
			for (int i=0; i<report->numChannels; i++)
			{
				takeTimerChannelSnapshot( &snapshots[i], report->channelStates[i].inputState);
			}
			__enable_irq();

			// calculate the Frequency we're reporting.
			//  - if we got a capture last cycle  -->  take it
			//  - if we don't have a capture, and the pulse we're currently measuring is still shorter than the last one  -->  take the previous measurement
			//  - if we don't have a capture, and the pulse we're currently measuring is already longer --> use the interim frequency (gradually getting lower frequency)
			//  - if we don't have a capture, and the signal got Invalid (return 0)
			for (int i=0; i<report->numChannels; i++)
			{
				TimerChannelState *rep = &(report->channelStates[i]);
				InputCaptuteState *ic = &snapshots[i];

				if (ic->countCaptures > 0)
				{
					// if we got multiple captures we're using the average.
					rep->lastMeasuredFreq = ((uint64_t)TIMER_FREQ * ic->countCaptures * CAN_REPORT_FREQ_MULTIPLIER) / ((uint64_t)ic->sumCaptures * currentInputCapturePrescaler);
					rep->lastReportedFreq = rep->lastMeasuredFreq;
				}
				else if (ic->isValid)
				{
					int32_t temp_capture = ic->overflowsSinceLastCapture*0x10000 - (int32_t)ic->lastRegisterValue;
					if (temp_capture > 0)
					{
						rep->lastReportedFreq = ((uint64_t)TIMER_FREQ * CAN_REPORT_FREQ_MULTIPLIER) / ((uint64_t)temp_capture * currentInputCapturePrescaler);
						rep->lastReportedFreq = min(rep->lastReportedFreq, rep->lastMeasuredFreq);
					}
				}
				else
				{
					rep->lastMeasuredFreq = 0;
					rep->lastReportedFreq = 0;
				}
			}

			// update Can-Frame data
			CanTxMsgTypeDef * frm = report->reportCanFrame;
			for (int i=0; i<report->numChannels; i++)
			{
				TimerChannelState *rep = &(report->channelStates[i]);
				uint16_t freq = min(rep->lastReportedFreq, 0xFFF0);  // leave room for a few reserved states
				frm->Data[i*2 + 0] = freq & 0xff;
				frm->Data[i*2 + 1] = freq >> 8;
			}
		}

		/**
		 * Collect debug-information / statistics on all channels for one Timer, and send it out on the USART.
		 * Note: this is called from the main loop
		 */
		void createUsartDebugReport(TimerState *reportState)
		{
			// create buffers to (shallow-)copy all states
			TimerChannelState channelStateCopy[MAX_CHANNELS_PER_TIMER];
			InputCaptuteState captureStateCopy[MAX_CHANNELS_PER_TIMER];

			// take an atomic snapshot of the structures and clear the statistics.
			__disable_irq();
			for (int i=0; i<reportState->numChannels; i++)
			{
				channelStateCopy[i] = reportState->channelStates[i];
				captureStateCopy[i] = *(reportState->channelStates[i].inputState);
				clearTimerChannelDebugStats(reportState->channelStates[i].inputState);
			}
			__enable_irq();


			for (int i=0; i<reportState->numChannels; i++)
			{
				TimerChannelState *currChannelState = &(channelStateCopy[i]);
				InputCaptuteState *currCaptureState = &(captureStateCopy[i]);
				if (currChannelState->inputState->isValid)
				{
					uint32_t maxFreq = ((uint64_t)TIMER_FREQ * CAN_REPORT_FREQ_MULTIPLIER) / ((uint64_t)currCaptureState->minCapture * currentInputCapturePrescaler);
					uint32_t minFreq = ((uint64_t)TIMER_FREQ * CAN_REPORT_FREQ_MULTIPLIER) / ((uint64_t)currCaptureState->maxCapture * currentInputCapturePrescaler);

					HAL_printf("%03X-%d) #Cap: %4d, #OvCap: %d, #OvlErr: %d, #OvFlow/cap: %2d, minPrd: %7d (maxFreq: %6d), maxPrd: %7d (minFreq: %6d) --- lastFreqMeasure: %6d, lastFreqReport: %6d\r\n",
							reportState->reportCanFrame->StdId,
							i,
							currCaptureState->countCapturesSiceLastDebugReport,
							currCaptureState->countOverCaptureErrors,
							currCaptureState->countAmbigousOverflowErrors,
							currCaptureState->maxOverflowsPerCapture,
							currCaptureState->minCapture,
							maxFreq,
							currCaptureState->maxCapture,
							minFreq,
							currChannelState->lastMeasuredFreq,
							currChannelState->lastReportedFreq);
				}
			}
		}

		/**
		 * Interrupt-handler for timer-events.
		 * this has a similar to the default HAL_TIM_IRQHandler EXCEPT:
		 *  - we only check TIMx_SR once, to make sure we only handle interrupts that happened BEFORE the event
		 *  - pass information to the event-handler that an overflow happened in parallel, so it can be handled accordingly
		 *  - we don't auto clear the interrupts so we can check for OverCapture before clearing the event.
		 *    (see also: https://community.st.com/thread/43132-checking-for-timer-overcapture-flag-inside-haltimiccapturecallback-is-erroneous)
		 */
		void MY_TIM_IRQHandler(TIM_HandleTypeDef *htim, InputCaptuteState channel_states[], TimerState *report_states)
		{

			// atomic operation to get all interrupts that are due
			uint32_t SR_snapshot = htim->Instance->SR;

			// in case we have an overflow of the timer the same time as capture, we might have to compensate
			uint32_t eventIncludesOverflow = checkFlag(SR_snapshot, TIM_FLAG_UPDATE);

			// toggle YELLOW led so we "see" frequencies (but only on actual capture-events, not on overflows)
			if (SR_snapshot & (TIM_FLAG_CC1|TIM_FLAG_CC2|TIM_FLAG_CC3|TIM_FLAG_CC4))
			{
				toggleFrequencyLed();
			}

			// call handler with channel specific arguments depending on which event was set
			if (checkFlag(SR_snapshot, TIM_FLAG_CC1))
			{
				handleTimerInputCaptureEvent(htim, &channel_states[0], TIM_CHANNEL_1, TIM_FLAG_CC1, TIM_FLAG_CC1OF, eventIncludesOverflow);
			}
			if (checkFlag(SR_snapshot, TIM_FLAG_CC2))
			{
				handleTimerInputCaptureEvent(htim, &channel_states[1], TIM_CHANNEL_2, TIM_FLAG_CC2, TIM_FLAG_CC2OF, eventIncludesOverflow);
			}
			if (checkFlag(SR_snapshot, TIM_FLAG_CC3))
			{
				handleTimerInputCaptureEvent(htim, &channel_states[2], TIM_CHANNEL_3, TIM_FLAG_CC3, TIM_FLAG_CC3OF, eventIncludesOverflow);
			}
			if (checkFlag(SR_snapshot, TIM_FLAG_CC4))
			{
				handleTimerInputCaptureEvent(htim, &channel_states[3], TIM_CHANNEL_4, TIM_FLAG_CC4, TIM_FLAG_CC4OF, eventIncludesOverflow);
			}
			// overflow must be executed last (so the ambiguity compensation in handleTimerInputCaptureEvent works)
			if (eventIncludesOverflow)
			{
				handleTimerOverflowEvent(htim, report_states);
			}
		}

		/**
		 * Used own IRQ handler for TIM2 rather than the default HAL one, see MY_TIM_IRQHandler.
		 * (code-generation for it turned off in CubeMX (in NVIC config))
		 */
		void TIM2_IRQHandler(void)
		{
			debugOutPinSet(OUT_DEBUG_SIGNAL1);
			MY_TIM_IRQHandler(&htim2, captureStatesTim2, &reportStateTim2);
			debugOutPinReset(OUT_DEBUG_SIGNAL1);
		}

		/**
		 * Used own IRQ handler for TIM3 rather than the default HAL one, see MY_TIM_IRQHandler.
		 * (code-generation for it turned off in CubeMX (in NVIC-config))
		 */
		void TIM3_IRQHandler(void)
		{
			debugOutPinSet(OUT_DEBUG_SIGNAL1);
			MY_TIM_IRQHandler(&htim3, captureStatesTim3, &reportStateTim3);
			debugOutPinReset(OUT_DEBUG_SIGNAL1);
		}


		// *************************** CAN functions & interrupt handlers ***************************


		/**
		 * calculates the frequencies and sends the CAN frame (asynchronously)
		 */
		void sendCanFrame(TimerState *timReport)
		{
			calculateCanFrequencyReport(timReport);
			hcan.pTxMsg = timReport->reportCanFrame;

			// the CAN_transmit_IT takes call about 12µs (while the CAN-Frame transmission takes about 450µs)
			if (HAL_CAN_Transmit_IT(&hcan) != HAL_OK)
			{
				totalCanErrors++;
				flashErrorLed();
			}
		}

		/**
		 * unused callback that the can-frame has been sent
		 */
		void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
		{
			debugOutPinSet(OUT_DEBUG_SIGNAL2);
			debugOutPinReset(OUT_DEBUG_SIGNAL2);
		}

		/**
		 * callback reporting a CAN error
		 * (rarely happens as e.g. the send-function already fails when we have an erroneous CAN-Bus)
		 */
		void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
		{
			debugOutPinSet(OUT_DEBUG_SIGNAL2);
			// seems only to be called once after an error conditions has been resolved.
			totalCanErrors++;
			flashErrorLed();
			debugOutPinReset(OUT_DEBUG_SIGNAL2);
		}

		/**
		 * SysTick-Callback (executed every 1ms) scheduling CAN sending.
		 *
		 * Because the SysTick has a rather high prio (and we probably don't wanna change that),
		 * we schedule a lower-prio interrupt that does the actual sending.
		 */
		void HAL_SYSTICK_Callback(void)
		{
			static uint32_t canTick = 0;

			debugOutPinSet(OUT_DEBUG_SIGNAL3);
			canTick = (canTick+1) % canReportSendInterval;

			if (canTick == 0)
			{
				reportStateTim2.scheduledForSending = true;
				__HAL_GPIO_EXTI_GENERATE_SWIT(SEND_CAN_SOFTWARE_INTERRUPT);
			}

			// the second frame will be send 2ms after the first (so they are close together, but don't overlap)
			// (although it probably wouldn't be a problem for the sends to overlap, i just wanted the timing it a bit more defined)
			if (canTick == min(2, canReportSendInterval-1) )
			{
				reportStateTim3.scheduledForSending = true;
				__HAL_GPIO_EXTI_GENERATE_SWIT(SEND_CAN_SOFTWARE_INTERRUPT);
			}

			debugOutPinReset(OUT_DEBUG_SIGNAL3);
		}

		/**
		 * Send CAN-Frames using a software-interrupt.
		 * this which has lower PRIO than SysTick & Input-Capture but higher than UART
		 */
		void HAL_GPIO_EXTI_Callback(uint16_t source)
		{
			debugOutPinSet(OUT_DEBUG_SIGNAL2);

			if ((source & SEND_CAN_SOFTWARE_INTERRUPT) == SEND_CAN_SOFTWARE_INTERRUPT)
			{
				if (reportStateTim2.scheduledForSending)
				{
					reportStateTim2.scheduledForSending = false;
					sendCanFrame(&reportStateTim2);
				}

				if (reportStateTim3.scheduledForSending)
				{
					reportStateTim3.scheduledForSending = false;
					sendCanFrame(&reportStateTim3);
				}
			}

			debugOutPinReset(OUT_DEBUG_SIGNAL2);
		}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

			// We have to move the ISR because we're being run from a Bootloader and have a different Base-Address.
			// see definition of "FLASH" in STM32F103RBTx_FLASH.ld
			// The bootloader would actually set this correctly but HAL is changing it back to 0x8000000 (in function SystemInit())
			SCB->VTOR = 0x8005000;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */


		// ************** SETUP PROCEDURE *********************

		// Backup reset-flags (reason for the reset) and clear them from the register for next reset
		resetFlagsBackup = RCC->CSR;
		SET_BIT(RCC->CSR, RCC_CSR_RMVF);	// setting this bit clears the reset-flags

		// Only relevant for debugging... stop timers 1-4 while in a breakpoint
		__HAL_DBGMCU_FREEZE_TIM1();
		__HAL_DBGMCU_FREEZE_TIM2();
		__HAL_DBGMCU_FREEZE_TIM3();
		__HAL_DBGMCU_FREEZE_TIM4();

		// Check whether the DebugEnable-Jumper has been set (connect UExt7 <-> UExt8).
		// (note UExt7 is the same as D12, and UExt8 just outputs GND)
		if (isDebugEnablePinSet())
		{
			// Actually start the PWM operation for the 2 Debug-PWM output channels
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);	// start PWM on Timer1-CH1 / PA_8 / Pin: D6)
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);	// start PWM on Timer4-CH1 / PB_6 / Pin: D5)
		}
		else
		{
			// De-initialize all debug outputs again
			deinitAllDebugOutputPins();
			isSerialOutputEnabled = false;
		}

		// Enable basic Timer interrupts for TIM 2&3 (in our case OVERFLOW)
		// Note: With OVERFLOW I mean "UIF (Update-Interrupt)" while Period being set to 65535
		HAL_TIM_Base_Start_IT(&htim2);
		HAL_TIM_Base_Start_IT(&htim3);

		// Change the current counter values of the IC-Timers so the overflows aren't happening at the same time
		// not really necessary, it's just to spread out the interrupts a bit more
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim3, 0x7fff);

		// Initialize Input-Capture interrupt-handlers
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);		// TIM2_CH1 / PA_0  / D2
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);		// TIM2_CH3 / PA_2  / D1
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);		// TIM2_CH4 / PA_03 / D0
		HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);		// TIM3_CH1 / PC_06 / D35
		HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);		// TIM3_CH2 / PC_07 / D36
		HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);		// TIM3_CH3 / PC_08 / D37

		// We also change the channels that we don't use (and don't have free GPIO pins) to INPUT_CAPTURE mode
		// (rather then the default output-compare) otherwise they would produce interrupts on overflow.
		SET_BIT(htim2.Instance->CCMR1, TIM_CCMR1_CC2S_0); // TIM2_CH2 - set to input-capture, without a channel being defined
		CLEAR_BIT(htim2.Instance->SR, TIM_SR_CC2IF);	  //          - and clear interrupts flag if it was set before
		SET_BIT(htim3.Instance->CCMR2, TIM_CCMR2_CC4S_0); // TIM3_CH4 - set to input-capture, without a channel being defined
		CLEAR_BIT(htim3.Instance->SR, TIM_SR_CC4IF);	  //          - and clear interrupts flag if it was set before

		// Initialize UART to receive interrupts
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

		// Disable edge detection on GPIO-Pin "UNUSED_EXTI10" again.
		// We didn't really want to use the "external" interrupt.
		// We just added it so we could configure the interrupt-handler in CubeMX, and use it as a software-interrupt
		CLEAR_BIT(EXTI->RTSR, UNUSED_EXTI10_Pin);
        CLEAR_BIT(EXTI->FTSR, UNUSED_EXTI10_Pin);
		__HAL_GPIO_EXTI_CLEAR_IT(SEND_CAN_SOFTWARE_INTERRUPT); // just to make sure, clear interrupt flag initially


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


		uint32_t lastDebugReportTime = 0;
		while (1)
		{
			uint32_t tick = HAL_GetTick();

			// Reset Watchdog. Needs to be called every 2 seconds at the current setting
			HAL_IWDG_Refresh(&hiwdg);

			// Clear error LED, if there are no errors after 300ms (or the counter ran over)
			if (lastErrorEventTime != 0 && (tick > lastErrorEventTime +300 || tick < lastErrorEventTime -100))
			{
				pinReset(OUT_LED_GREEN);
				lastErrorEventTime = 0;
			}

			// If enabled, create a debug-report on USART every second
			if (isSerialOutputEnabled)
			{
				if (tick > lastDebugReportTime + UART_DEBUG_REPORT_INTERVAL || tick < lastDebugReportTime)
				{
					lastDebugReportTime = tick;
					createUsartDebugReport(&reportStateTim2);
					createUsartDebugReport(&reportStateTim3);
					HAL_printf("---\r\n");
				}
			}

			#ifdef DEBUG_PINS_FOR_PERFORMANCE_MEASURE_ENABLED
				// spend a little bit more time and then toggle a pin, so we can measure the time spent in main loop (with a logic analyzer)
				for (int i=1; i<30; i++)
				{
				}
				debugOutPinToggle(OUT_DEBUG_SIGNAL0);
			#endif


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_13TQ;
  hcan.Init.BS2 = CAN_BS2_2TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = ENABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 1250;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = INPUT_CAPTURE_PRESCALER - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = INPUT_CAPTURE_FILTER_VALUE;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = INPUT_CAPTURE_PRESCALER - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = INPUT_CAPTURE_FILTER_VALUE;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OUT_DEBUG_SIGNAL0_Pin|OUT_DEBUG_SIGNAL1_Pin|OUT_DEBUG_SIGNAL2_Pin|OUT_DEBUG_SIGNAL3_Pin 
                          |OUT_DEBUG_SIGNAL4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OUT_LED_YELLOW_Pin|OUT_LED_GREEN_Pin|OUT_DEBUG_ALLWAYS_LOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUT_USB_DISC_GPIO_Port, OUT_USB_DISC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : RES_CAN_CTRL_Pin IN_BOOT_BTN_Pin IN_USB_P_Pin */
  GPIO_InitStruct.Pin = RES_CAN_CTRL_Pin|IN_BOOT_BTN_Pin|IN_USB_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_DEBUG_SIGNAL0_Pin OUT_DEBUG_SIGNAL1_Pin OUT_DEBUG_SIGNAL2_Pin OUT_DEBUG_SIGNAL3_Pin 
                           OUT_DEBUG_SIGNAL4_Pin */
  GPIO_InitStruct.Pin = OUT_DEBUG_SIGNAL0_Pin|OUT_DEBUG_SIGNAL1_Pin|OUT_DEBUG_SIGNAL2_Pin|OUT_DEBUG_SIGNAL3_Pin 
                          |OUT_DEBUG_SIGNAL4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_LED_YELLOW_Pin OUT_LED_GREEN_Pin OUT_DEBUG_ALLWAYS_LOW_Pin */
  GPIO_InitStruct.Pin = OUT_LED_YELLOW_Pin|OUT_LED_GREEN_Pin|OUT_DEBUG_ALLWAYS_LOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IN_DEBUG_ENABLED_Pin */
  GPIO_InitStruct.Pin = IN_DEBUG_ENABLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IN_DEBUG_ENABLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UNUSED_EXTI10_Pin */
  GPIO_InitStruct.Pin = UNUSED_EXTI10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(UNUSED_EXTI10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OUT_USB_DISC_Pin */
  GPIO_InitStruct.Pin = OUT_USB_DISC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUT_USB_DISC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RES_TRST_Pin */
  GPIO_InitStruct.Pin = RES_TRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RES_TRST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

		// usually just happens with API-Usage errors. Just report on USART (if enable) and try to continue.
		totalHalErrors++;
		flashErrorLed();
		HAL_printf("ERROR in File: %s, line: %d\r\n", file, line);

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
