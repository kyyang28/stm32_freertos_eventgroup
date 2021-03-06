
#include <stdio.h>
#include <string.h>

#include "target.h"
#include "system_stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"
#include "nvic.h"
#include "system.h"
#include "led.h"
#include "sound_beeper.h"
#include "config.h"
#include "configMaster.h"
#include "config_eeprom.h"
#include "exti.h"
#include "button.h"
#include "serial.h"
#include "serial_uart_impl.h"
//#include "msp_serial.h"
//#include "printf.h"
#include "gps.h"
#include "rxSerial1Test.h"
#include "rxSerial3Test.h"
#include "rxSerial6Test.h"
#include "bus_i2c.h"
#include "bitband_i2c_soft.h"					// self-implemented i2c protocol
#include "bus_spi.h"
#include "initialisation.h"
#include "accgyro_spi_mpu9250.h"
#include "gyro.h"
#include "acceleration.h"
#include "maths.h"

//#include "mpu6050.h"
//#include "mpu6050_soft_i2c.h"					// for MPU6050 testing purposes
#include "mpu9250_soft_i2c.h"					// for MPU9250 testing purposes
//#include "inv_mpu.h"							// for MPU6050/MPU9250 DMP testing purposes
//#include "inv_mpu_dmp_motion_driver.h"			// for MPU6050/MPU9250 DMP testing purposes

#include "pwm_output.h"			// including timer.h ledTimer.h
#include "rx_pwm.h"
#include "rx.h"
#include "feature.h"

#include "time.h"				// allow to use timeUs_t which is uint32_t
#include "fc_core.h"
#include "fc_tasks.h"

#include "mixer.h"

#include "debug.h"

#include "sdcard.h"
#include "asyncfatfs.h"

#include "blackbox.h"
#include "blackbox_io.h"

#ifdef USE_RTOS
/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"           // task operations
#include "list.h"           // list operations
#include "queue.h"          // queue operations
#include "semphr.h"         // semaphore operations
#endif

//#define GPIO_PA1_PIN				PA1
//#define GPIO_PB8_PIN				PB8

//uint32_t counter = 0;

//uint32_t currentTimeUs;

//uint32_t base = 1;
//uint32_t sub = 2;			// build = 0x00000050
//uint32_t build;
//uint32_t tmp1, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7, tmp8;

//uint8_t __g_basepri_save, __g_basepri_save2, __g_basepri_save3;
//uint8_t __g_ToDo, __g_ToDo2;
//uint8_t __basepriRestoreMem_inter;
//uint8_t basepri_val;
//int forCnt = 0;

//static IO_t g_scl;
//static IO_t g_sda;

#ifdef USE_RTOS

/* +------------------------------------------------------------------------------------------+ */
/* +--------------------------------------- FreeRTOS ------------------------------------------ */
/* +------------------------------------------------------------------------------------------+ */

#define START_TASK_PRIO                             1               // task priority
#define START_STK_SIZE                              256             // task stack size
TaskHandle_t StartTask_Handler;                                     // task handler
void start_task(void *pvParameters);                                // task callback function

#define SEMAGIVE_TASK_PRIO                          2
#define SEMAGIVE_STK_SIZE                           256
TaskHandle_t SemagiveTask_Handler;
void semagive_task(void *pvParameters);

#define SEMATAKE_TASK_PRIO                          3
#define SEMATAKE_STK_SIZE                           256
TaskHandle_t SematakeTask_Handler;
void sematake_task(void *pvParameters);

/* Declare semaphore variable */
SemaphoreHandle_t CountingSemaphore;

/* +------------------------------------------------------------------------------------------+ */
/* +--------------------------------------- FreeRTOS ------------------------------------------ */
/* +------------------------------------------------------------------------------------------+ */

#endif

typedef enum {
	SYSTEM_STATE_INITIALISING			= 0,
	SYSTEM_STATE_CONFIG_LOADED			= (1 << 0),
	SYSTEM_STATE_SENSORS_READY			= (1 << 1),
	SYSTEM_STATE_MOTORS_READY			= (1 << 2),
	SYSTEM_STATE_TRANSPONDER_ENABLED	= (1 << 3),
	SYSTEM_STATE_ALL_READY				= (1 << 7)
}systemState_e;

//int g_val = 0;

uint8_t systemState = SYSTEM_STATE_INITIALISING;
//IO_t PA1_PIN, PB8_PIN;


void systemInit(void);
//void gpioPA1Init(void);
//void gpioPA1Ops(void);
//void gpioPB8Init(void);
//void gpioPB8Ops(void);

//int i_cnt = 0;
//double f_cnt = 0.0;

#if 1
struct __FILE
{
    int dummy;
};

FILE __stdout;

int fputc(int ch, FILE *f)
{
    /* Send byte to USART */
//	gpsWrite(ch);
//	rxSerial1TestWrite(ch);
	rxSerial3TestWrite(ch);
//	rxSerial6TestWrite(ch);
    
    /* If everything is OK, you have to return character written */
    return ch;
    /* If character is not correct, you can return EOF (-1) to stop writing */
    //return -1;
}
#else
#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	rxSerial3TestWrite(ch);
	return ch;
}
#endif

//static IO_t mpuSpi9250CsPin = IO_NONE;

//bool sdaFlag = false;

//uint32_t TIM_ARR = 0;
//int16_t ledDutyCycle[LED_NUMBER];

//extern uint8_t TIM_CAPTURE_STATUS;			// status of timer input capture

extern uint8_t motorControlEnable;

#ifdef USE_RTOS
extern button_t button;
void semagive_task(void *pvParameters)
{
    uint8_t i = 0;
    uint8_t semaValue;
    BaseType_t err;
    
    while (1) {
        
        if (CountingSemaphore != NULL) {
            if (button.dev.btnPressed) {
                err = xSemaphoreGive(CountingSemaphore);        // Release counting semaphore
                if (err == pdFALSE) {
                    printf("Failed to release counting semaphore");
                }
                
                /* Acquire counting semaphore */
                semaValue = uxSemaphoreGetCount(CountingSemaphore);
                printf("Counting semaphore value: %u, %s\r\n", semaValue, __FUNCTION__);
            }
        }
        
        i++;
        
        if (i == 50) {
            i = 0;
            LedToggle(LED3_NUM);
        }
        
        vTaskDelay(10);
    }
}

void sematake_task(void *pvParameters)
{
    uint8_t semaValue;
    
    while (1) {
        xSemaphoreTake(CountingSemaphore, portMAX_DELAY);       // Wait for counting semaphore and get it
        semaValue = uxSemaphoreGetCount(CountingSemaphore);
        printf("Counting semaphore value: %u, %s\r\n", semaValue, __FUNCTION__);        
        LedToggle(LED4_NUM);
        vTaskDelay(1000);
    }
}

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();

    /* Create binary semaphore */
    CountingSemaphore = xSemaphoreCreateCounting(255, 0);
    
    /* Create semaphore give task */
    xTaskCreate((TaskFunction_t         )semagive_task,
                (const char *           )"semagive_task",
                (uint16_t               )SEMAGIVE_STK_SIZE,
                (void *                 )NULL,
                (UBaseType_t            )SEMAGIVE_TASK_PRIO,
                (TaskHandle_t *         )&SemagiveTask_Handler);
                
    /* Create semaphore take task */
    xTaskCreate((TaskFunction_t         )sematake_task,
                (const char *           )"sematake_task",
                (uint16_t               )SEMATAKE_STK_SIZE,
                (void *                 )NULL,
                (UBaseType_t            )SEMATAKE_TASK_PRIO,
                (TaskHandle_t *         )&SematakeTask_Handler);
    
    vTaskDelete(StartTask_Handler);
    
    taskEXIT_CRITICAL();
}

void TimerInitForFreeRTOS(TIM_TypeDef *tim, uint16_t period, uint8_t khz, uint8_t irqNum, uint8_t premptPrio, uint8_t subPrio)
{
    /* Configure TIMER 4 */
    configTimeBaseKhz(tim, period, khz);
    
    TIM_ITConfig(tim, TIM_IT_Update, ENABLE);      // Allow TIM4 updating interrupts
    TIM_Cmd(tim, ENABLE);                          // Enable TIMER 4
    
    timerNVICConfigure4FreeRTOS(irqNum, premptPrio, subPrio);    
}
#endif

int main(void)
{
//	printfSupportInit();
	
	systemInit();
	
	/* Initialise IO (needed for all IO operations) */
	IOGlobalInit();
	
	/* Initialise EEPROM */
	initEEPROM();
	
	/* Check if EEPROM contains valid data */
	checkEEPROMContainsValidData();

	/* Initialise serial port usage list */
	serialInit(SerialConfig());

	/* Initialise debugging serial port */
//	rxSerial1TestInit();
	rxSerial3TestInit();
//	rxSerial6TestInit();

	/* Write masterConfig info into FLASH EEPROM
	 *
	 * TODO: (ISSUE) After calling writeEEPROM() function, the program will not be running when STM32F4 board is powered on
	 */
//	writeEEPROM();	// TODO: writeEEPROM() should be included inside the checkEEPROMContainsValidData() function
					//       separated here just for using printf (serialInit and rxSerial3TestInit initialised before writeEEPROM() and readEEPROM())

	/* Read masterConfig info from FLASH EEPROM */
	readEEPROM();

	/* Initialise MSP (MCU Support Package) serial */
//	mspSerialInit();

	/* IMPORTANT: the printf is functional only if the rxSerial3TestInit() function is initialised
	 * i.e., use printf debugger thereafter
	 */

//	printf("masterConfig.version: %u, %d\r\n", masterConfig.version, __LINE__);
//	printf("masterConfig.size: %u, %d\r\n", masterConfig.size, __LINE__);
//	printf("masterConfig.magic_be: 0x%x, %d\r\n", masterConfig.magic_be, __LINE__);
//	printf("masterConfig.magic_ef: 0x%x, %d\r\n", masterConfig.magic_ef, __LINE__);
//	printf("masterConfig.chk: %u, %d\r\n", masterConfig.chk, __LINE__);

//	gpsInit();	
//	gpsSetPrintfSerialPort();

	/* LedTimerConfig()->ioTags[xxx] initialisation done correctly */
//	printf("LedTimerConfig()->ioTags[0]: 0x%x\r\n", LedTimerConfig()->ioTags[0]);	// LED4, PD12, ioTag_t 0x4C
//	printf("LedTimerConfig()->ioTags[1]: 0x%x\r\n", LedTimerConfig()->ioTags[1]);	// LED3, PD13, ioTag_t 0x4D
//	printf("LedTimerConfig()->ioTags[2]: 0x%x\r\n", LedTimerConfig()->ioTags[2]);	// LED5, PD14, ioTag_t 0x4E
//	printf("LedTimerConfig()->ioTags[3]: 0x%x\r\n", LedTimerConfig()->ioTags[3]);	// LED6, PD15, ioTag_t 0x4F
	
	/* PwmConfig()->ioTags[xxx] initialisation done correctly */
//	printf("PwmConfig()->ioTags[0]: 0x%x\r\n", PwmConfig()->ioTags[0]);
//	printf("PwmConfig()->ioTags[1]: 0x%x\r\n", PwmConfig()->ioTags[1]);
//	printf("PwmConfig()->ioTags[2]: 0x%x\r\n", PwmConfig()->ioTags[2]);
//	printf("PwmConfig()->ioTags[3]: 0x%x\r\n", PwmConfig()->ioTags[3]);
//	printf("PwmConfig()->ioTags[4]: 0x%x\r\n", PwmConfig()->ioTags[4]);
//	printf("PwmConfig()->ioTags[5]: 0x%x\r\n", PwmConfig()->ioTags[5]);
//	printf("PwmConfig()->ioTags[6]: 0x%x\r\n", PwmConfig()->ioTags[6]);
//	printf("PwmConfig()->ioTags[7]: 0x%x\r\n", PwmConfig()->ioTags[7]);

	/* Read the contents of EEPROM */
//	ReadEEPROM();
	
	systemState |= SYSTEM_STATE_CONFIG_LOADED;
	
	debugMode = masterConfig.debug_mode;
	
	/* IMPORTANT: 
	 * 		DO NOT FORGET TO CALL latchActiveFeatures() function to perform the following action
	 *			activeFeaturesLatch = masterConfig.enabledFeatures
	 * Latch active features to be used for feature() in the remainder of init()
	 */
	latchActiveFeatures();
//	printf("masterConfig.enabledFeatures: 0x%x, %s, %d\r\n", masterConfig.enabledFeatures, __FUNCTION__, __LINE__);		// 0x2000 (1 << 13) FEATURE_RX_PARALLEL_PWM
	
	/* Initialise leds */
	LedInit(LedStatusConfig());
	
	/* Initialise external interrupt */
	EXTIInit();
	
	/* allow configuration to settle */
//	delay(100);
	
//	LedSet(LED3_NUM, true);		// LED3_NUM = 0
//	LedSet(LED4_NUM, true);		// LED4_NUM = 1
//	LedSet(LED5_NUM, true);		// LED5_NUM = 2
//	LedSet(LED6_NUM, true);		// LED6_NUM = 3

//	LED3_ON;
//	LED4_ON;
//	LED5_ON;
//	LED6_ON;

//	userBtnPollInit();
	
	/* Initialisae button with interrupt */
	buttonInit();

	/* Timer must be initialised before any channel is allocated
     *
     * Enable TIMERS.
     */
	timerInit();					// reinitialise the LED IO configuration to timer AF_PP if USE_LEDTIMER has been set.
									// INFO: To use NORMAL LEDs, turn off the USE_LEDTIMER micro in target.h

#ifdef USE_RTOS
    /* +-------------------------------- Initialise TIM4 ----------------------------------+ */
    
    /* TIM_TimeBaseStructure.TIM_Period = (period - 1) & 0xFFFF 
     *
     * uint16_t period = 10000
     *
     * TIM_TimeBaseStructure.TIM_Period = (10000 - 1) & 0xFFFF = 9999 & 65535 = 9999
     */
    uint16_t period = 10000;
    
    /* TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / timerClockDivisor(tim) / ((uint32_t)pscKhz * 1000)) - 1
     * 
     * pscKhz = 10
     *
     * timerClockDivisor(TIM3) = 2, timerClockDivisor(TIM4) = 2
     * 
     * TIM_TimeBaseStructure.TIM_Prescaler = (168000000 / 2 / (10 * 1000)) - 1 = 8400 - 1 = 8399
     */
    uint8_t pscKhz = 10;
    uint8_t TIM3PremptPriority = 0x04;
    
    /* TIM4 interrupt will be disabled when calling portDISABLE_INTERRUPTS() as configMAX_SYSCALL_INTERRUPT_PRIORITY set to 5
     * All interrupt priorities greater than 5 will be disabled by calling portDISABLE_INTERRUPTS()
     */
    uint8_t TIM4PremptPriority = 0x05;
    uint8_t SubPriority = 0x00;
    
    TimerInitForFreeRTOS(TIM3, period, pscKhz, TIM3_IRQn, TIM3PremptPriority, SubPriority);
    TimerInitForFreeRTOS(TIM4, period, pscKhz, TIM4_IRQn, TIM4PremptPriority, SubPriority);

    /* TIM_TimeBaseStructure.TIM_Period = (period - 1) & 0xFFFF 
     *
     * uint16_t period = 10000
     *
     * TIM_TimeBaseStructure.TIM_Period = (10000 - 1) & 0xFFFF = 9999 & 65535 = 9999
     */
    uint16_t periodForTIM2 = 5000;
    
    /* TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / timerClockDivisor(tim) / ((uint32_t)pscKhz * 1000)) - 1
     * 
     * pscKhz = 10
     *
     * timerClockDivisor(TIM3) = 2, timerClockDivisor(TIM4) = 2
     * 
     * TIM_TimeBaseStructure.TIM_Prescaler = (168000000 / 2 / (10 * 1000)) - 1 = 8400 - 1 = 8399
     */
    uint8_t pscKhzForTIM2 = 10;
    uint8_t TIM2PremptPriority = 0x03;

    TimerInitForFreeRTOS(TIM2, periodForTIM2, pscKhzForTIM2, TIM2_IRQn, TIM2PremptPriority, SubPriority);
#endif

#if defined(USE_LEDTIMER)
//	uint16_t idlePulse = LedTimerConfig()->mincommand;
	uint16_t idlePulse = 0;
#if 0
//	uint32_t KhzGain = 50;
	uint32_t KhzGain = 500;
//	printf("idlePulse: %u, %s, %d\r\n", idlePulse, __FUNCTION__, __LINE__);		// idlePulse = 1000 (mincommand)

	ledTimerKhzInit(LedTimerConfig(), idlePulse, LED_NUMBER, KhzGain);
	
	TIM_ARR = 10 * KhzGain;	// KhzGain = 500, TIM_ARR = 10*500 = 5000 (5000 / 10000 = 0.5s = 500ms)
	
//	printf("TIM_ARR: %u\r\n", TIM_ARR);
	
	ledDutyCycle[0] = TIM_ARR * 0.75;		// TIM4 CH1 LED4 PD12
	ledDutyCycle[1] = TIM_ARR * 0.50;		// TIM4 CH2 LED3 PD13
	ledDutyCycle[2] = TIM_ARR * 0.25;		// TIM4 CH3 LED5 PD14
	ledDutyCycle[3] = TIM_ARR * 0.20;		// TIM4 CH4 LED6 PD15
	
	for (int i = 0; i < LED_NUMBER; i++) {
		pwmWriteLed(i, ledDutyCycle[i]);
	}
#else
//	ledTimerMhzInit(LedTimerConfig(), idlePulse, LED_NUMBER);
#endif
#endif
    
#if 0
    /* Disable the following codes to NOT using TIMER functionalities */

	/* Initialise the ESC endpoints */
	mixerInit(MixerConfig()->mixerMode, masterConfig.customMotorMixer);
	
	/* Configurate the quadcopter mixer */
	mixerConfigurationOutput();				// motorCount = QUAD_MOTOR_COUNT, reset disarmed motor output to mincommand(1000)
	
	uint16_t bldcMotoridlePulse = MotorConfig()->mincommand;
	motorInit(MotorConfig(), bldcMotoridlePulse, getMotorCount());
#endif
	
#if defined(USE_PWM)			// USE_PWM is defined in target.h
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	if (feature(FEATURE_RX_PARALLEL_PWM)) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		pwmRxInit(PwmConfig());
	}
	
//	IO_t pwmIO = IOGetByTag(PwmConfig()->ioTags[0]);					// IOGetByTag() converts ioTag_t to IO_t
//	printf("IO_GPIO(pwmIO): 0x%x\r\n", (uint32_t)IO_GPIO(pwmIO));		// gpio = 0x40020000 (GPIOA)
//	printf("IO_Pin(pwmIO): %u\r\n", IO_Pin(pwmIO));						// pin = 1 = 0x1 (1 << 0)
#endif

	/* Testing USER button (PA0) high voltage level time period using TIM5 Input Capture mode */
//	TIM5_CH1_Cap_Init(0xFFFFFFFF, 84 - 1);	// 1MHz counter frequency, period (ARR) = 0xFFFFFFFF, psc = 84 - 1 = 83 (1MHz, 1us)
////	TIM5_CH1_Cap_Init(0x10000, 84 - 1);	// 1MHz counter frequency, period (ARR) = 0x10000, psc = 84 - 1 = 83 (1MHz, 1us), 0x10000 is not a proper value to measure the user button high voltage level
	
	/* +--------- TIMER LED TESTING PURPOSES ONLY ---------+ */
//	board_leds_init();
//	timer_clock_init();
//	timer_pwm_init();
//	timer_start();
////	flash_green_led_forever();
	/* +--------- TIMER LED TESTING PURPOSES ONLY ---------+ */

#ifdef USE_SPI			// USE_SPI is defined in target.h
	#ifdef USE_SPI_DEVICE_1
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		spiInit(SPIDEV_1);		// SPIDEV_1 = 0
	#endif
	#ifdef USE_SPI_DEVICE_2
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		spiInit(SPIDEV_2);
	#endif
#endif

#ifdef BEEPER
	beeperInit(BeeperConfig());
#endif

#ifdef USE_I2C			// USE_I2C is defined in target.h
	/* Initialise I2C device */
	i2cInit(I2C_DEVICE);
#endif

#if 0
//	IIC_Init();
#endif
	
//	g_scl = IOGetByTag(IO_TAG(SOFT_I2C_SCL));				// PB10
//	g_sda = IOGetByTag(IO_TAG(SOFT_I2C_SDA));				// PB11
//	ioRec_t *ioRecSDA = IO_Rec(g_sda);
//	printf("IO_GPIO(g_scl): 0x%x\r\n", (uint32_t)IO_GPIO(g_scl));		// gpio = 0x40021000 (GPIOE)
//	printf("IO_Pin(g_scl): %u\r\n", IO_Pin(g_scl));					// pin = 16 = 0x10 (1 << 4)
//	printf("IO_GPIO(g_sda): 0x%x\r\n", (uint32_t)IO_GPIO(g_sda));		// gpio = 0x40021000 (GPIOE)
//	printf("IO_Pin(g_sda): %u\r\n", IO_Pin(g_sda));					// pin = 16 = 0x10 (1 << 4)
	
//	IOConfigGPIO(g_scl, IOCFG_OUT_PP);
//	IOConfigGPIO(g_sda, IOCFG_OUT_PP);
//	IOConfigGPIO(g_scl, IOCFG_OUT_OD);
//	IOConfigGPIO(g_sda, IOCFG_OUT_OD);

//#define YELLOW_LED		PE4
//	IO_t yellowLedPin = IO_NONE;
//	yellowLedPin = IOGetByTag(IO_TAG(YELLOW_LED));
//	IOInit(yellowLedPin, OWNER_LED, 0);
//	IOConfigGPIO(yellowLedPin, IOCFG_OUT_PP);

//	mpuSpi9250CsPin = IOGetByTag(IO_TAG(MPU9250_CS_PIN));
	
//	ioRec_t *ioRecMPUSpi9250Cs = IO_Rec(IOGetByTag(IO_TAG(MPU9250_CS_PIN)));
//	printf("ioRecMPUSpi9250Cs->gpio: 0x%x\r\n", (uint32_t)ioRecMPUSpi9250Cs->gpio);		// gpio = 0x40021000 (GPIOE)
//	printf("ioRecMPUSpi9250Cs->pin: %u\r\n", ioRecMPUSpi9250Cs->pin);					// pin = 16 = 0x10 (1 << 4)

//	IOInit(mpuSpi9250CsPin, OWNER_MPU_CS, 0);

//	printf("ioRecMPUSpi9250Cs->owner: %u\r\n", ioRecMPUSpi9250Cs->owner);				// owner = 11
//	printf("ioRecMPUSpi9250Cs->index: %u\r\n", ioRecMPUSpi9250Cs->index);				// index = 0
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);		// ok
//	printf("IO_GPIOPortIdx(mpuSpi9250CsPin): %d\r\n", IO_GPIOPortIdx(mpuSpi9250CsPin));		// IO_GPIOPortIdx(mpuSpi9250CsPin) = 4
//	printf("DEFIO_IO_USED_COUNT: %d\r\n", DEFIO_IO_USED_COUNT);		// DEFIO_IO_USED_COUNT = 80
//	IOConfigGPIO(mpuSpi9250CsPin, SPI_IO_CS_CFG);
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);		// ok

#if 0
	short gyro_x, gyro_y, gyro_z;
	short acc_x, acc_y, acc_z;
	float temp = 0.0;
	float roll, pitch, yaw;				// Euler angles
#endif
	
#if 0
	uint8_t res = MPU_Init();
	printf("MPU_Init result: %u, %s, %d\r\n", res, __FUNCTION__, __LINE__);
#endif

#if 0
	while (mpu_dmp_init()) {
		printf("MPU6050 initialisation error!, %s, %d\r\n", __FUNCTION__, __LINE__);
		delay(200);
	}
	
	printf("MPU6050 initialisation is done using mpu_dmp_init @%s, %d\r\n", __FUNCTION__, __LINE__);
#endif

#if 0
	if (!MPU6050_Soft_I2C_Init()) {
		printf("MPU6050 initialisation is done @%s, %d\r\n", __FUNCTION__, __LINE__);
	}else {
		printf("Failed to initialise MPU6050 @%s, %d\r\n", __FUNCTION__, __LINE__);
		return -1;
	}
#endif

#if 0
	if (!MPU9250_Soft_I2C_Init()) {
		printf("MPU9250 initialisation is done @%s, %d\r\n", __FUNCTION__, __LINE__);
	}else {
		printf("Failed to initialise MPU9250 @%s, %d\r\n", __FUNCTION__, __LINE__);
		return -1;
	}
#endif
	
//	bool flag = false;

#if defined(USE_IMU)			// USE_IMU is defined in target.h
	if (!sensorsAutodetect(GyroConfig(), AccelerometerConfig())) {
		//failureMode();
		printf("Failed to initialise IMU!, %s, %d\r\n", __FUNCTION__, __LINE__);
		while (1) {
			/* BLUE LED */
			LED6_ON;
			delay(100);
			LED6_OFF;
			delay(100);
		}
	}
#endif
	
	systemState |= SYSTEM_STATE_SENSORS_READY;
	
//	printf("Setup completed!\r\n");
	
//	gpioPA1Init();
//	gpioPB8Init();


#if defined(BEEPER)
	/* Board power on beeper */
	for (int i = 0; i < 10; i++) {
		delay(25);
		BEEP_ON;
		delay(25);
		BEEP_OFF;
	}
#endif
	
//	delay(3000);

	/* Testing standard deviation functions */
//	stdev_t var;
//	float dataset[] = {727.7, 1086.5, 1091.0, 1361.3, 1490.5, 1956.1};
//	devClear(&var);
//	for (int i = 0; i < sizeof(dataset)/sizeof(dataset[0]); i++) {
//		devPush(&var, dataset[i]);
//	}
//	float dev = devStandardDeviation(&var);
//	printf("dev: %f, %s, %d\r\n", dev, __FUNCTION__, __LINE__);					// 420.962463
//	printf("devint: %ld, %s, %d\r\n", lrintf(dev), __FUNCTION__, __LINE__);		// 421

	/* Initialise RX receiver */
	rxInit(RxConfig(), ModeActivationProfile()->modeActivationConditions);

#if 0
#ifdef USE_SDCARD
	if (feature(FEATURE_SDCARD) && BlackboxConfig()->device == BLACKBOX_SDCARD) {
//		printf("USE_SDCARD, %s, %d\r\n", __FUNCTION__, __LINE__);
		sdcardInsertionDetectInit();
		sdcard_init(SdcardConfig()->useDma);		// SdcardConfig()->useDma = false for now, use DMA later
		afatfs_init();
//		if (!sdcard_isInserted()) {
//			printf("SDCARD is not present!\r\n");
//		}else {
//			printf("SDCARD is present!\r\n");
//		}
	}
#endif
#endif

#if 0
#ifdef BLACKBOX
	initBlackbox();
#endif
#endif

#if defined(USE_IMU)
	/* set gyro calibration cycles */
	gyroSetCalibrationCycles();
#endif

//	const timeUs_t currentTimeUsAfter = micros();
//	printf("currentTimeUsAfter processRx: %u, %s, %d\r\n", currentTimeUsAfter, __FUNCTION__, __LINE__);

	/* Set the motorControlEnable flag to be TRUE to control the motors */
	motorControlEnable = true;

	uint16_t ledpwmval = 1500;
	uint8_t dir = 1;
	
//	printf("sizeof(master_t): %u\r\n", sizeof(master_t));			// sizeof(master_t): 868 so far


#ifdef USE_RTOS

/* +------------------------------------------------------------------------------------------+ */
/* +--------------------------------------- FreeRTOS ------------------------------------------ */
/* +------------------------------------------------------------------------------------------+ */

/* UBaseType_t = unsigned long */
    /* Create Start task */
    xTaskCreate((TaskFunction_t         )start_task,
                (const char *           )"start_task",
                (uint16_t               )START_STK_SIZE,
                (void *                 )NULL,
                (UBaseType_t            )START_TASK_PRIO,
                (TaskHandle_t *         )&StartTask_Handler);
                
    vTaskStartScheduler();                                          // Start FreeRTOS scheduler

/* +------------------------------------------------------------------------------------------+ */
/* +--------------------------------------- FreeRTOS ------------------------------------------ */
/* +------------------------------------------------------------------------------------------+ */
#else
	/* Main loop */
	while (1) {

//		delay(10);

//		if (dir) {
//			ledpwmval++;
//		}else {
//			ledpwmval--;
//		}
//		
//		if (ledpwmval > 2000) {
//			dir = 0;
//		}
//		
//		if (ledpwmval == 0) {
//			dir = 1;
//		}

//		for (int i = 0; i < getMotorCount(); i++) {
//			pwmWriteMotor(i, ledpwmval);
//		}
//		
////		for (int i = 0; i < LED_NUMBER; i++) {
////			pwmWriteLed(i, ledpwmval);
////		}
				
#if 0
		const timeUs_t currentTimeUs = micros();
//		printf("currentTimeUs before processRx: %u, %s, %d\r\n", currentTimeUs, __FUNCTION__, __LINE__);
		
//		rxSignalReceived = rxSignalReceivedNotDataDriven;		// rxSignalReceivedNotDataDriven initial false
//		rxIsInFailsafeMode = rxIsInFailsafeModeNotDataDriven;	// rxIsInFailsafeModeNotDataDriven initial true
		
		/* TODO: rxUpdateCheck() function will be running in a separate RTOS task */
		rxUpdateCheck(currentTimeUs, 0);			// update the rxSignalReceivedNotDataDriven, rxIsInFailsafeModeNotDataDriven, rxSignalReceived and rxIsInFailsafeMode
	
		taskUpdateRxMain(currentTimeUs);		// calling processRx(), updateRcCommands()
//		processRx(currentTimeUs);		// processRx function is called inside the taskUpdateRxMain function
		
		taskMainPidLoop(currentTimeUs);
		
		delay(20);				// taskUpdateRxMain() update period = 20 ms (20000 us = 50 Hz)
#endif
		
#ifdef BEEPER
//		BEEP_ON;		// turn beeper on
//		delay(200);
//		BEEP_OFF;		// turn beeper off
//		delay(200);

        for (int i = 0; i < 10; i++) {
    //		LED3_TOGGLE;
    //		LED4_TOGGLE;
            delay(25);
            BEEP_ON;
            delay(25);
            BEEP_OFF;
        }
        delay(2000);
#endif
		
//		gyroUpdate();													// read gyro data
//		delay(100);
#if defined(USE_IMU)		// USE_IMU is defined in target.h
		gyroUpdate();													// read gyro data
//		accUpdate(&AccelerometerConfig()->accelerometerTrims);			// read acc data
		if (gyro.dev.mpuDetectionResult.sensor == MPU_60x0 && gyro.dev.calibrationFlag) {
//			printf("MPU6050 data - gyroADCRaw[X]: %d, gyroADCRaw[Y]: %d, gyroADCRaw[Z]: %d, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyro.dev.gyroADCRaw[X], gyro.dev.gyroADCRaw[Y], gyro.dev.gyroADCRaw[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
//			printf("MPU6050 data - gyroADCCali[X]: %d, gyroADCCali[Y]: %d, gyroADCCali[Z]: %d, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyroADC[X], gyroADC[Y], gyroADC[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
//		}else if (gyro.dev.mpuDetectionResult.sensor == MPU_9250_I2C) {
		}else if (gyro.dev.mpuDetectionResult.sensor == MPU_9250_I2C && gyro.dev.calibrationFlag) {
			printf("MPU9250 (I2C) data - gyroADCRaw[X]: %d, gyroADCRaw[Y]: %d, gyroADCRaw[Z]: %d, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyro.dev.gyroADCRaw[X], gyro.dev.gyroADCRaw[Y], gyro.dev.gyroADCRaw[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
//			printf("MPU9250 (I2C) data - gyroADCCali[X]: %d, gyroADCCali[Y]: %d, gyroADCCali[Z]: %d, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyroADC[X], gyroADC[Y], gyroADC[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
//			printf("MPU9250 (I2C) data - gyroADCfiltered[X]: %.4f, gyroADCfiltered[Y]: %.4f, gyroADCfiltered[Z]: %.4f, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyro.gyroADCf[X], gyro.gyroADCf[Y], gyro.gyroADCf[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
		}else if (gyro.dev.mpuDetectionResult.sensor == MPU_9250_SPI && gyro.dev.calibrationFlag) {
//			printf("MPU9250 (SPI) data - gyroADCRaw[X]: %d, gyroADCRaw[Y]: %d, gyroADCRaw[Z]: %d, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyro.dev.gyroADCRaw[X], gyro.dev.gyroADCRaw[Y], gyro.dev.gyroADCRaw[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
//			printf("MPU9250 (SPI) data - gyroADCCali[X]: %d, gyroADCCali[Y]: %d, gyroADCCali[Z]: %d, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyroADC[X], gyroADC[Y], gyroADC[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
			printf("MPU9250 (SPI) data - gyroADCfiltered[X]: %.4f, gyroADCfiltered[Y]: %.4f, gyroADCfiltered[Z]: %.4f\r\n", gyro.gyroADCf[X], gyro.gyroADCf[Y], gyro.gyroADCf[Z]);
//			printf("MPU9250 (SPI) data - gyroADCfiltered[X]: %.4f, gyroADCfiltered[Y]: %.4f, gyroADCfiltered[Z]: %.4f, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyro.gyroADCf[X], gyro.gyroADCf[Y], gyro.gyroADCf[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
		}
//		printf("gyro.dev.gyroADCRaw[X]: %d, %s, %d\r\n", gyro.dev.gyroADCRaw[X], __FUNCTION__, __LINE__);
//		printf("gyro.dev.gyroADCRaw[Y]: %d, %s, %d\r\n", gyro.dev.gyroADCRaw[Y], __FUNCTION__, __LINE__);
//		printf("gyro.dev.gyroADCRaw[Z]: %d, %s, %d\r\n", gyro.dev.gyroADCRaw[Z], __FUNCTION__, __LINE__);
		delay(50);
#endif
		
#if 0
		if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0) {
			/* Get raw data of MPU6050 (gyroscope, accelerometer, temperature) */
			MPU6050_Get_Gyroscope_Data(&gyro_x, &gyro_y, &gyro_z);
			MPU6050_Get_Accelerometer_Data(&acc_x, &acc_y, &acc_z);
			temp = MPU6050_Get_Temperature_Data();
			/* Send data to ANO flight controller software */
			mpu6050_send_data(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
			usart3_report_imu(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, (int)(roll*100), (int)(pitch*100), (int)(yaw*10));
			/* Display on serial terminal */
//			printf("MPU6050 data - gyro_x: %d, gyro_y: %d, gyro_z: %d, acc_x: %d, acc_y: %d, acc_z: %d, temp: %.2f\r\n", gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temp);
//			printf("Euler angles - roll: %.4f, pitch: %.4f, yaw: %.4f\r\n\r\n", roll, pitch, yaw);
//			delay(100);
		}else {
//			printf("mpu_dmp_get_data failed, %s, %d\r\n", __FUNCTION__, __LINE__);
//			delay(100);
		}
#endif

#if 0
		MPU6050_Get_Gyroscope_Data(&gyro_x, &gyro_y, &gyro_z);
		MPU6050_Get_Accelerometer_Data(&acc_x, &acc_y, &acc_z);
		temp = MPU6050_Get_Temperature_Data();
//		printf("MPU6050 gyro data: gyro_x: %d, gyro_y: %d, gyro_z: %d\r\n", gyro_x, gyro_y, gyro_z);
//		printf("MPU6050 temp data: %.2f\r\n", temp);
		printf("MPU6050 data - gyro_x: %d, gyro_y: %d, gyro_z: %d, acc_x: %d, acc_y: %d, acc_z: %d, temp: %.2f\r\n", gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temp);		
		delay(100);
#endif

#if 0
		MPU9250_Get_Gyroscope_Data(&gyro_x, &gyro_y, &gyro_z);
		MPU9250_Get_Accelerometer_Data(&acc_x, &acc_y, &acc_z);
		temp = MPU9250_Get_Temperature_Data();
//		printf("MPU6050 gyro data: gyro_x: %d, gyro_y: %d, gyro_z: %d\r\n", gyro_x, gyro_y, gyro_z);
//		printf("MPU6050 temp data: %.2f\r\n", temp);
		printf("MPU9250 data - gyro_x: %d, gyro_y: %d, gyro_z: %d, acc_x: %d, acc_y: %d, acc_z: %d, temp: %.2f\r\n", gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temp);		
		delay(100);
#endif

#if 0
//#ifdef USE_PWM			// USE_PWM is defined in target.h
		/* Testing USER button (PA0) high voltage level time period using TIM5 Input Capture mode
		 * Make sure to use the following uint32_t type for captures
		 * 	-- uint32_t captures[PWM_PORTS_OR_PPM_CAPTURE_COUNT];
		 *  -- static void pwmEdgeCallback(timerCCHandlerRec_t *cbRec, timCCR_t capture)
		 *  -- static void pwmOverflowCallback(timerOvrHandlerRec_t *cbRec, timCCR_t capture)
		 *  -- 	timCCR_t rise;				// timCCR_t is uint32_t
		 *  -- 	timCCR_t fall;				// timCCR_t is uint32_t
		 *  -- 	timCCR_t capture;				// timCCR_t is uint32_t
		 *  -- 	//	captureCompare_t rise;		// captureCompare_t is uint16_t
		 *  -- 	//	captureCompare_t fall;
		 *  -- 	//	captureCompare_t capture;
		 */
		delay(10);
//		printf("TIM_CAPTURE_STATUS: 0x%x, %s, %d\r\n", TIM_CAPTURE_STATUS, __FUNCTION__, __LINE__);
		if (TIM_CAPTURE_STATUS & 0x80) {			// check if the time period between the rising and falling edge of the signal is captured.
			/* print the time period */
			printf("The period of signal high: %u us\r\n", pwmRead(TIM_Channel_1));		// TIM_Channel_1 = 0x0
			TIM_CAPTURE_STATUS = 0x0;				// clear the status to zero for the next timer input capture
		}
		
		/* Testing USER button (PA0) high voltage level time period using TIM5 Input Capture mode */
//		delay(10);
//		if (TIM5_CH1_CAPTURE_STATUS & 0x80) {		// 0x80: 1 << 7 (capture successful)
//			printf("TIM5_CH1_CAPTURE_VAL: %u\r\n", TIM5_CH1_CAPTURE_VAL);
////			temp = TIM5_CH1_CAPTURE_STATUS & 0x3F;
////			temp *= 0xFFFFFFFF;						// sum of overflow value
//			temp += TIM5_CH1_CAPTURE_VAL;			// retrieve total time period of high voltage level
//			printf("HIGH: %lld us\r\n", temp);
//			TIM5_CH1_CAPTURE_STATUS = 0;			// clear capture status to enable the next input capture
//		}
	/* Testing USER button (PA0) high voltage level time period using TIM5 Input Capture mode */
#endif
		
		/* +----------------------------------------------------------------------------------------+ */
		/* +------------------------------- bus_i2c_soft API testing -------------------------------+ */
		/* +----------------------------------------------------------------------------------------+ */
//		flag = I2C_Start();			// test ok
//		printf("I2C_Start flag: %d, %s, %d\r\n", flag, __FUNCTION__, __LINE__);
//		I2C_Stop();					// test ok
//		I2C_Ack();					// test ok
//		I2C_NoAck();				// test ok
//		flag = I2C_WaitAck();		// test ok
//		printf("I2C_WaitAck flag: %d, %s, %d\r\n", flag, __FUNCTION__, __LINE__);
//		I2C_SendByte(0x59);			// test ok, sends 'Y'
		/* +----------------------------------------------------------------------------------------+ */
		/* +------------------------------- bus_i2c_soft API testing -------------------------------+ */
		/* +----------------------------------------------------------------------------------------+ */
		
//		IIC_Start();
//		delay(1000);

//		IIC_Stop();
//		delay(1000);

//		IIC_Ack();
//		delay(1000);
//		IIC_NAck();
//		delay(1000);
		
//		IIC_SCL = 0;
//		delay(500);
//		IIC_SCL = 1;
//		delay(500);
		
//		IIC_SDA = 0;
//		delay(800);
//		IIC_SDA = 1;
//		delay(800);

//		printf("STM32F407 by QUADYANG using USART3 Port\r\n");
//		delay(1000);
		
//		IOHi(g_sda);
//		IOHi(g_scl);
////		I2C_delay();
//		delay(500);
//		printf("sda: %d\r\n", IORead(g_sda));
		
//		IOHi(g_scl);
//		IOHi(g_sda);
//		delay(500);
//		printf("scl: %d\r\n", IORead(g_scl));
//		printf("sda: %d\r\n", IORead(g_sda));
//		IOLo(g_scl);
//		IOLo(g_sda);
//		delay(500);
//		printf("scl: %d\r\n", IORead(g_scl));
//		printf("sda: %d\r\n", IORead(g_sda));

//		IOHi(g_sda);
//		delay(500);
//		printf("sda: %d\r\n", IORead(g_sda));

//		IOLo(g_sda);
//		delay(500);
//		printf("sda: %d\r\n", IORead(g_sda));		
		
//		IOHi(g_sda);
//		IOHi(g_scl);
////		I2C_delay();
//		delay(1000);
//		sdaFlag = IORead(g_sda);
//		printf("sdaFlag: %d\r\n", sdaFlag);
		
//		I2C_Start();
//		delayMicroseconds(3);
//		I2C_Stop();
//		delayMicroseconds(3);

//		printf("STM32F407 by QUADYANG using USART1 Port\r\n");
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//		f_cnt += 0.01;
//		printf("i_cnt: %d, f_cnt: %.4f\r\n", i_cnt++, f_cnt);
//		delay(1000);

//		IOWrite(yellowLedPin, false);		// high
//		delay(1000);
//		IOWrite(yellowLedPin, true);		// low
//		delay(1000);

//		mpu9250WriteRegister(0x1B, 0x10);
//		delay(1000);
//		uint8_t in;
//		mpu9250ReadRegister(0x1B, 1, &in);
//		printf("in: 0x%x\r\n", in);
		
//		IOLo(mpuSpi9250CsPin);
//		delayMicroseconds(1);
////		delay(1000);
//		spiTransferByte(MPU9250_SPI_INSTANCE, 0x1B);
//		IOHi(mpuSpi9250CsPin);
//		delayMicroseconds(1);

//		IOLo(mpuSpi9250CsPin);
//		delay(800);
//		IOHi(mpuSpi9250CsPin);
//		delay(800);
		
//		printf("QUADYANG\n");
//		delay(1000);
		//counter++;
				
//		userBtnPollOps();

//		gpioPA1Ops();
//		gpioPB8Ops();
		
        LedToggle(LED3_NUM);
        delay(200);
        
//		LED3_ON;
//		delay(500);
//		LED3_OFF;
//		delay(500);
		
//		LED4_ON;
//		delay(700);
//		LED4_OFF;
//		delay(700);

//		LED5_ON;
//		delay(400);
//		LED5_OFF;
//		delay(400);

//		LED6_ON;
//		delay(100);
//		LED6_OFF;
//		delay(100);
		
//		msElapse = millis();
//		currentTimeUs = micros();
	}
#endif  // End with USE_FREERTOS
    //	return 0;
}       // End with main

void EnableGPIOClk(void)
{
	/* AHB1 clocks enable */
	RCC_AHB1PeriphClockCmd(
		RCC_AHB1Periph_GPIOA 	|
		RCC_AHB1Periph_GPIOB 	|
		RCC_AHB1Periph_GPIOC 	|
		RCC_AHB1Periph_GPIOD 	|
		RCC_AHB1Periph_GPIOE 	|
		RCC_AHB1Periph_GPIOH 	|
		RCC_AHB1Periph_CRC	 	|
		RCC_AHB1Periph_FLITF 	|
		RCC_AHB1Periph_SRAM1	|
		RCC_AHB1Periph_SRAM2	|
		RCC_AHB1Periph_BKPSRAM	|
		RCC_AHB1Periph_DMA1		|
		RCC_AHB1Periph_DMA2		|
		0, ENABLE
	);
	
	/* AHB2 clocks enable */
	RCC_AHB2PeriphClockCmd(0, ENABLE);
	
	/* APB1 clocks enable */
	RCC_APB1PeriphClockCmd(
		RCC_APB1Periph_PWR		|
//		RCC_APB1Periph_I2C3		|	
//		RCC_APB1Periph_I2C2		|	
//		RCC_APB1Periph_I2C1		|	
		RCC_APB1Periph_USART2	|	
		RCC_APB1Periph_SPI3		|	
		RCC_APB1Periph_SPI2		|	
		RCC_APB1Periph_WWDG		|	
		RCC_APB1Periph_TIM5		|	
		RCC_APB1Periph_TIM4		|	
		RCC_APB1Periph_TIM3		|	
		RCC_APB1Periph_TIM2		|	
		0, ENABLE
	);
	
	/* APB2 clocks enable */
	RCC_APB2PeriphClockCmd(
		RCC_APB2Periph_ADC1		|
		RCC_APB2Periph_SPI5		|
		RCC_APB2Periph_TIM11	|
		RCC_APB2Periph_TIM10	|
		RCC_APB2Periph_TIM9		|
		RCC_APB2Periph_TIM1		|
		RCC_APB2Periph_SYSCFG	|
		RCC_APB2Periph_SPI4		|
		RCC_APB2Periph_SPI1		|
		RCC_APB2Periph_SDIO		|		
		RCC_APB2Periph_USART6	|		
		RCC_APB2Periph_USART1	|
		0, ENABLE
	);
	
	/* Initialise GPIOA */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_PuPd		= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin			= GPIO_Pin_All;
//	GPIO_InitStructure.GPIO_Pin			&= ~(GPIO_Pin_11 | GPIO_Pin_12);		// leave USB D+/D- alone
	GPIO_InitStructure.GPIO_Pin			&= ~(GPIO_Pin_13 | GPIO_Pin_14);		// leave JTAG pins alone
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Initialise GPIOB */
	GPIO_InitStructure.GPIO_Pin			= GPIO_Pin_All;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Initialise GPIOC */
	GPIO_InitStructure.GPIO_Pin			= GPIO_Pin_All;
	GPIO_Init(GPIOC, &GPIO_InitStructure);			// Board ID, AUDIO MCLK, SCLK, SDIN, OTG_FS_PowerSwitchOn
	GPIO_Init(GPIOD, &GPIO_InitStructure);			// 4 USER LEDS, AUDIO, OTG_FS
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_Init(GPIOH, &GPIO_InitStructure);
}

void systemInit(void)
{
	/* Configure the system clock */
	SetSysClock();

	/* Configure NVIC preempt/priority groups */
	NVIC_PriorityGroupConfig(NVIC_PRIORITY_GROUPING);	// SCB->AIRCR, [10:8] = 101, 2 bits of preemption priority, 2 bits of subpriority (response priority)

	/* Clear the reset flags */
	RCC_ClearFlag();		// RCC->CSR |= RCC_CSR_RMVF;
	
	/* Enable AHB, APB1, APB2 Peripheral clocks and configure GPIOx, where x = A, B, C, D, E, H */
	EnableGPIOClk();
	
	/* Initialise SysTick counter */
	SysTick_Init();
	
	/* Configure SysTick in milliseconds time base */
#ifdef USE_RTOS    
	SysTick_Config(SystemCoreClock / configTICK_RATE_HZ);       // configTICK_RATE_HZ = 1000 defined in FreeRTOSConfig.h
#else
    SysTick_Config(SystemCoreClock / 1000);
#endif
}


//void gpioPA1Init(void)
//{
//	PA1_PIN = IOGetByTag(IO_TAG(GPIO_PA1_PIN));
//	IOInit(PA1_PIN, OWNER_SYSTEM, 0);
//	IOConfigGPIO(PA1_PIN, IOCFG_OUT_PP);
//}

//void gpioPA1Ops(void)
//{
//	IOWrite(PA1_PIN, false);
//	delay(800);
//	IOWrite(PA1_PIN, true);
//	delay(800);	
//}

//void gpioPB8Init(void)
//{
//	PB8_PIN = IOGetByTag(IO_TAG(GPIO_PB8_PIN));
//	IOInit(PB8_PIN, OWNER_SYSTEM, 0);
//	IOConfigGPIO(PB8_PIN, IOCFG_OUT_PP);
//}

//void gpioPB8Ops(void)
//{
//	IOWrite(PB8_PIN, false);
//	delay(400);
//	IOWrite(PB8_PIN, true);
//	delay(400);	
//}

#if 0	// so far so good
	__g_ToDo = __basepriSetMemRetVal(0x10);		// basepri can be set to 0x10 only
	__g_basepri_save = __get_BASEPRI();			// __get_BASEPRI() returns 0x10
	
	__set_BASEPRI(0x00);		// reset basepri to 0x00
	__g_basepri_save2 = __get_BASEPRI();		// __get_BASEPRI() returns 0x00

	__g_ToDo2 = __basepriSetMemRetVal(0x10);		// basepri can be set to 0x10 only
	__g_basepri_save3 = __get_BASEPRI();			// __get_BASEPRI() returns 0x10

	basepri_val = 0x10;
	__basepriRestoreMem(&basepri_val);

	basepri_val = 0x00;
	__basepriRestoreMem(&basepri_val);
#endif

#if 0
	{
//		uint8_t __basepri_save = __get_BASEPRI();
		uint8_t __basepri_save __attribute__ ((__cleanup__(__basepriRestoreMem))) = __get_BASEPRI();
		__g_basepri_save = __basepri_save;
		uint8_t __ToDo = __basepriSetMemRetVal(0x10);
		__g_basepri_save2 = __get_BASEPRI();
		forCnt++;
	}
#endif
	
#if 0
	for ( uint8_t __basepri_save __attribute__ ((__cleanup__(__basepriRestoreMem))) = __get_BASEPRI(), __ToDo = __basepriSetMemRetVal(0x10); __ToDo ; __ToDo = 0 ) {
//	for ( uint8_t __ToDo = __basepriSetMemRetVal(0x10); __ToDo ; __ToDo = 0 ) {
		__g_basepri_save2 = __get_BASEPRI();
		//__set_BASEPRI(0x00);
	}
	//__set_BASEPRI(0x00);
	__g_basepri_save3 = __get_BASEPRI();	// WTF?! __g_basepri_save2 should be 0x00, but it has 0x10??!!
#endif

#if 0
void clean_up(int *final_value)
{
	g_val = *final_value;
}
#endif
	
#if 0
	{
		int avar __attribute__ ((__cleanup__(clean_up))) = 1;
	}
#endif
