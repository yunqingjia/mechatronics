/*
 * Lab 10 for AE6705 Header File
 *
 *  Created on: Dec 7, 2024
 *      Author: yjia67
 *
 */

void GPIO_init(void);
void TimerG_PWM_init(void);
void TimerG6_CC_init(void);
void TimerG_init(void);
void UART_init(void);

void check_switches(void);
float calc_rpm(void);
void drive_motor(float duty_cycle);
void print_data(void);
void pid(void);

volatile uint32_t pwm_period = 512000; // PWM setting


/*
 * timer configs
 */

// TimerG0 for running PID data at 100Hz
DL_TimerG_ClockConfig gTIMER_0ClockConfig = {
                                            .clockSel = DL_TIMER_CLOCK_LFCLK, // 32 KHz
                                            .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
                                            .prescale = 0 // 32 KHz
};

DL_TimerG_TimerConfig gTIMER_0TimerConfig = {
                                             .period = 320, // 100 Hz
                                             .timerMode = DL_TIMER_TIMER_MODE_PERIODIC,
                                             .startTimer = DL_TIMER_STOP
};

// TimerG6 for CC
DL_TimerG_ClockConfig gCAPTURE_0ClockConfig = {
                                          .clockSel = DL_TIMER_CLOCK_BUSCLK, // 32 MHz
                                          .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
                                          .prescale = 0 // 4 MHz
};


DL_TimerG_CaptureConfig gCAPTURE_0CaptureConfig = {
                                                 .captureMode = DL_TIMER_CAPTURE_MODE_EDGE_TIME,
                                                 .period = 65535, // ~ 2 ms per period
                                                 .startTimer = DL_TIMER_STOP,
                                                 .edgeCaptMode = DL_TIMER_CAPTURE_EDGE_DETECTION_MODE_FALLING,
                                                 .inputChan = DL_TIMER_INPUT_CHAN_0,
                                                 .inputInvMode = DL_TIMER_CC_INPUT_INV_NOINVERT
};

// TimerG12 for PWM
DL_TimerG_ClockConfig gPWM_0ClockConfig = {
                                           .clockSel = DL_TIMER_CLOCK_BUSCLK, // 32 MHz
                                           .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
                                           .prescale = 0 // 32 MHz
};

DL_TimerG_PWMConfig gPWM_0Config = {
                                    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
                                    .period = 512000, // 1/32e6 * 512000 = 16 ms per period
                                    .startTimer = DL_TIMER_STOP
};

// UART stuff
DL_UART_ClockConfig gUART_0ClockConfig = {
                                               .clockSel = DL_UART_CLOCK_BUSCLK,
                                               .divideRatio = DL_UART_CLOCK_DIVIDE_RATIO_1
};

DL_UART_Config gUART_0Config = {
                                .mode = DL_UART_MODE_NORMAL,
                                .direction = DL_UART_DIRECTION_TX_RX,
                                .flowControl = DL_UART_FLOW_CONTROL_NONE,
                                .parity = DL_UART_PARITY_NONE,
                                .wordLength = DL_UART_WORD_LENGTH_8_BITS,
                                .stopBits = DL_UART_STOP_BITS_ONE
};

/*
 * pin names
 */

// for capturing encoder
#define timg6_iomux IOMUX_PINCM15
#define timg6_port  GPIOB
#define timg6_pin   2

// for driving motor
#define pwm1_iomux  IOMUX_PINCM30
#define pwm1_port   GPIOB
#define pwm1_pin    13

// LEDs
#define LED1    GPIOA, DL_GPIO_PIN_0
#define LED2_R  GPIOB, DL_GPIO_PIN_26
#define LED2_G  GPIOB, DL_GPIO_PIN_27
#define LED2_B  GPIOB, DL_GPIO_PIN_22
#define S1      GPIOA, DL_GPIO_PIN_18
#define S2      GPIOB, DL_GPIO_PIN_21

/*
 * initializing peripherals
 */

// initialize / configure timer for Capture mode
void TimerG6_CC_init(void)
{
    DL_GPIO_initPeripheralInputFunction(timg6_iomux, 7);
    DL_TimerG_enablePower(TIMG6);
    DL_TimerG_setClockConfig(TIMG6, &gCAPTURE_0ClockConfig);
    DL_TimerG_initCaptureMode(TIMG6, &gCAPTURE_0CaptureConfig);
    DL_TimerG_enableInterrupt(TIMG6, DL_TIMERG_INTERRUPT_CC0_DN_EVENT);
    DL_TimerG_enableClock(TIMG6);
    NVIC_ClearPendingIRQ(TIMG6_INT_IRQn);
    NVIC_EnableIRQ(TIMG6_INT_IRQn);
    DL_TimerG_startCounter(TIMG6);
}

// initialize / configure timer for PWM
void TimerG_PWM_init(void)
{
    DL_GPIO_initPeripheralOutputFunction(pwm1_iomux, 4);
    DL_GPIO_enableOutput(pwm1_port, pwm1_pin);

    DL_TimerG_enablePower(TIMG12);
    DL_TimerG_setClockConfig(TIMG12, &gPWM_0ClockConfig);
    DL_TimerG_initPWMMode(TIMG12, &gPWM_0Config);

    DL_TimerG_setCaptureCompareOutCtl(TIMG12,
                                      DL_TIMER_CC_OCTL_INIT_VAL_LOW,
                                      DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
                                      DL_TIMER_CC_OCTL_SRC_FUNCVAL,
                                      DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareValue(TIMG12, pwm_period, DL_TIMER_CC_0_INDEX);

    DL_TimerG_enableClock(TIMG12);
    DL_TimerG_setCCPDirection(TIMG12, DL_TIMER_CC0_OUTPUT);
    DL_TimerG_startCounter(TIMG12);
}

// initialize timer for control loop counting
void TimerG_init(void)
{
    DL_TimerG_enablePower(TIMG0);
    DL_TimerG_setClockConfig(TIMG0, &gTIMER_0ClockConfig);
    DL_TimerG_initTimerMode(TIMG0, &gTIMER_0TimerConfig);
    DL_TimerG_enableClock(TIMG0);
}

// UART initialization
void UART_init(void)
{
    // enable peripheral functions
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM21, IOMUX_PINCM21_PF_UART0_TX);
    DL_UART_reset(UART0);
    DL_UART_enablePower(UART0);
    DL_UART_setClockConfig(UART0, &gUART_0ClockConfig);
    DL_UART_init(UART0, &gUART_0Config);
    DL_UART_setOversampling(UART0, DL_UART_OVERSAMPLING_RATE_16X);

    // set baud rate to 115200
    int clock_rate = 32e6;
    int baud_rate = 115200;
    int oversample_freq = 16;
    double temp = (double) clock_rate / (oversample_freq * baud_rate);
    int IBRD = (int)temp;
    int FBRD = (int)((temp-IBRD) * 64 + 0.5 );
    DL_UART_setBaudRateDivisor(UART0, IBRD, FBRD);

    DL_UART_enable(UART0);
}

// configure LEDs
void GPIO_init(void)
{
    // LEDs
    DL_GPIO_initDigitalOutput(IOMUX_PINCM1);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM57);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM58);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM50);

    DL_GPIO_setPins(LED1);
    DL_GPIO_clearPins(LED2_R);
    DL_GPIO_clearPins(LED2_G);
    DL_GPIO_clearPins(LED2_B);

    DL_GPIO_enableOutput(LED1);
    DL_GPIO_enableOutput(LED2_R);
    DL_GPIO_enableOutput(LED2_G);
    DL_GPIO_enableOutput(LED2_B);

    // switches: S1 - PD resistor -> logic high = ON; S2 - PU resistor -> logic low = ON
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM40, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
                                     DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM49, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
                                     DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}
