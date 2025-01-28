/*
 * Final Project - Self-Balancing Cube Header File
 *
 *  Created on: Oct 22, 2024
 *      Author: yjia67
 *
 */

DL_I2C_ClockConfig gI2C_0ClockConfig = {
                                      .clockSel = DL_I2C_CLOCK_BUSCLK, // 32 MHz
                                      .divideRatio = DL_I2C_CLOCK_DIVIDE_1
};

DL_TimerG_ClockConfig gTIMER_0ClockConfig = {
                                            .clockSel = DL_TIMER_CLOCK_LFCLK, // 32 KHz
                                            .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
                                            .prescale = 0 // 32 KHz
};

DL_TimerG_TimerConfig gTIMER_0TimerConfig = {
                                             .period = 160, // 200 Hz
                                             .timerMode = DL_TIMER_TIMER_MODE_PERIODIC,
                                             .startTimer = DL_TIMER_STOP
};

DL_TimerA_ClockConfig gPWM_1ClockConfig = {
                                           .clockSel = DL_TIMER_CLOCK_BUSCLK, // 32 MHz
                                           .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
                                           .prescale = 0 // 32 MHz
};

DL_TimerA_PWMConfig gPWM_1Config = {
                                    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
                                    .period = 65535,
                                    .startTimer = DL_TIMER_STOP
};

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

// for storing the raw data read from the sensor; using signed integer to handle 2's complement
volatile int16_t aX = 0, aY = 0, aZ = 0;
volatile int16_t aX_offset = 0, aY_offset = 0, aZ_offset = 0;
volatile int16_t aXc = 0, aYc = 0, aZc = 0; // corrected & filtered values

volatile int16_t gX = 0, gY = 0, gZ = 0;
volatile int16_t gX_offset = 0, gY_offset = 0, gZ_offset = 0;  // calibration values
volatile int16_t gXc = 0, gYc = 0, gZc = 0; // corrected & filtered values

// angles calculated from integrating gyro reading in degrees
volatile float theta_gX = 0.0, theta_gY = 0.0;
volatile float theta_gX_wrong = 0.0;

// angles calculated from accelerometer readings
volatile float theta_aX = 0.0, theta_aY = 0.0, theta_aX2 = 0.0;

// angles calculated from fusing accel and gyro readings
volatile float theta_X_target;
volatile float theta_X = 0.0;
volatile float integral_error = 0.0;

// calibration stuff
volatile uint8_t calibrating = 0;
volatile uint8_t calibrated = 0;
volatile const uint16_t num_cal_data = 500;
volatile uint8_t balancing = 0;

// switch polling flags
volatile uint8_t S1_pressed = 0;
volatile uint8_t S2_pressed = 0;

// motor control tuning
volatile const uint16_t pwm_period = 65535;
volatile uint16_t min_pwm = 65535 * 0.05; // 5% duty cycle -> ~ 120 RPM
volatile uint16_t max_pwm = 65535 * 50.0;
volatile int t_count = 0;

volatile float max_duty_cycle = 50.0;
volatile float min_duty_cycle = 0.0;
volatile float duty_cycle = 0.0;
volatile int pwm_x = 0;

// gains
float kp = -150.0;
float ki = 0.0;
float kd = -10.5;
float ks = -0.957;

int motor_speed_x = 0;

// other params
volatile const float dt = 0.005; // time step per control loop (200 Hz)

/*
 * DEBUG ONLY
 */


// store and print some data just to see
volatile const uint16_t num_data = 500;
volatile uint16_t count = 0;

// raw accel and gyro data
volatile int16_t accel_data[1] = {0};
volatile int16_t gyro_data[1] = {0};
volatile float theta_X_data[1] = {0};
volatile int motor_speed_X_data[1] = {0};
volatile int pwm_X_data [1] = {0};

volatile uint8_t print_data_flag = 0;

// MPU6050 settings
const uint8_t accelFS = 0; // 0: 2g, 1: 4g, 2: 8g, 3: 16g
const int32_t accelSensitivity = 16384; // sensitivity of the accelerometer: LSB/g
const uint8_t gyroFS = 0; // 0: 250deg/s, 1: 500deg/s, 2: 1000deg/s, 3: 2000 deg/s
const int16_t gyroSensitivity = 131; // LSB/deg/s

// helper functions

void calibrate_gyro(void);
void calibrate_accel(void);
void check_switches(void);

void read_sensor_data(void);
void read_gyro_data(void);
void read_accel_data(void);

void estimate_attitude(void);
void pid(void);
void drive_motor(int32_t pwm);

void test_drive(void);

void monitor_raw_data(void);
void monitor_attitude_data(void);
void monitor_control_data(void);

void store_raw_data(void);
void store_euler_data(void);

void print_raw_data(void);
void print_converted_data(void);
void print_attitude_data(void);
void print_control_data(void);

// parameters that might need to be tuned
// signal conditioning
volatile const float alpha = 0.7; // smoothing factor for gyro reading and accel
volatile const float accel_gain = 0.003; // weight for the accelerometer measurement in complementary filter


// MPU6050 information
#define MPU6050         0x68 // default I2C address for MPU6050
#define CONFIG          0x1A // config digital low pass filter setting
#define ACCEL_CONFIG    0x1C // config address for accelerometer
#define GYRO_CONFIG     0x1B // config address for gyroscope
#define PWR_MGMT_1      0x6B // config power mode and clock source
#define ACCEL_OUT       0x3B // address for accel measurements: XOUT, YOUT, ZOUT (16-bit 2's complement)
#define GYRO_OUT        0x43 // address for gyro measurements: XOUT, YOUT, ZOUT (16-bit 2's complement)
#define WHO_AM_I        0x75 // address for verifying device address

// I2C stuff
#define I2C_SCL_IOMUX  IOMUX_PINCM15
#define I2C_SCL_PORT   GPIOB
#define I2C_SCL_PIN    DL_GPIO_PIN_2
#define I2C_SDA_IOMUX  IOMUX_PINCM16
#define I2C_SDA_PORT   GPIOB
#define I2C_SDA_PIN    DL_GPIO_PIN_3

// Motor stuff
#define BRAKE_IOMUX IOMUX_PINCM55
#define BRAKE_PORT  GPIOA
#define BRAKE_PIN   DL_GPIO_PIN_25

#define DIR1_IOMUX  IOMUX_PINCM54
#define DIR1_PORT   GPIOA
#define DIR1_PIN    DL_GPIO_PIN_24

// 3 PWM using TIMA
// TIMA0_C0 [5]
#define PWM1_IOMUX  IOMUX_PINCM19
#define PWM1_PORT   GPIOA
#define PWM1_PIN    DL_GPIO_PIN_8

// reserve 6 capture timers in case i need to use encoders
// TIMG12_C0 [6]
#define ENC1_1_IOMUX    IOMUX_PINCM21
#define ENC1_1_PORT     GPIOA
#define ENC1_1_PIN      DL_GPIO_PIN_10
// TIMG12_C1 [5]
#define ENC1_2_IOMUX    IOMUX_PINCM52
#define ENC1_2_PORT     GPIOB
#define ENC1_2_PIN      DL_GPIO_PIN_24
// TIMG6_C0 [7]
#define ENC2_1_IOMUX    IOMUX_PINCM23
#define ENC2_1_PORT     GPIOB
#define ENC2_1_PIN      DL_GPIO_PIN_6
// TIMG6_C1 [7]
#define ENC2_2_IOMUX    IOMUX_PINCM24
#define ENC2_2_PORT     GPIOB
#define ENC2_2_PIN      DL_GPIO_PIN_7
// TIMG8_C0 [5]
#define ENC3_1_IOMUX    IOMUX_PINCM32
#define ENC3_1_PORT     GPIOB
#define ENC3_1_PIN      DL_GPIO_PIN_15
// TIMG8_C1 [5]
#define ENC3_2_IOMUX    IOMUX_PINCM33
#define ENC3_2_PORT     GPIOB
#define ENC3_2_PIN      DL_GPIO_PIN_16

// LEDs
#define LED1    GPIOA, DL_GPIO_PIN_0
#define LED2_R  GPIOB, DL_GPIO_PIN_26
#define LED2_G  GPIOB, DL_GPIO_PIN_27
#define LED2_B  GPIOB, DL_GPIO_PIN_22
#define S1      GPIOA, DL_GPIO_PIN_18
#define S2      GPIOB, DL_GPIO_PIN_21


/*
 * INITALIZATION STUFF
 */

// initialization functions
void GPIO_init(void);
void UART_init(void);
void TimerG_init(void);
void TimerA_PWM_init(void);
void I2C_init(void);
void i2c_write(uint8_t device_address, uint8_t register_address, uint8_t* tx_buffer, int n_tx_bytes);
void i2c_read(uint8_t device_address, uint8_t register_address, uint8_t* rx_buffer, int n_rx_bytes);
void mpu6050_init(void);
void motor_init(void);

// initialize pins for driving the motor
void motor_init(void)
{
    DL_GPIO_initDigitalOutput(DIR1_IOMUX);
    DL_GPIO_clearPins(DIR1_PORT, DIR1_PIN);
    DL_GPIO_enableOutput(DIR1_PORT, DIR1_PIN);

    DL_GPIO_initDigitalOutput(BRAKE_IOMUX);
    DL_GPIO_clearPins(BRAKE_PORT, BRAKE_PIN);
    DL_GPIO_enableOutput(BRAKE_PORT, BRAKE_PIN);
}

// configure MPU6050
void mpu6050_init(void)
{
    // use 8MHz internal oscillator
    i2c_write(MPU6050, PWR_MGMT_1, 0, 1);

    // bits 3-4 for selecting full scale range for accel and gyro
    uint8_t accel_config_val = accelFS << 3;
    uint8_t gyro_config_val = gyroFS << 3;
    i2c_write(MPU6050, ACCEL_CONFIG, &accel_config_val, 1);
    i2c_write(MPU6050, GYRO_CONFIG, &gyro_config_val, 1);

    // configure DLPF (digital low pass filter) to 1: accel = 184 Hz, gyro = 188 Hz
    uint8_t DLPF = 0;
    i2c_write(MPU6050, CONFIG, &DLPF, 1);
}

// write to a register on the target device
void i2c_write(uint8_t device_address, uint8_t register_address, uint8_t* tx_buffer, int n_tx_bytes)
{
    // use a temporary buffer to prepend the register address
    uint8_t temp_buffer[n_tx_bytes + 1];
    temp_buffer[0] = register_address;

    // add the data
    for (uint8_t i = 0; i < n_tx_bytes; i++) {
        temp_buffer[i + 1] = tx_buffer[i];
    }

    DL_I2C_fillControllerTXFIFO(I2C1, temp_buffer, n_tx_bytes + 1);

    // wait until the controller becomes idle
    while (! (DL_I2C_getControllerStatus(I2C1) & DL_I2C_CONTROLLER_STATUS_IDLE) );
    DL_I2C_startControllerTransfer(I2C1, device_address, DL_I2C_CONTROLLER_DIRECTION_TX, n_tx_bytes + 1);
    while (DL_I2C_getControllerStatus(I2C1) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
}

// read from a register on the target device
void i2c_read(uint8_t device_address, uint8_t register_address, uint8_t* rx_buffer, int n_rx_bytes)
{
    // write the register address on the target device i want to read from to the I2C bus
    DL_I2C_fillControllerTXFIFO(I2C1, &register_address, 1);

    // wait until the controller becomes idle
    while (! (DL_I2C_getControllerStatus(I2C1) & DL_I2C_CONTROLLER_STATUS_IDLE) );
    DL_I2C_startControllerTransfer(I2C1, device_address, DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    // wait until transfer is complete
    while (DL_I2C_getControllerStatus(I2C1) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    while (! (DL_I2C_getControllerStatus(I2C1) & DL_I2C_CONTROLLER_STATUS_IDLE) );

    delay_cycles(1000);

    // send a read request to target
    DL_I2C_startControllerTransfer(I2C1, device_address, DL_I2C_CONTROLLER_DIRECTION_RX, n_rx_bytes);
    for (uint8_t i = 0; i < n_rx_bytes; i++) {
        while (DL_I2C_isControllerRXFIFOEmpty(I2C1));
        rx_buffer[i] = DL_I2C_receiveControllerData(I2C1);
    }
}

// initialize I2C
void I2C_init(void)
{
    DL_I2C_reset(I2C1);
    DL_I2C_enablePower(I2C1);

    // configure SDA pin
    DL_GPIO_initPeripheralInputFunctionFeatures(I2C_SDA_IOMUX,
                                                4,
                                                DL_GPIO_INVERSION_DISABLE,
                                                DL_GPIO_RESISTOR_NONE,
                                                DL_GPIO_HYSTERESIS_DISABLE,
                                                DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(I2C_SDA_IOMUX);

    // configure SCL pin
    DL_GPIO_initPeripheralInputFunctionFeatures(I2C_SCL_IOMUX,
                                                4,
                                                DL_GPIO_INVERSION_DISABLE,
                                                DL_GPIO_RESISTOR_NONE,
                                                DL_GPIO_HYSTERESIS_DISABLE,
                                                DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(I2C_SCL_IOMUX);

    DL_I2C_setClockConfig(I2C1, &gI2C_0ClockConfig);
    DL_I2C_disableAnalogGlitchFilter(I2C1);
    DL_I2C_resetControllerTransfer(I2C1);
    DL_I2C_setTimerPeriod(I2C1, 7); // 400 kHz
    DL_I2C_setControllerTXFIFOThreshold(I2C1, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(I2C1, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(I2C1);
    DL_I2C_enableController(I2C1);
}

// initialize PWM on timerA
void TimerA_PWM_init(void)
{
    DL_GPIO_initPeripheralOutputFunction(PWM1_IOMUX, 5);
    DL_GPIO_enableOutput(PWM1_PORT, PWM1_PIN);
    DL_TimerA_enablePower(TIMA0);
    DL_TimerA_setClockConfig(TIMA0, &gPWM_1ClockConfig);
    DL_TimerA_initPWMMode(TIMA0, &gPWM_1Config);

    DL_TimerA_setCaptureCompareOutCtl(TIMA0,
                                      DL_TIMER_CC_OCTL_INIT_VAL_LOW,
                                      DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
                                      DL_TIMER_CC_OCTL_SRC_FUNCVAL,
                                      DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareValue(TIMA0, 0, DL_TIMER_CC_0_INDEX);
    DL_TimerA_enableClock(TIMA0);
    DL_TimerA_setCCPDirection(TIMA0, DL_TIMER_CC0_OUTPUT);
    DL_TimerA_startCounter(TIMA0);
}

// initialize timer for control loop counting
void TimerG_init(void)
{
    DL_TimerG_enablePower(TIMG0);
    DL_TimerG_setClockConfig(TIMG0, &gTIMER_0ClockConfig);
    DL_TimerG_initTimerMode(TIMG0, &gTIMER_0TimerConfig);
    DL_TimerG_enableInterrupt(TIMG0, DL_TIMER_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(TIMG0);
}

// initialize UART
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

