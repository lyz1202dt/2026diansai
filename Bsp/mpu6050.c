#include "mpu6050.h"

#include "Driver/mpu6050/inv_mpu.h"
#include "Driver/mpu6050/inv_mpu_dmp_motion_driver.h"
#include <semphr.h>
#include <string.h>
#include <task.h>

#define I2C_TIMEOUT_MS     (10)
#define mspm0_delay_ms     vTaskDelay
#define mspm0_get_clock_ms SysGetTick

static SemaphoreHandle_t k_mpu6050_i2c_semphr;
static uint8_t const *g_mpu6050_i2c_tx_ptr;
static uint32_t g_mpu6050_i2c_tx_cnt;
static uint8_t *g_mpu6050_i2c_rx_ptr;
static uint32_t g_mpu6050_i2c_rx_len;
static uint32_t g_mpu6050_i2c_rx_idx;
static volatile int g_mpu6050_i2c_status;

static void mpu6050_i2c_init_semphr(void)
{
    if (k_mpu6050_i2c_semphr == NULL)
    {
        k_mpu6050_i2c_semphr = xSemaphoreCreateBinary();
    }

    if (k_mpu6050_i2c_semphr != NULL)
    {
        xSemaphoreTake(k_mpu6050_i2c_semphr, 0);
    }
}

static void mpu6050_i2c_tx_fifo_fill(void)
{
    if (g_mpu6050_i2c_tx_cnt == 0) return;

    unsigned fillcnt = DL_I2C_fillControllerTXFIFO(MPU6050_I2C_INST,
                                                   g_mpu6050_i2c_tx_ptr,
                                                   g_mpu6050_i2c_tx_cnt);
    g_mpu6050_i2c_tx_ptr += fillcnt;
    g_mpu6050_i2c_tx_cnt -= fillcnt;
}

static void mpu6050_i2c_rx_fifo_drain(void)
{
    while (!DL_I2C_isControllerRXFIFOEmpty(MPU6050_I2C_INST))
    {
        uint8_t c;
        c = DL_I2C_receiveControllerData(MPU6050_I2C_INST);
        if (g_mpu6050_i2c_rx_idx < g_mpu6050_i2c_rx_len)
        {
            g_mpu6050_i2c_rx_ptr[g_mpu6050_i2c_rx_idx] = c;
            ++g_mpu6050_i2c_rx_idx;
        }
    }
}

static int mpu6050_i2c_wait_idle(void)
{
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(I2C_TIMEOUT_MS);

    do
    {
        if (DL_I2C_getControllerStatus(MPU6050_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)
        {
            return 0;
        }
        vTaskDelay(1);
    } while ((xTaskGetTickCount() - start_tick) < timeout_ticks);

    mpu6050_i2c_sda_unlock();
    return -1;
}

static int mspm0_i2c_disable(void)
{
    DL_I2C_reset(MPU6050_I2C_INST);
    DL_GPIO_initDigitalOutput(GPIO_MPU6050_I2C_IOMUX_SCL);
    DL_GPIO_initDigitalInputFeatures(GPIO_MPU6050_I2C_IOMUX_SDA,
                                     DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_NONE,
                                     DL_GPIO_HYSTERESIS_DISABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_clearPins(GPIO_MPU6050_I2C_SCL_PORT, GPIO_MPU6050_I2C_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_MPU6050_I2C_SCL_PORT, GPIO_MPU6050_I2C_SCL_PIN);
    return 0;
}

static int mspm0_i2c_enable(void)
{
    DL_I2C_reset(MPU6050_I2C_INST);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_MPU6050_I2C_IOMUX_SDA,
                                                GPIO_MPU6050_I2C_IOMUX_SDA_FUNC,
                                                DL_GPIO_INVERSION_DISABLE,
                                                DL_GPIO_RESISTOR_NONE,
                                                DL_GPIO_HYSTERESIS_DISABLE,
                                                DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_MPU6050_I2C_IOMUX_SCL,
                                                GPIO_MPU6050_I2C_IOMUX_SCL_FUNC,
                                                DL_GPIO_INVERSION_DISABLE,
                                                DL_GPIO_RESISTOR_NONE,
                                                DL_GPIO_HYSTERESIS_DISABLE,
                                                DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_MPU6050_I2C_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_MPU6050_I2C_IOMUX_SCL);
    DL_I2C_enablePower(MPU6050_I2C_INST);
    SYSCFG_DL_MPU6050_I2C_init();
    return 0;
}

void mpu6050_i2c_sda_unlock(void)
{
    uint8_t cycleCnt = 0;
    mspm0_i2c_disable();
    do
    {
        DL_GPIO_clearPins(GPIO_MPU6050_I2C_SCL_PORT, GPIO_MPU6050_I2C_SCL_PIN);
        mspm0_delay_ms(1);
        DL_GPIO_setPins(GPIO_MPU6050_I2C_SCL_PORT, GPIO_MPU6050_I2C_SCL_PIN);
        mspm0_delay_ms(1);

        if (DL_GPIO_readPins(GPIO_MPU6050_I2C_SDA_PORT, GPIO_MPU6050_I2C_SDA_PIN)) break;
    } while (++cycleCnt < 100);
    mspm0_i2c_enable();
}

int mspm0_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data)
{
    if (!length) return 0;
    if (k_mpu6050_i2c_semphr == NULL) return -1;

    xSemaphoreTake(k_mpu6050_i2c_semphr, 0);
    g_mpu6050_i2c_status = 0;
    g_mpu6050_i2c_tx_ptr = data;
    g_mpu6050_i2c_tx_cnt = length;

    DL_I2C_transmitControllerData(MPU6050_I2C_INST, reg_addr);
    DL_I2C_clearInterruptStatus(MPU6050_I2C_INST,
                                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
                                DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER |
                                DL_I2C_INTERRUPT_CONTROLLER_NACK |
                                DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);

    if (mpu6050_i2c_wait_idle() != 0) return -1;

    mpu6050_i2c_tx_fifo_fill();
    DL_I2C_startControllerTransfer(MPU6050_I2C_INST, slave_addr, DL_I2C_CONTROLLER_DIRECTION_TX, length + 1);

    if (xSemaphoreTake(k_mpu6050_i2c_semphr, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdFALSE)
    {
        mpu6050_i2c_sda_unlock();
        return -1;
    }

    return g_mpu6050_i2c_status;
}

int mspm0_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
    if (!length) return 0;
    if (k_mpu6050_i2c_semphr == NULL) return -1;

    xSemaphoreTake(k_mpu6050_i2c_semphr, 0);
    g_mpu6050_i2c_status = 0;
    g_mpu6050_i2c_rx_ptr = data;
    g_mpu6050_i2c_rx_len = length;
    g_mpu6050_i2c_rx_idx = 0;

    DL_I2C_transmitControllerData(MPU6050_I2C_INST, reg_addr);
    MPU6050_I2C_INST->MASTER.MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE;
    DL_I2C_clearInterruptStatus(MPU6050_I2C_INST,
                                DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
                                DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER |
                                DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_FULL |
                                DL_I2C_INTERRUPT_CONTROLLER_NACK |
                                DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);

    if (mpu6050_i2c_wait_idle() != 0) return -1;

    DL_I2C_startControllerTransfer(MPU6050_I2C_INST, slave_addr, DL_I2C_CONTROLLER_DIRECTION_RX, length);

    if (xSemaphoreTake(k_mpu6050_i2c_semphr, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdFALSE)
    {
        MPU6050_I2C_INST->MASTER.MCTR = 0;
        DL_I2C_flushControllerTXFIFO(MPU6050_I2C_INST);
        mpu6050_i2c_sda_unlock();
        return -1;
    }

    mpu6050_i2c_rx_fifo_drain();
    MPU6050_I2C_INST->MASTER.MCTR = 0;
    DL_I2C_flushControllerTXFIFO(MPU6050_I2C_INST);

    if ((g_mpu6050_i2c_status == 0) && (g_mpu6050_i2c_rx_idx == length)) return 0;
    else return -1;
}

void MPU6050_I2C_INST_IRQHandler(void)
{
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

    switch (DL_I2C_getPendingInterrupt(MPU6050_I2C_INST))
    {
        case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
        case DL_I2C_IIDX_CONTROLLER_TXFIFO_EMPTY: {
            mpu6050_i2c_tx_fifo_fill();
            break;
        }
        case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
        case DL_I2C_IIDX_CONTROLLER_RXFIFO_FULL: {
            mpu6050_i2c_rx_fifo_drain();
            break;
        }
        case DL_I2C_IIDX_CONTROLLER_TX_DONE: {
            if (k_mpu6050_i2c_semphr != NULL)
            {
                xSemaphoreGiveFromISR(k_mpu6050_i2c_semphr, &pxHigherPriorityTaskWoken);
            }
            break;
        }
        case DL_I2C_IIDX_CONTROLLER_RX_DONE: {
            mpu6050_i2c_rx_fifo_drain();
            if (k_mpu6050_i2c_semphr != NULL)
            {
                xSemaphoreGiveFromISR(k_mpu6050_i2c_semphr, &pxHigherPriorityTaskWoken);
            }
            break;
        }
        case DL_I2C_IIDX_CONTROLLER_NACK:
        case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST: {
            g_mpu6050_i2c_status = -1;
            if (k_mpu6050_i2c_semphr != NULL)
            {
                xSemaphoreGiveFromISR(k_mpu6050_i2c_semphr, &pxHigherPriorityTaskWoken);
            }
            break;
        }
        default:
            break;
    }

    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

/* Data requested by client. */
#define PRINT_ACCEL (0x01)
#define PRINT_GYRO  (0x02)
#define PRINT_QUAT  (0x04)

#define ACCEL_ON    (0x01)
#define GYRO_ON     (0x02)

#define MOTION      (0)
#define NO_MOTION   (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (50)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void *)0x1800)

struct rx_s
{
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s
{
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};
static struct hal_s hal = {0};

unsigned long sensor_timestamp;
short Data_Gyro[3], Data_Accel[3], sensors;
unsigned char more;
long quat[4];

#define q30 (1073741824.0f) /* 2^30 = 1073741824 */
//float Data_Pitch, Data_Roll, Data_Yaw;

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0, 0, -1, 0, 0, 0, 1};

static void tap_cb(unsigned char direction, unsigned char count)
{
}

static void android_orient_cb(unsigned char orientation)
{
}

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0) b = 0;
    else if (row[0] < 0) b = 4;
    else if (row[1] > 0) b = 1;
    else if (row[1] < 0) b = 5;
    else if (row[2] > 0) b = 2;
    else if (row[2] < 0) b = 6;
    else b = 7; // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

void MPU6050_Init(void)
{
    int result;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;

    mpu6050_i2c_init_semphr();
    NVIC_EnableIRQ(MPU6050_I2C_INST_INT_IRQN);

    if (DL_I2C_getSDAStatus(MPU6050_I2C_INST) == DL_I2C_CONTROLLER_SDA_LOW) mpu6050_i2c_sda_unlock();

    result = mpu_init();
#ifdef DEBUG
    if (result) return;
#endif

#ifndef DEBUG
    if (result) DL_SYSCTL_resetDevice(DL_SYSCTL_RESET_POR);//软件复位 调试模式调用会锁芯片！！
#endif

    result = 0;
    /* Get/set hardware configuration. Start Data_Gyro. */
    /* Wake up all sensors. */
    result += mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both Data_Gyro and Data_Accel data into the FIFO. */
    result += mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    result += mpu_set_sample_rate(DEFAULT_MPU_HZ);
    /* Read back configuration in case it was set improperly. */
    result += mpu_get_sample_rate(&gyro_rate);
    result += mpu_get_gyro_fsr(&gyro_fsr);
    result += mpu_get_accel_fsr(&accel_fsr);

    /* Initialize HAL state variables. */
    memset(&hal, 0, sizeof(hal));
    hal.sensors = ACCEL_ON | GYRO_ON;
    hal.report = PRINT_QUAT;

    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the Data_Gyro and Data_Accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a Data_Gyro-only quaternion on the DMP at
     * 200Hz. Integrating the Data_Gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a Data_Gyro/Data_Accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the Data_Gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw Data_Gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated Data_Gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    result += dmp_load_motion_driver_firmware();
    result += dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    result += dmp_register_tap_cb(tap_cb);
    result += dmp_register_android_orient_cb(android_orient_cb);
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT |
                       DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL;
    result += dmp_enable_feature(hal.dmp_features);
    result += dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    result += mpu_set_dmp_state(1);
    hal.dmp_on = 1;

    if (result) return;

}

int read_quad(float *q)
{
    /* This function gets new data from the FIFO when the DMP is in
    * use. The FIFO can contain any combination of Data_Gyro, Data_Accel,
    * quaternion, and gesture data. The sensors parameter tells the
    * caller which data fields were actually populated with new data.
    * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
    * the FIFO isn't being filled with Data_Accel data.
    * The driver parses the gesture data to determine if a gesture
    * event has occurred; on an event, the application will be notified
    * via a callback (assuming that a callback function was properly
    * registered). The more parameter is non-zero if there are
    * leftover packets in the FIFO.
    */

    int result;
    do
    {
        result = dmp_read_fifo(Data_Gyro, Data_Accel, quat, &sensor_timestamp, &sensors, &more);
    } while (more);
    if (result) return -1;
    q[0] = quat[0] / q30;
    q[1] = quat[1] / q30;
    q[2] = quat[2] / q30;
    q[3] = quat[3] / q30;
    return 0;
}

void get_euler_angles(const float *q,float *roll,float *pitch,float *yaw)
{
    float q0=q[0];
    float q1=q[1];
    float q2=q[2];
    float q3=q[3];

    if(pitch)
        *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
    if(roll)
        *roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;
    if(yaw)
        *yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;
}
