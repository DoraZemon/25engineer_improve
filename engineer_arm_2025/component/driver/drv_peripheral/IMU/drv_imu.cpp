//
// Created by 34147 on 2024/1/26.
//

#include "drv_imu.h"

#include "ist8310_reg.h"
#include "stm32f4xx_hal.h"
#include "mpu6500_reg.h"
#include "spi.h"
#include "tim.h"

#define BOARD_DOWN (1)

#define MPU_HSPI hspi5

#define Kp 2.0f                                              /*
                                                              * proportional gain governs rate of
                                                              * convergence to accelerometer/magnetometer
																															*/
#define Ki 0.01f                                             /*
                                                              * integral gain governs rate of
                                                              * convergence of gyroscope biases
																															*/
volatile float q0 = 1.0f;
volatile float q1 = 0.0f;
volatile float q2 = 0.0f;
volatile float q3 = 0.0f;
volatile float exInt, eyInt, ezInt;                   /* error integral */
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;
volatile uint32_t last_update, now_update;               /* Sampling cycle count, ubit ms */
static uint8_t tx, rx;
static uint8_t tx_buff[14] = {0xff};
uint8_t mpu_buff[14];                          /* buffer to save imu raw data */
uint8_t ist_buff[6];                           /* buffer to save IST8310 raw data */
float gyroDiff[3], gNormDiff;
uint32_t INS_DWT_Count = 0;
/**
  * @brief  fast inverse square-root, to calculate 1/Sqrt(x)
  * @param  x: the number need to be calculated
  * @retval 1/Sqrt(x)
  * @usage  call in imu_ahrs_update() function
  */
float inv_sqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;

    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));

    return y;
}

/**
  * @brief  write a byte of data to specified register
  * @param  reg:  the address of register to be written
  *         data: data to be written
  * @retval
  * @usage  call in ist_reg_write_by_mpu(),
  *                 ist_reg_read_by_mpu(),
  *                 mpu_master_i2c_auto_read_config(),
  *                 ist8310_init(),
  *                 mpu_set_gyro_fsr(),
  *                 mpu_set_accel_fsr(),
  *                 mpu_device_init() function
  */
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data) {
    MPU_NSS_LOW;
    tx = reg & 0x7F;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    tx = data;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return 0;
}

/**
  * @brief  read a byte of data from specified register
  * @param  reg: the address of register to be read
  * @retval
  * @usage  call in ist_reg_read_by_mpu(),
  *                 mpu_device_init() function
  */
uint8_t mpu_read_byte(uint8_t const reg) {
    MPU_NSS_LOW;
    tx = reg | 0x80;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return rx;
}

/**
  * @brief  read bytes of data from specified register
  * @param  reg: address from where data is to be written
  * @retval
  * @usage  call in ist8310_get_data(),
  *                 mpu_get_data(),
  *                 mpu_offset_call() function
  */
uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t *pData, uint8_t len) {
    MPU_NSS_LOW;
    tx = regAddr | 0x80;
    tx_buff[0] = tx;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);
    MPU_NSS_HIGH;
    return 0;
}

uint8_t mpu_read_bytes_dma(uint8_t const regAddr, uint8_t *pData, uint8_t len) {
    MPU_NSS_LOW;
    tx = regAddr | 0x80;
    tx_buff[0] = tx;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive_DMA(&MPU_HSPI, tx_buff, pData, len);
//    MPU_NSS_HIGH;
    return 0;
}

/**
	* @brief  write IST8310 register through MPU6500's I2C master
  * @param  addr: the address to be written of IST8310's register
  *         data: data to be written
	* @retval
  * @usage  call in ist8310_init() function
	*/
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data) {
    /* turn off slave 1 at first */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    HAL_Delay(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
    HAL_Delay(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
    HAL_Delay(2);
    /* turn on slave 1 with one byte transmitting */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* wait longer to ensure the data is transmitted from slave 1 */
    HAL_Delay(10);
}

/**
	* @brief  write IST8310 register through MPU6500's I2C Master
	* @param  addr: the address to be read of IST8310's register
	* @retval
  * @usage  call in ist8310_init() function
	*/
static uint8_t ist_reg_read_by_mpu(uint8_t addr) {
    uint8_t retval;
    mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
    HAL_Delay(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
    HAL_Delay(10);
    retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
    /* turn off slave4 after read */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    HAL_Delay(10);
    return retval;
}

/**
	* @brief    initialize the MPU6500 I2C Slave 0 for I2C reading.
* @param    device_address: slave device address, Address[6:0]
	* @retval   void
	* @note
	*/
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num) {
    /*
	   * configure the device address of the IST8310
     * use slave1, auto transmit single measure mode
	   */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
    HAL_Delay(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    HAL_Delay(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    HAL_Delay(2);

    /* use slave0,auto read data */
    mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    HAL_Delay(2);
    mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
    HAL_Delay(2);

    /* every eight mpu6500 internal samples one i2c master read */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
    HAL_Delay(2);
    /* enable slave 0 and 1 access delay */
    mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    HAL_Delay(2);
    /* enable slave 1 auto transmit */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* Wait 6ms (minimum waiting time for 16 times internal average setup) */
    HAL_Delay(6);
    /* enable slave 0 with data_num bytes reading */
    mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    HAL_Delay(2);
}

/**
	* @brief  Initializes the IST8310 device
	* @param
	* @retval
  * @usage  call in mpu_device_init() function
	*/
uint8_t ist8310_init() {
    /* enable iic master mode */
    mpu_write_byte(MPU6500_USER_CTRL, 0x30);
    HAL_Delay(10);
    /* enable iic 400khz */
    mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d);
    HAL_Delay(10);

    /* turn on slave 1 for ist write and slave 4 to ist read */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
    HAL_Delay(10);
    mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
    HAL_Delay(10);

    /* IST8310_R_CONFB 0x01 = device rst */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    HAL_Delay(10);
    if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
        return 1;

    /* soft reset */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    HAL_Delay(10);

    /* config as ready mode to access register */
    ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
        return 2;
    HAL_Delay(10);

    /* normal state, no int */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
        return 3;
    HAL_Delay(10);

    /* config low noise mode, x,y,z axis 16 time 1 avg */
    ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
    if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
        return 4;
    HAL_Delay(10);

    /* Set/Reset pulse duration setup,normal mode */
    ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
    if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
        return 5;
    HAL_Delay(10);

    /* turn off slave1 & slave 4 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    HAL_Delay(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    HAL_Delay(10);

    /* configure and turn on slave 0 */
    mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    HAL_Delay(10);
    return 0;
}

void ist8310_get_data() {
    mpu_read_bytes_dma(MPU6500_EXT_SENS_DATA_00, ist_buff, 6);
}

void mpu6500_get_data() {
    mpu_read_bytes_dma(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
}

void mpu_get_data(imu_data_t *data) {
    static int16_t mpu6500_raw_temp;
    static int16_t m[3];
    /* 8g -> m/s2 */
    mpu6500_raw_temp = (int16_t) (mpu_buff[0] << 8 | mpu_buff[1]);
    data->accel[0] = mpu6500_raw_temp * MPU6500_ACCEL_8G_SEN * data->AccelScale;
    mpu6500_raw_temp = (int16_t) (mpu_buff[2] << 8 | mpu_buff[3]);
    data->accel[1] = mpu6500_raw_temp * MPU6500_ACCEL_8G_SEN * data->AccelScale;
    mpu6500_raw_temp = (int16_t) (mpu_buff[4] << 8 | mpu_buff[5]);
    data->accel[2] = mpu6500_raw_temp * MPU6500_ACCEL_8G_SEN * data->AccelScale;

//    g_imu.data.temperature = mpu_buff[6] << 8 | mpu_buff[7];
    /* 2000dps -> rad/s */
    mpu6500_raw_temp = (int16_t) (mpu_buff[8] << 8 | mpu_buff[9]);
    data->gyro[0] = (mpu6500_raw_temp * MPU6500_GYRO_2000_SEN - data->gyro_offset[0]);
    mpu6500_raw_temp = (int16_t) (mpu_buff[10] << 8 | mpu_buff[11]);
    data->gyro[1] = (mpu6500_raw_temp * MPU6500_GYRO_2000_SEN - data->gyro_offset[1]);
    mpu6500_raw_temp = (int16_t) (mpu_buff[12] << 8 | mpu_buff[13]);
    data->gyro[2] = (mpu6500_raw_temp * MPU6500_GYRO_2000_SEN - data->gyro_offset[2]);

    mpu6500_raw_temp = (int16_t) (mpu_buff[6] << 8 | mpu_buff[7]);

    data->temperature = mpu6500_raw_temp * MPU6500_TEMP_FACTOR + MPU6500_TEMP_OFFSET;

    memcpy(&data->mx, ist_buff, 6);

}

/**
	* @brief  set imu 6500 gyroscope measure range
  * @param  fsr: range(0,±250dps;1,±500dps;2,±1000dps;3,±2000dps)
	* @retval
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_gyro_fsr(uint8_t fsr) {
    return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}

/**
	* @brief  set imu 6050/6500 accelerate measure range
  * @param  fsr: range(0,±2g;1,±4g;2,±8g;3,±16g)
	* @retval
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_accel_fsr(uint8_t fsr) {
    return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3);
}

uint8_t id;

/**
	* @brief  initialize imu mpu6500 and magnet meter ist3810
  * @param
	* @retval
  * @usage  call in main() function
	*/
uint8_t mpu_device_init(void) {
//	MPU_DELAY(100);为了将初始化放进main函数
    HAL_Delay(100);

    id = mpu_read_byte(MPU6500_WHO_AM_I);
    uint8_t i = 0;
    uint8_t MPU6500_Init_Data[10][2] = {{MPU6500_PWR_MGMT_1, 0x80},     /* Reset Device */
                                        {MPU6500_PWR_MGMT_1, 0x03},     /* Clock Source - Gyro-Z */
                                        {MPU6500_PWR_MGMT_2, 0x00},     /* Enable Acc & Gyro */
                                        {MPU6500_CONFIG, 0x04},         /* LPF 41Hz */
                                        {MPU6500_GYRO_CONFIG, 0x18},    /* +-2000dps */
                                        {MPU6500_ACCEL_CONFIG, 0x10},   /* +-8G */
                                        {MPU6500_ACCEL_CONFIG_2, 0x02}, /* enable LowPassFilter  Set Acc LPF */
                                        {MPU6500_USER_CTRL, 0x20},};    /* Enable AUX */
    for (i = 0; i < 10; i++) {
        mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
        HAL_Delay(1);
    }

    mpu_set_gyro_fsr(3);
    mpu_set_accel_fsr(2);

    ist8310_init();
    return 0;
}

void mpu_get_offset(imu_data_t *data) {
    static uint32_t startTime;
    static uint16_t CaliTimes = 6000;
    int16_t mpu6500_raw_temp;
    float gyroMax[3], gyroMin[3];
    float gNormTemp, gNormMax, gNormMin;

    startTime = HAL_GetTick();
    do {
        if (HAL_GetTick() - startTime > 60000) {
            data->gyro_offset[0] = GxOFFSET;
            data->gyro_offset[1] = GyOFFSET;
            data->gyro_offset[2] = GzOFFSET;
            data->gNorm = gNORM;
            data->TempWhenCali = 40;
            break;
        }
        DWT_Delay(0.005);
        data->gyro_offset[0] = 0;
        data->gyro_offset[1] = 0;
        data->gyro_offset[2] = 0;
        data->gNorm = 0;

        for (uint16_t i = 0; i < CaliTimes; i++) {
            mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
            mpu6500_raw_temp = (int16_t) (mpu_buff[0] << 8 | mpu_buff[1]);
            data->accel[0] = mpu6500_raw_temp * MPU6500_ACCEL_8G_SEN;
            mpu6500_raw_temp = (int16_t) (mpu_buff[2] << 8 | mpu_buff[3]);
            data->accel[1] = mpu6500_raw_temp * MPU6500_ACCEL_8G_SEN;
            mpu6500_raw_temp = (int16_t) (mpu_buff[4] << 8 | mpu_buff[5]);
            data->accel[2] = mpu6500_raw_temp * MPU6500_ACCEL_8G_SEN;
            gNormTemp = sqrtf(data->accel[0] * data->accel[0] +
                data->accel[1] * data->accel[1] +
                data->accel[2] * data->accel[2]);
            data->gNorm += gNormTemp;

            mpu6500_raw_temp = (int16_t) (mpu_buff[8] << 8 | mpu_buff[9]);
            data->gyro[0] = (mpu6500_raw_temp * MPU6500_GYRO_2000_SEN);
            data->gyro_offset[0] += data->gyro[0];
            mpu6500_raw_temp = (int16_t) (mpu_buff[10] << 8 | mpu_buff[11]);
            data->gyro[1] = (mpu6500_raw_temp * MPU6500_GYRO_2000_SEN);
            data->gyro_offset[1] += data->gyro[1];
            mpu6500_raw_temp = (int16_t) (mpu_buff[12] << 8 | mpu_buff[13]);
            data->gyro[2] = (mpu6500_raw_temp * MPU6500_GYRO_2000_SEN);
            data->gyro_offset[2] += data->gyro[2];


            // 记录数据极差
            if (i == 0) {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++) {
                    gyroMax[j] = data->gyro[j];
                    gyroMin[j] = data->gyro[j];
                }
            } else {
                if (gNormTemp > gNormMax)
                    gNormMax = gNormTemp;
                if (gNormTemp < gNormMin)
                    gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++) {
                    if (data->gyro[j] > gyroMax[j])
                        gyroMax[j] = data->gyro[j];
                    if (data->gyro[j] < gyroMin[j])
                        gyroMin[j] = data->gyro[j];
                }
            }

            // 数据差异过大认为收到外界干扰，需重新校准
            gNormDiff = gNormMax - gNormMin;
            for (uint8_t j = 0; j < 3; j++)
                gyroDiff[j] = gyroMax[j] - gyroMin[j];
            if (gNormDiff > 0.5f ||
                gyroDiff[0] > 0.15f ||
                gyroDiff[1] > 0.15f ||
                gyroDiff[2] > 0.15f)
                break;
            DWT_Delay(0.0005);

        }

        // 取平均值得到标定结果
        data->gNorm /= (float) CaliTimes;
        for (float &i : data->gyro_offset)
            i /= (float) CaliTimes;

        // 记录标定时IMU温度
        mpu6500_raw_temp = mpu_buff[6] << 8 | mpu_buff[7];

        data->TempWhenCali = mpu6500_raw_temp * MPU6500_TEMP_FACTOR + MPU6500_TEMP_OFFSET;

    } while (gNormDiff > 0.5f ||
        fabsf(data->gNorm - 9.8f) > 0.5f ||
        gyroDiff[0] > 0.15f ||
        gyroDiff[1] > 0.15f ||
        gyroDiff[2] > 0.15f ||
        fabsf(data->gyro_offset[0]) > 0.02f ||
        fabsf(data->gyro_offset[1]) > 0.02f ||
        fabsf(data->gyro_offset[2]) > 0.02f);

    // 根据标定结果校准加速度计标度因数
    data->AccelScale = 9.81f / data->gNorm;
}

/**
	* @brief  Initialize quaternion
  * @param
	* @retval
  * @usage  call in main() function
	*/
void init_quaternion(imu_data_t *data) {
    int16_t hx, hy;//hz;

    hx = data->mx;
    hy = data->my;
    //hz = imu.mz;

#ifdef BOARD_DOWN
    if (hx < 0 && hy < 0) {
        if (fabs(hx / hy) >= 1) {
            q0 = -0.005;
            q1 = -0.199;
            q2 = 0.979;
            q3 = -0.0089;
        } else {
            q0 = -0.008;
            q1 = -0.555;
            q2 = 0.83;
            q3 = -0.002;
        }

    } else if (hx < 0 && hy > 0) {
        if (fabs(hx / hy) >= 1) {
            q0 = 0.005;
            q1 = -0.199;
            q2 = -0.978;
            q3 = 0.012;
        } else {
            q0 = 0.005;
            q1 = -0.553;
            q2 = -0.83;
            q3 = -0.0023;
        }

    } else if (hx > 0 && hy > 0) {
        if (fabs(hx / hy) >= 1) {
            q0 = 0.0012;
            q1 = -0.978;
            q2 = -0.199;
            q3 = -0.005;
        } else {
            q0 = 0.0023;
            q1 = -0.83;
            q2 = -0.553;
            q3 = 0.0023;
        }

    } else if (hx > 0 && hy < 0) {
        if (fabs(hx / hy) >= 1) {
            q0 = 0.0025;
            q1 = 0.978;
            q2 = -0.199;
            q3 = 0.008;
        } else {
            q0 = 0.0025;
            q1 = 0.83;
            q2 = -0.56;
            q3 = 0.0045;
        }
    }
#else
    if (hx < 0 && hy < 0)
{
    if (fabs(hx / hy) >= 1)
    {
        q0 = 0.195;
        q1 = -0.015;
        q2 = 0.0043;
        q3 = 0.979;
    }
    else
    {
        q0 = 0.555;
        q1 = -0.015;
        q2 = 0.006;
        q3 = 0.829;
    }

}
else if (hx < 0 && hy > 0)
{
    if(fabs(hx / hy) >= 1)
    {
        q0 = -0.193;
        q1 = -0.009;
        q2 = -0.006;
        q3 = 0.979;
    }
    else
    {
        q0 = -0.552;
        q1 = -0.0048;
        q2 = -0.0115;
        q3 = 0.8313;
    }

}
else if (hx > 0 && hy > 0)
{
    if(fabs(hx / hy) >= 1)
    {
        q0 = -0.9785;
        q1 = 0.008;
        q2 = -0.02;
        q3 = 0.195;
    }
    else
    {
        q0 = -0.9828;
        q1 = 0.002;
        q2 = -0.0167;
        q3 = 0.5557;
    }

}
else if (hx > 0 && hy < 0)
{
    if(fabs(hx / hy) >= 1)
    {
        q0 = -0.979;
        q1 = 0.0116;
        q2 = -0.0167;
        q3 = -0.195;
    }
    else
    {
        q0 = -0.83;
        q1 = 0.014;
        q2 = -0.012;
        q3 = -0.556;
    }
}
#endif
}

/**
	* @brief  update imu AHRS
  * @param
	* @retval
  * @usage  call in main() function
	*/
void imu_ahrs_update(imu_data_t *data) {
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez, halfT;
    float tempq0, tempq1, tempq2, tempq3;

    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    gx = data->gyro[0];
    gy = data->gyro[1];
    gz = data->gyro[2];
    ax = data->accel[0];
    ay = data->accel[1];
    az = data->accel[2];
    mx = data->mx;
    my = data->my;
    mz = data->mz;

    now_update = HAL_GetTick(); //ms
    halfT = ((float) (now_update - last_update) / 2000.0f);
    last_update = now_update;

    /* Fast inverse square-root */
    norm = inv_sqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

#ifdef IST8310
    norm = inv_sqrt(mx * mx + my * my + mz * mz);
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm;
#else
    mx = 0;
    my = 0;
    mz = 0;
#endif
    /* compute reference direction of flux */
    hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
    hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
    hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = hz;

    /* estimated direction of gravity and flux (v and w) */
    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
    wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
    wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);

    /*
     * error is sum of cross product between reference direction
     * of fields and direction measured by sensors
     */
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    /* PI */
    if (ex != 0.0f && ey != 0.0f && ez != 0.0f) {
        exInt = exInt + ex * Ki * halfT;
        eyInt = eyInt + ey * Ki * halfT;
        ezInt = ezInt + ez * Ki * halfT;

        gx = gx + Kp * ex + exInt;
        gy = gy + Kp * ey + eyInt;
        gz = gz + Kp * ez + ezInt;
    }

    tempq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    tempq1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    tempq2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    tempq3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

    /* normalise quaternion */
    norm = inv_sqrt(tempq0 * tempq0 + tempq1 * tempq1 + tempq2 * tempq2 + tempq3 * tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;
}

/**
	* @brief  update imu attitude
  * @param
	* @retval
  * @usage  call in main() function
	*/
void imu_attitude_update(imu_data_t *data) {
    /* yaw    -pi----pi */
    data->yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3;
    /* pitch  -pi/2----pi/2 */
    data->pitch = -asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
    /* roll   -pi----pi  */
    data->roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;
}

uint8_t imu_device::init(SPI_HandleTypeDef *hspi, uint8_t calibrate) {
    this->hspi = hspi;
    this->ready_flag = true;
    this->lost_flag = false;
    this->zero_offset_flag = false;
    this->enable_flag = true;
    this->offset_cnt = 0;
    mpu_device_init();
    init_quaternion(&this->data);
    this->temppid.pid_reset(9500.0f, 9000.0f, 1600.0f, 0.2f, 0.0f,0.0f, 0.0f, 0.0f);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    if (calibrate) {
        mpu_get_offset(&this->data);
        return 0;
    } else {
        this->data.gyro_offset[0] = GxOFFSET;
        this->data.gyro_offset[1] = GyOFFSET;
        this->data.gyro_offset[2] = GzOFFSET;
        this->data.gNorm = gNORM;
        this->data.AccelScale = 9.81f / this->data.gNorm;
        this->data.TempWhenCali = 40;
        return 0;
    }
}
#if MAHONY

uint32_t mahony_offset = false;
#endif
void imu_device::update_data() {
    taskENTER_CRITICAL();
    if (this->lost_flag) {
        this->zero_offset_flag = false;
        this->offset_cnt = 0;
    }
#if MAHONY
    if (mahony_offset < 1000) {
        mahony_offset++;
        taskEXIT_CRITICAL();
        return;
    }
#endif
    if (this->offset_cnt < 50 && (!this->zero_offset_flag)) {
        this->euler.Yaw.last_deg = this->euler.Yaw.current_deg;
        this->euler.Yaw.zero_offset_deg = (float) this->data.yaw;
        this->euler.Yaw.current_deg = this->euler.Yaw.zero_offset_deg;

        this->euler.Pitch.last_deg = this->euler.Pitch.current_deg;
        this->euler.Pitch.zero_offset_deg = (float) this->data.pitch;
        this->euler.Pitch.current_deg = this->euler.Pitch.zero_offset_deg;

        this->euler.Roll.last_deg = this->euler.Roll.current_deg;
        this->euler.Roll.zero_offset_deg = (float) this->data.roll;
        this->euler.Roll.current_deg = this->euler.Roll.zero_offset_deg;
        this->offset_cnt++;
        this->euler.Yaw.round_cnt = 0;
        this->euler.Roll.round_cnt = 0;
        this->euler.Pitch.round_cnt = 0;
        taskEXIT_CRITICAL();
        return;
    }
    this->zero_offset_flag = true;
    this->update_euler();
    taskEXIT_CRITICAL();
}

void imu_device::update_euler() {
    euler_t *euler = &this->euler;
    imu_data_t *raw = &this->data;

    euler->Yaw.last_deg = euler->Yaw.current_deg;
    euler->Pitch.last_deg = euler->Pitch.current_deg;
    euler->Roll.last_deg = euler->Roll.current_deg;

    euler->Yaw.current_deg = (float) raw->yaw;
    euler->Pitch.current_deg = (float) raw->pitch;
    euler->Roll.current_deg = (float) raw->roll;

    if (euler->Yaw.current_deg - euler->Yaw.last_deg < -IMU_YAW_RANGE) {
        euler->Yaw.round_cnt++;
    } else if (euler->Yaw.current_deg - euler->Yaw.last_deg > IMU_YAW_RANGE) {
        euler->Yaw.round_cnt--;
    }
    euler->Yaw.total_degree =
        (euler->Yaw.round_cnt * IMU_YAW_RANGE * 2 + euler->Yaw.current_deg - euler->Yaw.zero_offset_deg) / 360.0f;
// pitch轴变化很奇异

    if (euler->Roll.current_deg - euler->Roll.last_deg < -IMU_ROLL_RANGE) {
        euler->Roll.round_cnt++;
    } else if (euler->Roll.current_deg - euler->Roll.last_deg > IMU_ROLL_RANGE) {
        euler->Roll.round_cnt--;
    }
    euler->Roll.total_degree =
        (euler->Roll.round_cnt * IMU_ROLL_RANGE * 2 + euler->Roll.current_deg - euler->Roll.zero_offset_deg) /
            360.0f;
}

void imu_device::update_temperature_control() {
    uint16_t out;
    out = (uint16_t) this->temppid.pid_calculate(IMU_TEMP_TARGET, this->data.temperature);
    if (out < 0) {
        out = 0;
    }
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, out);
}

void imu_device::update_ready() {
    if (!this->lost_flag && this->zero_offset_flag) {
        this->ready_flag = true;
    } else {
        this->ready_flag = false;
    }
}

void imu_device::set_current_as_offset() {
    this->offset_cnt = 0;
    this->zero_offset_flag = false;
}

void imu_device::get_euler_whx() {
    static float s_dt;
    s_dt = DWT_GetDeltaT(&INS_DWT_Count);
    IMU_QuaternionEKF_Update(this->data.gyro[0], this->data.gyro[1], this->data.gyro[2], this->data.accel[0],
                             this->data.accel[1], this->data.accel[2], s_dt);
    this->data.yaw = QEKF_INS.Yaw;
    this->data.roll = QEKF_INS.Roll;
    this->data.pitch = QEKF_INS.Pitch;
}

void imu_device::enable() {
    this->enable_flag = true;
}

void imu_device::disable() {
    this->enable_flag = false;
}

void imu_device::ahrs_update() {
    imu_ahrs_update(&this->data);
}

void imu_device::attitude_update() {
    imu_attitude_update(&this->data);
}

void imu_device::get_data() {
    mpu_get_data(&this->data);
}

void imu_device::set_lost() {
    this->lost_flag = true;
}

void imu_device::set_connected() {
    this->lost_flag = false;
}

bool imu_device::check_enable() {
    return this->enable_flag;
}

bool imu_device::check_lost() {
    return this->lost_flag;
}

bool imu_device::check_zero_offset() {
    return this->zero_offset_flag;
}

float imu_device::get_yaw_total_rounds() {
    return this->euler.Yaw.total_degree;
}