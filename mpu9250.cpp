/*
 * MIT License
 *
 * Copyright (c) 2020 Hyeonki Hong <hhk7734@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "mpu9250.h"

#include <math.h>

namespace lot {
static const uint8_t SPI_READ_REG  = 0x80;
static const uint8_t SPI_WRITE_REG = 0x00;
static const int16_t ACC_1G_FS     = 4096 - 1;

Mpu9250::Mpu9250(SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *     GPIOx,
                 uint16_t           GPIO_Pin)
    : hspi(hspi)
    , gpiox(GPIOx)
    , gpio_pin(GPIO_Pin) {}

HAL_StatusTypeDef Mpu9250::init(void) {
    HAL_StatusTypeDef status;
    int16_t           id;

    set_read_only_mode(false);

    status = reset();

    id = read_reg(MPU9250_WHO_AM_I);
    if(status != HAL_OK || (id != 0x71 && id != 0x73)) status = HAL_ERROR;

    status = status == HAL_OK
                 ? write_reg(MPU9250_PWR_MGMT_1, MPU9250_CLKSEL_PLL)
                 : HAL_ERROR;

    // Set Mag first.

    /*
     * Magnetometer(MPU9250(master) - AK8963(slave))
     * I2C clock: 400kHz
     * Sample Rate 100Hz
     * 4900uT (14-bit)
     */

    status = status == HAL_OK ? write_reg(MPU9250_USER_CTRL, MPU9250_I2C_MST_EN)
                              : HAL_ERROR;
    status = status == HAL_OK
                 ? write_reg(MPU9250_I2C_MST_CTRL, MPU9250_I2C_MST_CLK_13)
                 : HAL_ERROR;

    status = status == HAL_OK ? reset_ak8963() : HAL_ERROR;
    // After getting sensitivity, AK8963_POWER_DOWN_MODE is set.
    status = status == HAL_OK ? get_ak8963_sensitivity() : HAL_ERROR;

    id = read_ak8963_reg(AK8963_WHO_AM_I);
    if(status != HAL_OK || id != 0x48) status = HAL_ERROR;

    status = status == HAL_OK ? write_reg(MPU9250_I2C_MST_DELAY_CTRL,
                                          MPU9250_I2C_SLV0_DLY_EN)
                              : HAL_ERROR;
    status = status == HAL_OK ? set_ak8963_resolution(AK8963_14_BIT_OUTPUT)
                              : HAL_ERROR;
    status = status == HAL_OK ? enable_ak8963_slave(true) : HAL_ERROR;

    /*
     * Gyroscope
     * LPF Bandwidth 250Hz
     * Delay 0.96ms
     * Sample Rate 8kHz
     * 2000degree/s (16-bit)
     */
    status = status == HAL_OK ? write_reg(MPU9250_CONFIG, MPU9250_DLPF_CFG_0)
                              : HAL_ERROR;
    status
        = status == HAL_OK
              ? write_reg(MPU9250_GYRO_CONFIG,
                          MPU9250_GYRO_FS_SEL_2000DPS | MPU9250_GYRO_FCHOICE_11)
              : HAL_ERROR;

    // At least 300Hz
    status = status == HAL_OK ? set_ak8963_sampling_divider((8000 / 300) - 1)
                              : HAL_ERROR;

    /*
     * Accelerometer
     * LPF Bandwidth 99Hz
     * Delay 2.88ms
     * Sample Rate 1kHz
     * 8g (16-bit)
     */
    status = status == HAL_OK
                 ? write_reg(MPU9250_ACCEL_CONFIG, MPU9250_ACCEL_FS_SEL_8G)
                 : HAL_ERROR;
    status = status == HAL_OK
                 ? write_reg(MPU9250_ACCEL_CONFIG2,
                             MPU9250_ACCEL_FCHOICE_1 | MPU9250_A_DLPFCFG_2)
                 : HAL_ERROR;

    set_read_only_mode(true);

    return status;
}

HAL_StatusTypeDef Mpu9250::reset(void) {
    HAL_StatusTypeDef status;
    status = write_reg(MPU9250_PWR_MGMT_1, MPU9250_H_RESET);
    HAL_Delay(100);
    return status;
}

HAL_StatusTypeDef Mpu9250::calibrate_acc(void) {
    int16_t           raw_acc[3];
    int32_t           raw_acc_sum[3] = {0};
    HAL_StatusTypeDef status;

    for(int16_t i = 0; i < 256; ++i) {
        status = read_3_axis(MPU9250_ACCEL_XOUT_H, raw_acc);
        if(status != HAL_OK) { return status; }
        raw_acc_sum[0] += raw_acc[0];
        raw_acc_sum[1] += raw_acc[1];
        raw_acc_sum[2] += raw_acc[2];
        HAL_Delay(2);
    }

    acc_offset[0] = raw_acc_sum[0] >> 8;
    acc_offset[1] = raw_acc_sum[1] >> 8;
    acc_offset[2] = raw_acc_sum[2] >> 8;

    // an axis must be parallel to
    // vertical
    for(int16_t i = 0; i < 3; ++i) {
        if(acc_offset[i] > ACC_1G_FS * 0.9) {
            acc_offset[i] -= ACC_1G_FS;
        } else if(acc_offset[i] < -ACC_1G_FS * 0.9) {
            acc_offset[i] += ACC_1G_FS;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef Mpu9250::calibrate_gyro(void) {
    int16_t           raw_gyro[3];
    int32_t           raw_gyro_sum[3] = {0};
    HAL_StatusTypeDef status;

    for(int16_t i = 0; i < 256; ++i) {
        status = read_3_axis(MPU9250_GYRO_XOUT_H, raw_gyro);
        if(status != HAL_OK) { return status; }
        raw_gyro_sum[0] += raw_gyro[0];
        raw_gyro_sum[1] += raw_gyro[1];
        raw_gyro_sum[2] += raw_gyro[2];
        HAL_Delay(2);
    }

    gyro_offset[0] = raw_gyro_sum[0] >> 8;
    gyro_offset[1] = raw_gyro_sum[1] >> 8;
    gyro_offset[2] = raw_gyro_sum[2] >> 8;

    return HAL_OK;
}

HAL_StatusTypeDef Mpu9250::calibrate_mag(void) {
    HAL_StatusTypeDef status;
    int16_t           raw_mag[3];
    uint8_t           temp[6];
    double            component[6];
    double            sigma_A[21] = {0};
    double            sigma_b[6]  = {0};

    // M^t * M * coef = M^t * 1[k:1]
    // sA * coef = sb
    // ax^2 + by^2 + cz^2 + dx + ey + fz
    // = 1
    for(int16_t k = 0; k < 100; ++k) {
        status = read_3_axis(MPU9250_EXT_SENS_DATA_00, raw_mag);
        if(status != HAL_OK) { return status; }
        component[3] = raw_mag[0];
        component[4] = raw_mag[1];
        component[5] = raw_mag[2];
        component[0] = component[3] * component[3];
        component[1] = component[4] * component[4];
        component[2] = component[5] * component[5];

        // Lower triangular matrix
        for(int16_t i = 0; i < 6; ++i) {
            temp[i] = i * (i + 1) >> 1;    // row
            for(int16_t j = 0; j < i + 1; ++j) {
                sigma_A[temp[i] + j] += component[i] * component[j];
            }
            sigma_b[i] += component[i];
        }

        HAL_Delay(200);
    }

    // Cholesky decomposition
    // sA = L * L^t
    for(int16_t i = 0; i < 6; ++i) {
        for(int16_t j = 0; j < i + 1; ++j) {
            if(i == j) {
                sigma_A[temp[i] + i] = sqrt(sigma_A[temp[i] + i]);
            } else {
                for(int16_t k = 0; k < j; ++k) {
                    sigma_A[temp[i] + j]
                        -= sigma_A[temp[i] + k] * sigma_A[temp[j] + k];
                }
                sigma_A[temp[i] + j] /= sigma_A[temp[j] + j];
                sigma_A[temp[i] + i]
                    -= sigma_A[temp[i] + j] * sigma_A[temp[i] + j];
            }
        }
    }

    // L * ( L^t * coef ) = b
    // L * x = b
    // component == x
    for(int16_t i = 0; i < 6; ++i) {
        for(int16_t j = 0; j < i; ++j) {
            sigma_b[i] -= sigma_A[temp[i] + j] * component[j];
        }
        component[i] = sigma_b[i] / sigma_A[temp[i] + i];
    }

    // L^t * coef = x
    // component == x
    // sigma_b == coef
    for(int16_t i = 5; i >= 0; --i) {
        for(int16_t j = 5; j > i; --j) {
            component[i] -= sigma_A[temp[j] + i] * sigma_b[j];
        }
        sigma_b[i] = component[i] / sigma_A[temp[i] + i];
    }

    mag_offset[0]      = (-sigma_b[3] / sigma_b[0]) / 2;
    mag_offset[1]      = (-sigma_b[4] / sigma_b[1]) / 2;
    mag_offset[2]      = (-sigma_b[5] / sigma_b[2]) / 2;
    mag_sensitivity[0] = 1;
    mag_sensitivity[1] = sqrt(sigma_b[1] / sigma_b[0]);
    mag_sensitivity[2] = sqrt(sigma_b[2] / sigma_b[0]);

    return HAL_OK;
}

HAL_StatusTypeDef Mpu9250::set_ak8963_resolution(uint8_t resolution) {
    HAL_StatusTypeDef status;
    if(resolution == AK8963_16_BIT_OUTPUT) {
        ak8963_resolution = AK8963_16_BIT_OUTPUT;
        status            = write_ak8963_reg(AK8963_CNTL1,
                                  AK8963_16_BIT_OUTPUT | ak8963_mode);
    } else {
        ak8963_resolution = AK8963_14_BIT_OUTPUT;
        status            = write_ak8963_reg(AK8963_CNTL1,
                                  AK8963_14_BIT_OUTPUT | ak8963_mode);
    }
    return status;
}

HAL_StatusTypeDef Mpu9250::set_ak8963_mode(uint8_t mode) {
    HAL_StatusTypeDef status;
    ak8963_mode = mode;
    status      = write_ak8963_reg(AK8963_CNTL1, ak8963_resolution | mode);
    return status;
}

HAL_StatusTypeDef Mpu9250::set_ak8963_sampling_divider(uint8_t divider) {
    if(divider > 32 - 1) { divider = 31; }
    ak8963_sampling_divider = divider;
    return write_reg(MPU9250_I2C_SLV4_CTRL, divider);
}

void Mpu9250::get_acc_offset(int16_t *xyz_offset) {
    xyz_offset[0] = acc_offset[0];
    xyz_offset[1] = acc_offset[1];
    xyz_offset[2] = acc_offset[2];
}

void Mpu9250::get_gyro_offset(int16_t *xyz_offset) {
    xyz_offset[0] = gyro_offset[0];
    xyz_offset[1] = gyro_offset[1];
    xyz_offset[2] = gyro_offset[2];
}

void Mpu9250::get_mag_offset(int16_t *xyz_offset) {
    xyz_offset[0] = mag_offset[0];
    xyz_offset[1] = mag_offset[1];
    xyz_offset[2] = mag_offset[2];
}

void Mpu9250::get_mag_sensitivity(float *xyz_sensitivity) {
    xyz_sensitivity[0] = mag_sensitivity[0];
    xyz_sensitivity[1] = mag_sensitivity[1];
    xyz_sensitivity[2] = mag_sensitivity[2];
}

void Mpu9250::set_acc_offset(int16_t *xyz_offset) {
    acc_offset[0] = xyz_offset[0];
    acc_offset[1] = xyz_offset[1];
    acc_offset[2] = xyz_offset[2];
}

void Mpu9250::set_gyro_offset(int16_t *xyz_offset) {
    gyro_offset[0] = xyz_offset[0];
    gyro_offset[1] = xyz_offset[1];
    gyro_offset[2] = xyz_offset[2];
}

void Mpu9250::set_mag_offset(int16_t *xyz_offset) {
    mag_offset[0] = xyz_offset[0];
    mag_offset[1] = xyz_offset[1];
    mag_offset[2] = xyz_offset[2];
}

void Mpu9250::set_mag_sensitivity(float *xyz_sensitivity) {
    mag_sensitivity[0] = xyz_sensitivity[0];
    mag_sensitivity[1] = xyz_sensitivity[1];
    mag_sensitivity[2] = xyz_sensitivity[2];
}

HAL_StatusTypeDef Mpu9250::get_acc(int16_t *xyz) {
    HAL_StatusTypeDef status;
    status = read_3_axis(MPU9250_ACCEL_XOUT_H, xyz);
    xyz[0] -= acc_offset[0];
    xyz[1] -= acc_offset[1];
    xyz[2] -= acc_offset[2];
    return status;
}

HAL_StatusTypeDef Mpu9250::get_gyro(int16_t *xyz) {
    HAL_StatusTypeDef status;
    status = read_3_axis(MPU9250_GYRO_XOUT_H, xyz);
    xyz[0] -= gyro_offset[0];
    xyz[1] -= gyro_offset[1];
    xyz[2] -= gyro_offset[2];
    return status;
}

HAL_StatusTypeDef Mpu9250::get_mag(int16_t *xyz) {
    HAL_StatusTypeDef status;
    int16_t           mag_xyz[3];
    status = read_3_axis(MPU9250_EXT_SENS_DATA_00, mag_xyz);

    /**
     * Orientation
     * mag_x acc_y
     * mag_y acc_x
     * mag_z -acc_z
     */
    xyz[0] = mag_sensitivity[1] * (mag_xyz[1] - mag_offset[1]);
    xyz[1] = mag_sensitivity[0] * (mag_xyz[0] - mag_offset[0]);
    xyz[2] = -mag_sensitivity[2] * (mag_xyz[2] - mag_offset[2]);

    return status;
}

HAL_StatusTypeDef
    Mpu9250::get_all(int16_t *acc_xyz, int16_t *gyro_xyz, int16_t *mag_xyz) {
    uint8_t temp[22];
    int16_t raw_mag_xyz[3];
    temp[0]                  = SPI_READ_REG | MPU9250_ACCEL_XOUT_H;
    HAL_StatusTypeDef status = transceive(temp, temp, 22);

    acc_xyz[0]     = (temp[1] << 8) | temp[2];
    acc_xyz[1]     = (temp[3] << 8) | temp[4];
    acc_xyz[2]     = (temp[5] << 8) | temp[6];
    gyro_xyz[0]    = (temp[9] << 8) | temp[10];
    gyro_xyz[1]    = (temp[11] << 8) | temp[12];
    gyro_xyz[2]    = (temp[13] << 8) | temp[14];
    raw_mag_xyz[0] = (temp[15] << 8) | temp[16];
    raw_mag_xyz[1] = (temp[17] << 8) | temp[18];
    raw_mag_xyz[2] = (temp[19] << 8) | temp[20];

    acc_xyz[0] -= acc_offset[0];
    acc_xyz[1] -= acc_offset[1];
    acc_xyz[2] -= acc_offset[2];
    gyro_xyz[0] -= gyro_offset[0];
    gyro_xyz[1] -= gyro_offset[1];
    gyro_xyz[2] -= gyro_offset[2];

    /**
     * Orientation
     * mag_x acc_y
     * mag_y acc_x
     * mag_z -acc_z
     */
    mag_xyz[0] = mag_sensitivity[1] * (raw_mag_xyz[1] - mag_offset[1]);
    mag_xyz[1] = mag_sensitivity[0] * (raw_mag_xyz[0] - mag_offset[0]);
    mag_xyz[2] = -mag_sensitivity[2] * (raw_mag_xyz[2] - mag_offset[2]);

    return status;
}

void Mpu9250::get_acc_lpf(float alpha, int16_t *input_xyz, int16_t *xyz_lpf) {
    acc_lpf[0] = alpha * (float)input_xyz[0] + (1 - alpha) * acc_lpf[0];
    acc_lpf[1] = alpha * (float)input_xyz[1] + (1 - alpha) * acc_lpf[1];
    acc_lpf[2] = alpha * (float)input_xyz[2] + (1 - alpha) * acc_lpf[2];

    xyz_lpf[0] = acc_lpf[0];
    xyz_lpf[1] = acc_lpf[1];
    xyz_lpf[2] = acc_lpf[2];
}

void Mpu9250::get_gyro_lpf(float alpha, int16_t *input_xyz, int16_t *xyz_lpf) {
    gyro_lpf[0] = alpha * (float)input_xyz[0] + (1 - alpha) * gyro_lpf[0];
    gyro_lpf[1] = alpha * (float)input_xyz[1] + (1 - alpha) * gyro_lpf[1];
    gyro_lpf[2] = alpha * (float)input_xyz[2] + (1 - alpha) * gyro_lpf[2];

    xyz_lpf[0] = gyro_lpf[0];
    xyz_lpf[1] = gyro_lpf[1];
    xyz_lpf[2] = gyro_lpf[2];
}

void Mpu9250::get_mag_lpf(float alpha, int16_t *input_xyz, int16_t *xyz_lpf) {
    mag_lpf[0] = alpha * (float)input_xyz[0] + (1 - alpha) * mag_lpf[0];
    mag_lpf[1] = alpha * (float)input_xyz[1] + (1 - alpha) * mag_lpf[1];
    mag_lpf[2] = alpha * (float)input_xyz[2] + (1 - alpha) * mag_lpf[2];

    xyz_lpf[0] = mag_lpf[0];
    xyz_lpf[1] = mag_lpf[1];
    xyz_lpf[2] = mag_lpf[2];
}

HAL_StatusTypeDef
    Mpu9250::transceive(uint8_t *tx_buff, uint8_t *rx_buff, uint16_t size) {
    HAL_StatusTypeDef status;
    HAL_GPIO_WritePin(gpiox, gpio_pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(hspi, tx_buff, rx_buff, size, size);
    HAL_GPIO_WritePin(gpiox, gpio_pin, GPIO_PIN_SET);
    return status;
}

HAL_StatusTypeDef Mpu9250::write_reg(uint8_t reg, uint8_t value) {
    uint8_t buff[2] = {static_cast<uint8_t>(SPI_WRITE_REG | reg), value};
    return transceive(buff, buff, 2);
}

int16_t Mpu9250::read_reg(uint8_t reg) {
    uint8_t buff[2] = {static_cast<uint8_t>(SPI_READ_REG | reg), 0};
    if(transceive(buff, buff, 2) != HAL_OK) { return -1; }
    return buff[1];
}

HAL_StatusTypeDef Mpu9250::write_ak8963_reg(uint8_t reg, uint8_t value) {
    HAL_StatusTypeDef status;
    status = write_reg(MPU9250_I2C_SLV4_ADDR,
                       MPU9250_I2C_SLVx_WRITE | AK8963_ADDRESS);
    status
        = status == HAL_OK ? write_reg(MPU9250_I2C_SLV4_REG, reg) : HAL_ERROR;
    status
        = status == HAL_OK ? write_reg(MPU9250_I2C_SLV4_DO, value) : HAL_ERROR;
    // SLV4_EN bit is cleared when a single transfer is complete.
    status = status == HAL_OK
                 ? write_reg(MPU9250_I2C_SLV4_CTRL,
                             MPU9250_I2C_SLVx_EN | ak8963_sampling_divider)
                 : HAL_ERROR;

    HAL_Delay(2);

    return status;
}

int16_t Mpu9250::read_ak8963_reg(uint8_t reg) {
    HAL_StatusTypeDef status;
    int16_t           data;

    status = write_reg(MPU9250_I2C_SLV4_ADDR,
                       MPU9250_I2C_SLVx_READ | AK8963_ADDRESS);
    status
        = status == HAL_OK ? write_reg(MPU9250_I2C_SLV4_REG, reg) : HAL_ERROR;
    // SLV4_EN bit is cleared when a single transfer is complete.
    status = status == HAL_OK
                 ? write_reg(MPU9250_I2C_SLV4_CTRL,
                             MPU9250_I2C_SLVx_EN | ak8963_sampling_divider)
                 : HAL_ERROR;

    HAL_Delay(2);

    data = read_reg(MPU9250_I2C_SLV4_DI);
    if(data == -1 || status != HAL_OK) return -1;
    return data;
}

void Mpu9250::set_read_only_mode(bool enable) {
    if(enable) {
        // In read only mode, 20 MHz is the highest speed.
        hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
        WRITE_REG(hspi->Instance->CR1,
                  (hspi->Init.Mode | hspi->Init.Direction | hspi->Init.DataSize
                   | hspi->Init.CLKPolarity | hspi->Init.CLKPhase
                   | (hspi->Init.NSS & SPI_CR1_SSM)
                   | hspi->Init.BaudRatePrescaler | hspi->Init.FirstBit
                   | hspi->Init.CRCCalculation));
    } else {
        // In normal mode, 1 MHz is the highest speed.
        hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
        WRITE_REG(hspi->Instance->CR1,
                  (hspi->Init.Mode | hspi->Init.Direction | hspi->Init.DataSize
                   | hspi->Init.CLKPolarity | hspi->Init.CLKPhase
                   | (hspi->Init.NSS & SPI_CR1_SSM)
                   | hspi->Init.BaudRatePrescaler | hspi->Init.FirstBit
                   | hspi->Init.CRCCalculation));
    }

    HAL_Delay(10);
}

HAL_StatusTypeDef Mpu9250::read_3_axis(uint8_t reg, int16_t *xyz) {
    HAL_StatusTypeDef status;
    uint8_t           temp[7];
    temp[0] = SPI_READ_REG | reg;
    status  = transceive(temp, temp, 7);
    xyz[0]  = (temp[1] << 8) | temp[2];
    xyz[1]  = (temp[3] << 8) | temp[4];
    xyz[2]  = (temp[5] << 8) | temp[6];
    return status;
}

HAL_StatusTypeDef Mpu9250::reset_ak8963(void) {
    write_ak8963_reg(AK8963_CNTL2, AK8963_SOFT_RESET);
    do { HAL_Delay(10); } while(read_ak8963_reg(AK8963_CNTL2));
    return HAL_OK;
}

HAL_StatusTypeDef Mpu9250::get_ak8963_sensitivity(void) {
    uint8_t           _mode = ak8963_mode;
    HAL_StatusTypeDef status;

    status = set_ak8963_mode(AK8963_POWER_DOWN_MODE);
    status = status == HAL_OK ? set_ak8963_mode(AK8963_FUSE_ROM_ACCESS_MODE)
                              : HAL_ERROR;

    mag_sensitivity[0] = read_ak8963_reg(AK8963_ASAX) - 128;
    mag_sensitivity[0] = (mag_sensitivity[0] / 256.0f) + 1.0f;
    mag_sensitivity[1] = read_ak8963_reg(AK8963_ASAY) - 128;
    mag_sensitivity[1] = (mag_sensitivity[1] / 256.0f) + 1.0f;
    mag_sensitivity[2] = read_ak8963_reg(AK8963_ASAZ) - 128;
    mag_sensitivity[2] = (mag_sensitivity[2] / 256.0f) + 1.0f;

    status = status == HAL_OK ? set_ak8963_mode(AK8963_POWER_DOWN_MODE)
                              : HAL_ERROR;
    return status;
}

HAL_StatusTypeDef Mpu9250::enable_ak8963_slave(bool enabled) {
    HAL_StatusTypeDef status;
    if(enabled) {
        /*
         * XL, XH, YL, YH, ZL, ZH, ST2
         * -> XH, XL, YH, YL, ZH, ZL, ST2
         * When ST2 register is read, AK8963 judges that data reading is
         * finished.
         */
        status = set_ak8963_mode(AK8963_CONTINUOUS_MEASUREMENT_MODE_2);
        status = status == HAL_OK
                     ? write_reg(MPU9250_I2C_SLV0_ADDR,
                                 MPU9250_I2C_SLVx_READ | AK8963_ADDRESS)
                     : HAL_ERROR;
        status = status == HAL_OK
                     ? write_reg(MPU9250_I2C_SLV0_REG, AK8963_XOUT_L)
                     : HAL_ERROR;
        status
            = status == HAL_OK
                  ? write_reg(MPU9250_I2C_SLV0_CTRL,
                              MPU9250_I2C_SLVx_EN | MPU9250_I2C_SLVx_BYTE_SWAP
                                  | MPU9250_I2C_SLVx_GRP_EVEN_END
                                  | (7 * MPU9250_I2C_SLVx_LENG))
                  : HAL_ERROR;
    } else {
        status = write_reg(MPU9250_I2C_SLV0_CTRL, 0);
        status = status == HAL_OK ? set_ak8963_mode(AK8963_POWER_DOWN_MODE)
                                  : HAL_ERROR;
    }

    return status;
}
}    // namespace lot