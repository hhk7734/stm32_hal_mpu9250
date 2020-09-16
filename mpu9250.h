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

#pragma once

#include "main.h"
#include "mpu9250_reg_map.h"

#include <stdbool.h>
#include <stdint.h>

#define SPI_READ_REG  0x80
#define SPI_WRITE_REG 0x00

namespace lot {
class Mpu9250 {
public:
    Mpu9250(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

    HAL_StatusTypeDef init(void);
    HAL_StatusTypeDef reset(void);

    HAL_StatusTypeDef calibrate_acc(void);
    HAL_StatusTypeDef calibrate_gyro(void);
    HAL_StatusTypeDef calibrate_mag(void);

    HAL_StatusTypeDef set_ak8963_resolution(uint8_t resolution);
    HAL_StatusTypeDef set_ak8963_mode(uint8_t mode);
    HAL_StatusTypeDef set_ak8963_sampling_divider(uint8_t divider);

    void get_acc_offset(int16_t *xyz_offset);
    void get_gyro_offset(int16_t *xyz_offset);
    void get_mag_offset(int16_t *xyz_offset);
    void get_mag_sensitivity(float *xyz_sensitivity);
    void set_acc_offset(int16_t *xyz_offset);
    void set_gyro_offset(int16_t *xyz_offset);
    void set_mag_offset(int16_t *xyz_offset);
    void set_mag_sensitivity(float *xyz_sensitivity);

    HAL_StatusTypeDef get_acc(int16_t *xyz);
    HAL_StatusTypeDef get_gyro(int16_t *xyz);
    HAL_StatusTypeDef get_mag(int16_t *xyz);
    HAL_StatusTypeDef
        get_all(int16_t *acc_xyz, int16_t *gyro_xyz, int16_t *mag_xyz);

    void get_acc_lpf(float alpha, int16_t *input_xyz, int16_t *xyz_lpf);
    void get_gyro_lpf(float alpha, int16_t *input_xyz, int16_t *xyz_lpf);
    void get_mag_lpf(float alpha, int16_t *input_xyz, int16_t *xyz_lpf);

private:
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *     gpiox;
    uint16_t           gpio_pin;

    uint8_t ak8963_mode             = AK8963_POWER_DOWN_MODE;
    uint8_t ak8963_resolution       = AK8963_14_BIT_OUTPUT;
    uint8_t ak8963_sampling_divider = 5 - 1;

    int16_t acc_offset[3]      = {0};
    int16_t gyro_offset[3]     = {0};
    int16_t mag_offset[3]      = {0};
    float   mag_sensitivity[3] = {1.0, 1.0, 1.0};
    float   acc_lpf[3]         = {0.0};
    float   gyro_lpf[3]        = {0.0};
    float   mag_lpf[3]         = {0.0};

    HAL_StatusTypeDef
        transceive(uint8_t *tx_buff, uint8_t *rx_buff, uint16_t size);

    HAL_StatusTypeDef write_reg(uint8_t reg, uint8_t value);
    int16_t           read_reg(uint8_t reg);
    HAL_StatusTypeDef write_ak8963_reg(uint8_t reg, uint8_t value);
    int16_t           read_ak8963_reg(uint8_t reg);

    void              set_read_only_mode(bool enable);
    HAL_StatusTypeDef read_3_axis(uint8_t reg, int16_t *xyz);

    HAL_StatusTypeDef reset_ak8963(void);
    HAL_StatusTypeDef get_ak8963_sensitivity(void);
    HAL_StatusTypeDef enable_ak8963_slave(bool enable);
};
}    // namespace lot
