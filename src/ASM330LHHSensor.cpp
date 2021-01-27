/**
 ******************************************************************************
 * @file    ASM330LHHSensor.cpp
 * @author  SRA
 * @version V1.0.0
 * @date    March 2020
 * @brief   Implementation of an ASM330LHH Automotive IMU 6 axes
 *          sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
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

#include "ASM330LHHSensor.h"


/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
ASM330LHHSensor::ASM330LHHSensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  dev_spi = NULL;
  reg_ctx.write_reg = ASM330LHH_io_write;
  reg_ctx.read_reg = ASM330LHH_io_read;
  reg_ctx.handle = (void *)this;
  acc_is_enabled = 0U;
  gyro_is_enabled = 0U;
}

/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param spi_speed the SPI speed
 */
ASM330LHHSensor::ASM330LHHSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), spi_speed(spi_speed)
{
  reg_ctx.write_reg = ASM330LHH_io_write;
  reg_ctx.read_reg = ASM330LHH_io_read;
  reg_ctx.handle = (void *)this;
  dev_i2c = NULL;
  address = 0U;  
  acc_is_enabled = 0U;
  gyro_is_enabled = 0U;
}

/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::begin()
{
  if(dev_spi)
  {
    // Configure CS pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH); 
  }

  /* Set DEVICE_CONF bit */
  if (asm330lhh_device_conf_set(&reg_ctx, PROPERTY_ENABLE) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  /* Enable register address automatically incremented during a multiple byte
  access with a serial interface. */
  if (asm330lhh_auto_increment_set(&reg_ctx, PROPERTY_ENABLE) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  /* Enable BDU */
  if (asm330lhh_block_data_update_set(&reg_ctx, PROPERTY_ENABLE) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  /* FIFO mode selection */
  if (asm330lhh_fifo_mode_set(&reg_ctx, ASM330LHH_BYPASS_MODE) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  /* Select default output data rate. */
  acc_odr = ASM330LHH_XL_ODR_104Hz;

  /* Output data rate selection - power down. */
  if (asm330lhh_xl_data_rate_set(&reg_ctx, ASM330LHH_XL_ODR_OFF) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  /* Full scale selection. */
  if (asm330lhh_xl_full_scale_set(&reg_ctx, ASM330LHH_2g) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  /* Select default output data rate. */
  gyro_odr = ASM330LHH_GY_ODR_104Hz;

  /* Output data rate selection - power down. */
  if (asm330lhh_gy_data_rate_set(&reg_ctx, ASM330LHH_GY_ODR_OFF) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  /* Full scale selection. */
  if (asm330lhh_gy_full_scale_set(&reg_ctx, ASM330LHH_2000dps) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }
  
  acc_is_enabled = 0;
  gyro_is_enabled = 0;

  return ASM330LHH_OK;
}

/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::end()
{
  /* Disable both acc and gyro */
  if (Disable_X() != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  if (Disable_G() != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  /* Reset CS configuration */
  if(dev_spi)
  {
    // Configure CS pin
    pinMode(cs_pin, INPUT); 
  }

  return ASM330LHH_OK;
}


/**
 * @brief  Read component ID
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::ReadID(uint8_t *Id)
{
  if (asm330lhh_device_id_get(&reg_ctx, Id) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  return ASM330LHH_OK;
}

/**
 * @brief  Enable the ASM330LHH accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Enable_X()
{
  /* Check if the component is already enabled */
  if (acc_is_enabled == 1U)
  {
    return ASM330LHH_OK;
  }

  /* Output data rate selection. */
  if (asm330lhh_xl_data_rate_set(&reg_ctx, acc_odr) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  acc_is_enabled = 1;

  return ASM330LHH_OK;
}

/**
 * @brief  Disable the ASM330LHH accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Disable_X()
{
  /* Check if the component is already disabled */
  if (acc_is_enabled == 0U)
  {
    return ASM330LHH_OK;
  }

  /* Get current output data rate. */
  if (asm330lhh_xl_data_rate_get(&reg_ctx, &acc_odr) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  /* Output data rate selection - power down. */
  if (asm330lhh_xl_data_rate_set(&reg_ctx, ASM330LHH_XL_ODR_OFF) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  acc_is_enabled = 0;

  return ASM330LHH_OK;
}

/**
 * @brief  Get the ASM330LHH accelerometer sensor sensitivity
 * @param  Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Get_X_Sensitivity(float *Sensitivity)
{
  ASM330LHHStatusTypeDef ret = ASM330LHH_OK;
  asm330lhh_fs_xl_t full_scale;

  /* Read actual full scale selection from sensor. */
  if (asm330lhh_xl_full_scale_get(&reg_ctx, &full_scale) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  /* Store the Sensitivity based on actual full scale. */
  switch (full_scale)
  {
    case ASM330LHH_2g:
      *Sensitivity = ASM330LHH_ACC_SENSITIVITY_FS_2G;
      break;

    case ASM330LHH_4g:
      *Sensitivity = ASM330LHH_ACC_SENSITIVITY_FS_4G;
      break;

    case ASM330LHH_8g:
      *Sensitivity = ASM330LHH_ACC_SENSITIVITY_FS_8G;
      break;

    case ASM330LHH_16g:
      *Sensitivity = ASM330LHH_ACC_SENSITIVITY_FS_16G;
      break;

    default:
      ret = ASM330LHH_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Get the ASM330LHH accelerometer sensor output data rate
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Get_X_ODR(float *Odr)
{
  ASM330LHHStatusTypeDef ret = ASM330LHH_OK;
  asm330lhh_odr_xl_t odr_low_level;

  /* Get current output data rate. */
  if (asm330lhh_xl_data_rate_get(&reg_ctx, &odr_low_level) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  switch (odr_low_level)
  {
    case ASM330LHH_XL_ODR_OFF:
      *Odr = 0.0f;
      break;

    case ASM330LHH_XL_ODR_12Hz5:
      *Odr = 12.5f;
      break;

    case ASM330LHH_XL_ODR_26Hz:
      *Odr = 26.0f;
      break;

    case ASM330LHH_XL_ODR_52Hz:
      *Odr = 52.0f;
      break;

    case ASM330LHH_XL_ODR_104Hz:
      *Odr = 104.0f;
      break;

    case ASM330LHH_XL_ODR_208Hz:
      *Odr = 208.0f;
      break;

    case ASM330LHH_XL_ODR_417Hz:
      *Odr = 417.0f;
      break;

    case ASM330LHH_XL_ODR_833Hz:
      *Odr = 833.0f;
      break;

    case ASM330LHH_XL_ODR_1667Hz:
      *Odr = 1667.0f;
      break;

    case ASM330LHH_XL_ODR_3333Hz:
      *Odr = 3333.0f;
      break;

    case ASM330LHH_XL_ODR_6667Hz:
      *Odr = 6667.0f;
      break;

    default:
      ret = ASM330LHH_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the ASM330LHH accelerometer sensor output data rate
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Set_X_ODR(float Odr)
{
  /* Check if the component is enabled */
  if (acc_is_enabled == 1U)
  {
    return Set_X_ODR_When_Enabled(Odr);
  }
  else
  {
    return Set_X_ODR_When_Disabled(Odr);
  }
}

/**
 * @brief  Set the ASM330LHH accelerometer sensor output data rate when enabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Set_X_ODR_When_Enabled(float Odr)
{
  asm330lhh_odr_xl_t new_odr;

  new_odr = (Odr <=   12.5f) ? ASM330LHH_XL_ODR_12Hz5
          : (Odr <=   26.0f) ? ASM330LHH_XL_ODR_26Hz
          : (Odr <=   52.0f) ? ASM330LHH_XL_ODR_52Hz
          : (Odr <=  104.0f) ? ASM330LHH_XL_ODR_104Hz
          : (Odr <=  208.0f) ? ASM330LHH_XL_ODR_208Hz
          : (Odr <=  417.0f) ? ASM330LHH_XL_ODR_417Hz
          : (Odr <=  833.0f) ? ASM330LHH_XL_ODR_833Hz
          : (Odr <= 1667.0f) ? ASM330LHH_XL_ODR_1667Hz
          : (Odr <= 3333.0f) ? ASM330LHH_XL_ODR_3333Hz
          :                    ASM330LHH_XL_ODR_6667Hz;

  /* Output data rate selection. */
  if (asm330lhh_xl_data_rate_set(&reg_ctx, new_odr) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  return ASM330LHH_OK;
}

/**
 * @brief  Set the ASM330LHH accelerometer sensor output data rate when disabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Set_X_ODR_When_Disabled(float Odr)
{
        acc_odr = (Odr <=   12.5f) ? ASM330LHH_XL_ODR_12Hz5
                : (Odr <=   26.0f) ? ASM330LHH_XL_ODR_26Hz
                : (Odr <=   52.0f) ? ASM330LHH_XL_ODR_52Hz
                : (Odr <=  104.0f) ? ASM330LHH_XL_ODR_104Hz
                : (Odr <=  208.0f) ? ASM330LHH_XL_ODR_208Hz
                : (Odr <=  417.0f) ? ASM330LHH_XL_ODR_417Hz
                : (Odr <=  833.0f) ? ASM330LHH_XL_ODR_833Hz
                : (Odr <= 1667.0f) ? ASM330LHH_XL_ODR_1667Hz
                : (Odr <= 3333.0f) ? ASM330LHH_XL_ODR_3333Hz
                :                    ASM330LHH_XL_ODR_6667Hz;

  return ASM330LHH_OK;
}


/**
 * @brief  Get the ASM330LHH accelerometer sensor full scale
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Get_X_FS(int32_t *FullScale)
{
  ASM330LHHStatusTypeDef ret = ASM330LHH_OK;
  asm330lhh_fs_xl_t fs_low_level;

  /* Read actual full scale selection from sensor. */
  if (asm330lhh_xl_full_scale_get(&reg_ctx, &fs_low_level) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  switch (fs_low_level)
  {
    case ASM330LHH_2g:
      *FullScale =  2;
      break;

    case ASM330LHH_4g:
      *FullScale =  4;
      break;

    case ASM330LHH_8g:
      *FullScale =  8;
      break;

    case ASM330LHH_16g:
      *FullScale = 16;
      break;

    default:
      ret = ASM330LHH_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the ASM330LHH accelerometer sensor full scale
 * @param  FullScale the functional full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Set_X_FS(int32_t FullScale)
{
  asm330lhh_fs_xl_t new_fs;

  /* Seems like MISRA C-2012 rule 14.3a violation but only from single file statical analysis point of view because
     the parameter passed to the function is not known at the moment of analysis */
  new_fs = (FullScale <= 2) ? ASM330LHH_2g
           : (FullScale <= 4) ? ASM330LHH_4g
           : (FullScale <= 8) ? ASM330LHH_8g
           :                    ASM330LHH_16g;

  if (asm330lhh_xl_full_scale_set(&reg_ctx, new_fs) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  return ASM330LHH_OK;
}

/**
 * @brief  Get the ASM330LHH accelerometer sensor raw axes
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Get_X_AxesRaw(int16_t *Value)
{
  axis3bit16_t data_raw;

  /* Read raw data values. */
  if (asm330lhh_acceleration_raw_get(&reg_ctx, data_raw.u8bit) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  /* Format the data. */
  Value[0] = data_raw.i16bit[0];
  Value[1] = data_raw.i16bit[1];
  Value[2] = data_raw.i16bit[2];

  return ASM330LHH_OK;
}


/**
 * @brief  Get the ASM330LHH accelerometer sensor axes
 * @param  Acceleration pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Get_X_Axes(int32_t *Acceleration)
{
  axis3bit16_t data_raw;
  float sensitivity = 0.0f;

  /* Read raw data values. */
  if (asm330lhh_acceleration_raw_get(&reg_ctx, data_raw.u8bit) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  /* Get ASM330LHH actual sensitivity. */
  if (Get_X_Sensitivity(&sensitivity) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  /* Calculate the data. */
  Acceleration[0] = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
  Acceleration[1] = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
  Acceleration[2] = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

  return ASM330LHH_OK;
}


/**
 * @brief  Get the ASM330LHH ACC data ready bit value
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Get_X_DRDY_Status(uint8_t *Status)
{
  if (asm330lhh_xl_flag_data_ready_get(&reg_ctx, Status) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  return ASM330LHH_OK;
}


/**
 * @brief  Enable the ASM330LHH gyroscope sensor
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Enable_G()
{
  /* Check if the component is already enabled */
  if (gyro_is_enabled == 1U)
  {
    return ASM330LHH_OK;
  }

  /* Output data rate selection. */
  if (asm330lhh_gy_data_rate_set(&reg_ctx, gyro_odr) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  gyro_is_enabled = 1;

  return ASM330LHH_OK;
}


/**
 * @brief  Disable the ASM330LHH gyroscope sensor
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Disable_G()
{
  /* Check if the component is already disabled */
  if (gyro_is_enabled == 0U)
  {
    return ASM330LHH_OK;
  }

  /* Get current output data rate. */
  if (asm330lhh_gy_data_rate_get(&reg_ctx, &gyro_odr) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  /* Output data rate selection - power down. */
  if (asm330lhh_gy_data_rate_set(&reg_ctx, ASM330LHH_GY_ODR_OFF) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  gyro_is_enabled = 0;

  return ASM330LHH_OK;
}

/**
 * @brief  Get the ASM330LHH gyroscope sensor sensitivity
 * @param  Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Get_G_Sensitivity(float *Sensitivity)
{
  ASM330LHHStatusTypeDef ret = ASM330LHH_OK;
  asm330lhh_fs_g_t full_scale;

  /* Read actual full scale selection from sensor. */
  if (asm330lhh_gy_full_scale_get(&reg_ctx, &full_scale) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch (full_scale)
  {
    case ASM330LHH_125dps:
      *Sensitivity = ASM330LHH_GYRO_SENSITIVITY_FS_125DPS;
      break;

    case ASM330LHH_250dps:
      *Sensitivity = ASM330LHH_GYRO_SENSITIVITY_FS_250DPS;
      break;

    case ASM330LHH_500dps:
      *Sensitivity = ASM330LHH_GYRO_SENSITIVITY_FS_500DPS;
      break;

    case ASM330LHH_1000dps:
      *Sensitivity = ASM330LHH_GYRO_SENSITIVITY_FS_1000DPS;
      break;

    case ASM330LHH_2000dps:
      *Sensitivity = ASM330LHH_GYRO_SENSITIVITY_FS_2000DPS;
      break;

    case ASM330LHH_4000dps:
      *Sensitivity = ASM330LHH_GYRO_SENSITIVITY_FS_4000DPS;
      break;

    default:
      ret = ASM330LHH_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Get the ASM330LHH gyroscope sensor output data rate
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Get_G_ODR(float *Odr)
{
  ASM330LHHStatusTypeDef ret = ASM330LHH_OK;
  asm330lhh_odr_g_t odr_low_level;

  /* Get current output data rate. */
  if (asm330lhh_gy_data_rate_get(&reg_ctx, &odr_low_level) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  switch (odr_low_level)
  {
    case ASM330LHH_GY_ODR_OFF:
      *Odr = 0.0f;
      break;

    case ASM330LHH_GY_ODR_12Hz5:
      *Odr = 12.5f;
      break;

    case ASM330LHH_GY_ODR_26Hz:
      *Odr = 26.0f;
      break;

    case ASM330LHH_GY_ODR_52Hz:
      *Odr = 52.0f;
      break;

    case ASM330LHH_GY_ODR_104Hz:
      *Odr = 104.0f;
      break;

    case ASM330LHH_GY_ODR_208Hz:
      *Odr = 208.0f;
      break;

    case ASM330LHH_GY_ODR_417Hz:
      *Odr = 417.0f;
      break;

    case ASM330LHH_GY_ODR_833Hz:
      *Odr = 833.0f;
      break;

    case ASM330LHH_GY_ODR_1667Hz:
      *Odr =  1667.0f;
      break;

    case ASM330LHH_GY_ODR_3333Hz:
      *Odr =  3333.0f;
      break;

    case ASM330LHH_GY_ODR_6667Hz:
      *Odr =  6667.0f;
      break;

    default:
      ret = ASM330LHH_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the ASM330LHH gyroscope sensor output data rate
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Set_G_ODR(float Odr)
{
  /* Check if the component is enabled */
  if (gyro_is_enabled == 1U)
  {
    return Set_G_ODR_When_Enabled(Odr);
  }
  else
  {
    return Set_G_ODR_When_Disabled(Odr);
  }
}

/**
 * @brief  Set the ASM330LHH gyroscope sensor output data rate when enabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Set_G_ODR_When_Enabled(float Odr)
{
  asm330lhh_odr_g_t new_odr;

  new_odr = (Odr <=   12.5f) ? ASM330LHH_GY_ODR_12Hz5
          : (Odr <=   26.0f) ? ASM330LHH_GY_ODR_26Hz
          : (Odr <=   52.0f) ? ASM330LHH_GY_ODR_52Hz
          : (Odr <=  104.0f) ? ASM330LHH_GY_ODR_104Hz
          : (Odr <=  208.0f) ? ASM330LHH_GY_ODR_208Hz
          : (Odr <=  417.0f) ? ASM330LHH_GY_ODR_417Hz
          : (Odr <=  833.0f) ? ASM330LHH_GY_ODR_833Hz
          : (Odr <= 1667.0f) ? ASM330LHH_GY_ODR_1667Hz
          : (Odr <= 3333.0f) ? ASM330LHH_GY_ODR_3333Hz
          :                    ASM330LHH_GY_ODR_6667Hz;

  /* Output data rate selection. */
  if (asm330lhh_gy_data_rate_set(&reg_ctx, new_odr) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  return ASM330LHH_OK;
}

/**
 * @brief  Set the ASM330LHH gyroscope sensor output data rate when disabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Set_G_ODR_When_Disabled(float Odr)
{
  gyro_odr = (Odr <=   12.5f) ? ASM330LHH_GY_ODR_12Hz5
                 : (Odr <=   26.0f) ? ASM330LHH_GY_ODR_26Hz
                 : (Odr <=   52.0f) ? ASM330LHH_GY_ODR_52Hz
                 : (Odr <=  104.0f) ? ASM330LHH_GY_ODR_104Hz
                 : (Odr <=  208.0f) ? ASM330LHH_GY_ODR_208Hz
                 : (Odr <=  417.0f) ? ASM330LHH_GY_ODR_417Hz
                 : (Odr <=  833.0f) ? ASM330LHH_GY_ODR_833Hz
                 : (Odr <= 1667.0f) ? ASM330LHH_GY_ODR_1667Hz
                 : (Odr <= 3333.0f) ? ASM330LHH_GY_ODR_3333Hz
                 :                    ASM330LHH_GY_ODR_6667Hz;

  return ASM330LHH_OK;
}


/**
 * @brief  Get the ASM330LHH gyroscope sensor full scale
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Get_G_FS(int32_t  *FullScale)
{
  ASM330LHHStatusTypeDef ret = ASM330LHH_OK;
  asm330lhh_fs_g_t fs_low_level;

  /* Read actual full scale selection from sensor. */
  if (asm330lhh_gy_full_scale_get(&reg_ctx, &fs_low_level) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  switch (fs_low_level)
  {
    case ASM330LHH_125dps:
      *FullScale =  125;
      break;

    case ASM330LHH_250dps:
      *FullScale =  250;
      break;

    case ASM330LHH_500dps:
      *FullScale =  500;
      break;

    case ASM330LHH_1000dps:
      *FullScale = 1000;
      break;

    case ASM330LHH_2000dps:
      *FullScale = 2000;
      break;

    case ASM330LHH_4000dps:
      *FullScale = 4000;
      break;

    default:
      ret = ASM330LHH_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the ASM330LHH gyroscope sensor full scale
 * @param  FullScale the functional full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Set_G_FS(int32_t FullScale)
{
  asm330lhh_fs_g_t new_fs;

  new_fs = (FullScale <= 125)  ? ASM330LHH_125dps
           : (FullScale <= 250)  ? ASM330LHH_250dps
           : (FullScale <= 500)  ? ASM330LHH_500dps
           : (FullScale <= 1000) ? ASM330LHH_1000dps
           : (FullScale <= 2000) ? ASM330LHH_2000dps
           :                       ASM330LHH_4000dps;

  if (asm330lhh_gy_full_scale_set(&reg_ctx, new_fs) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  return ASM330LHH_OK;
}

/**
 * @brief  Get the ASM330LHH gyroscope sensor raw axes
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Get_G_AxesRaw(int16_t *Value)
{
  axis3bit16_t data_raw;

  /* Read raw data values. */
  if (asm330lhh_angular_rate_raw_get(&reg_ctx, data_raw.u8bit) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  /* Format the data. */
  Value[0] = data_raw.i16bit[0];
  Value[1] = data_raw.i16bit[1];
  Value[2] = data_raw.i16bit[2];

  return ASM330LHH_OK;
}


/**
 * @brief  Get the ASM330LHH gyroscope sensor axes
 * @param  AngularRate pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Get_G_Axes(int32_t *AngularRate)
{
  axis3bit16_t data_raw;
  float sensitivity;

  /* Read raw data values. */
  if (asm330lhh_angular_rate_raw_get(&reg_ctx, data_raw.u8bit) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  /* Get ASM330LHH actual sensitivity. */
  if (Get_G_Sensitivity(&sensitivity) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  /* Calculate the data. */
  AngularRate[0] = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
  AngularRate[1] = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
  AngularRate[2] = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

  return ASM330LHH_OK;
}


/**
 * @brief  Get the ASM330LHH GYRO data ready bit value
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Get_G_DRDY_Status(uint8_t *Status)
{
  if (asm330lhh_gy_flag_data_ready_get(&reg_ctx, Status) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  return ASM330LHH_OK;
}


/**
 * @brief  Get the ASM330LHH register value
 * @param  Reg address to be read
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Read_Reg(uint8_t Reg, uint8_t *Data)
{
  if (asm330lhh_read_reg(&reg_ctx, Reg, Data, 1) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  return ASM330LHH_OK;
}


/**
 * @brief  Set the ASM330LHH register value
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
ASM330LHHStatusTypeDef ASM330LHHSensor::Write_Reg(uint8_t Reg, uint8_t Data)
{
  if (asm330lhh_write_reg(&reg_ctx, Reg, &Data, 1) != ASM330LHH_OK)
  {
    return ASM330LHH_ERROR;
  }

  return ASM330LHH_OK;
}


int32_t ASM330LHH_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  return ((ASM330LHHSensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}


int32_t ASM330LHH_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  return ((ASM330LHHSensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
