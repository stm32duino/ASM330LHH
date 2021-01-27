/**
 ******************************************************************************
 * @file    ASM330LHHSensor.h
 * @author  SRA
 * @version V1.0.0
 * @date    March 2020
 * @brief   Abstract Class of an ASM330LHH Automotive IMU 6 axes
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


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __ASM330LHHSensor_H__
#define __ASM330LHHSensor_H__


/* Includes ------------------------------------------------------------------*/

#include "Wire.h"
#include "SPI.h"
#include "asm330lhh_reg.h"

/* Defines -------------------------------------------------------------------*/

#define ASM330LHH_ACC_SENSITIVITY_FS_2G   0.061f
#define ASM330LHH_ACC_SENSITIVITY_FS_4G   0.122f
#define ASM330LHH_ACC_SENSITIVITY_FS_8G   0.244f
#define ASM330LHH_ACC_SENSITIVITY_FS_16G  0.488f

#define ASM330LHH_GYRO_SENSITIVITY_FS_125DPS    4.370f
#define ASM330LHH_GYRO_SENSITIVITY_FS_250DPS    8.750f
#define ASM330LHH_GYRO_SENSITIVITY_FS_500DPS   17.500f
#define ASM330LHH_GYRO_SENSITIVITY_FS_1000DPS  35.000f
#define ASM330LHH_GYRO_SENSITIVITY_FS_2000DPS  70.000f
#define ASM330LHH_GYRO_SENSITIVITY_FS_4000DPS 140.000f


/* Typedefs ------------------------------------------------------------------*/

typedef enum
{
  ASM330LHH_OK = 0,
  ASM330LHH_ERROR =-1
} ASM330LHHStatusTypeDef;


/* Class Declaration ---------------------------------------------------------*/
   
/**
 * Abstract class of an ASM330LHH Inertial Measurement Unit (IMU) 3 axes
 * sensor.
 */
class ASM330LHHSensor
{
  public:
    ASM330LHHSensor(TwoWire *i2c, uint8_t address=ASM330LHH_I2C_ADD_H);
    ASM330LHHSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed=2000000);
    ASM330LHHStatusTypeDef begin();
    ASM330LHHStatusTypeDef end();
    ASM330LHHStatusTypeDef ReadID(uint8_t *Id);
    ASM330LHHStatusTypeDef Enable_X();
    ASM330LHHStatusTypeDef Disable_X();
    ASM330LHHStatusTypeDef Get_X_Sensitivity(float *Sensitivity);
    ASM330LHHStatusTypeDef Get_X_ODR(float *Odr);
    ASM330LHHStatusTypeDef Set_X_ODR(float Odr);
    ASM330LHHStatusTypeDef Get_X_FS(int32_t *FullScale);
    ASM330LHHStatusTypeDef Set_X_FS(int32_t FullScale);
    ASM330LHHStatusTypeDef Get_X_AxesRaw(int16_t *Value);
    ASM330LHHStatusTypeDef Get_X_Axes(int32_t *Acceleration);
    ASM330LHHStatusTypeDef Get_X_DRDY_Status(uint8_t *Status);
    
    ASM330LHHStatusTypeDef Enable_G();
    ASM330LHHStatusTypeDef Disable_G();
    ASM330LHHStatusTypeDef Get_G_Sensitivity(float *Sensitivity);
    ASM330LHHStatusTypeDef Get_G_ODR(float *Odr);
    ASM330LHHStatusTypeDef Set_G_ODR(float Odr);
    ASM330LHHStatusTypeDef Get_G_FS(int32_t *FullScale);
    ASM330LHHStatusTypeDef Set_G_FS(int32_t FullScale);
    ASM330LHHStatusTypeDef Get_G_AxesRaw(int16_t *Value);
    ASM330LHHStatusTypeDef Get_G_Axes(int32_t *AngularRate);
    ASM330LHHStatusTypeDef Get_G_DRDY_Status(uint8_t *Status);
    
    ASM330LHHStatusTypeDef Read_Reg(uint8_t reg, uint8_t *Data);
    ASM330LHHStatusTypeDef Write_Reg(uint8_t reg, uint8_t Data);
    
    /**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {        
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

        digitalWrite(cs_pin, LOW);

        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr | 0x80);
        /* Read the data */
        for (uint16_t i=0; i<NumByteToRead; i++) {
          *(pBuffer+i) = dev_spi->transfer(0x00);
        }
         
        digitalWrite(cs_pin, HIGH);

        dev_spi->endTransaction();

        return 0;
      }
		
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
        dev_i2c->write(RegisterAddr);
        dev_i2c->endTransmission(false);

        dev_i2c->requestFrom(((uint8_t)(((address) >> 1) & 0x7F)), (uint8_t) NumByteToRead);

        int i=0;
        while (dev_i2c->available()) {
          pBuffer[i] = dev_i2c->read();
          i++;
        }

        return 0;
      }

      return 1;
    }
    
    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {  
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

        digitalWrite(cs_pin, LOW);

        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr);
        /* Write the data */
        for (uint16_t i=0; i<NumByteToWrite; i++) {
          dev_spi->transfer(pBuffer[i]);
        }

        digitalWrite(cs_pin, HIGH);

        dev_spi->endTransaction();

        return 0;                    
      }
  
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));

        dev_i2c->write(RegisterAddr);
        for (uint16_t i = 0 ; i < NumByteToWrite ; i++) {
          dev_i2c->write(pBuffer[i]);
        }

        dev_i2c->endTransmission(true);

        return 0;
      }

      return 1;
    }

  private:
  
    ASM330LHHStatusTypeDef Set_X_ODR_When_Enabled(float Odr);
    ASM330LHHStatusTypeDef Set_X_ODR_When_Disabled(float Odr);
    ASM330LHHStatusTypeDef Set_G_ODR_When_Enabled(float Odr);
    ASM330LHHStatusTypeDef Set_G_ODR_When_Disabled(float Odr);

    /* Helper classes. */
    TwoWire *dev_i2c;
    SPIClass *dev_spi;
    
    /* Configuration */
    uint8_t address;
    int cs_pin;
    uint32_t spi_speed;
    
    asm330lhh_odr_xl_t acc_odr;
    asm330lhh_odr_g_t gyro_odr;
    
    uint8_t acc_is_enabled;
    uint8_t gyro_is_enabled;
       
    asm330lhh_ctx_t reg_ctx;  
};

#ifdef __cplusplus
 extern "C" {
#endif
int32_t ASM330LHH_io_write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
int32_t ASM330LHH_io_read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
#ifdef __cplusplus
  }
#endif

#endif
