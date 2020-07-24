/*
    __temphum9_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __temphum9_driver.h
@brief    Temp_Hum_9 Driver
@mainpage Temp_Hum_9 Click
@{

@image html libstock_fb_view.jpg

@}

@defgroup   TEMPHUM9
@brief      Temp_Hum_9 Click Driver
@{

| Global Library Prefix | **TEMPHUM9** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **Nov 2018.**      |
| Developer             | **Aleksandar Paunovic**     |

*/
/* -------------------------------------------------------------------------- */

#include "stdint.h"

#ifndef _TEMPHUM9_H_
#define _TEMPHUM9_H_

#include "shtc3.h"

/** 
 * @macro T_TEMPHUM9_P
 * @brief Driver Abstract type 
 */
#define T_TEMPHUM9_P    const uint8_t*

/** @defgroup TEMPHUM9_COMPILE Compilation Config */              /** @{ */

// #define   __TEMPHUM9_DRV_SPI__                            /**<     @macro __TEMPHUM9_DRV_SPI__  @brief SPI driver selector */
   #define   __TEMPHUM9_DRV_I2C__                            /**<     @macro __TEMPHUM9_DRV_I2C__  @brief I2C driver selector */                                          
// #define   __TEMPHUM9_DRV_UART__                           /**<     @macro __TEMPHUM9_DRV_UART__ @brief UART driver selector */ 

                                                                       /** @} */
/** @defgroup TEMPHUM9_VAR Variables */                           /** @{ */

/* device address */
extern const uint8_t _TEMPHUM9_DEVICE_ADDRESS;

/* measurement modes */
extern const uint8_t _TEMPHUM9_NORMAL_MODE;
extern const uint8_t _TEMPHUM9_LOW_POWER_MODE;

/* data */
extern const uint8_t _TEMPHUM9_SINGLE_DATA;
extern const uint8_t _TEMPHUM9_DUAL_DATA;

/* commands */
extern const uint16_t _TEMPHUM9_SLEEP;
extern const uint16_t _TEMPHUM9_WAKEUP;
extern const uint16_t _TEMPHUM9_SOFT_RESET;
extern const uint16_t _TEMPHUM9_GENERAL_CALL_RESET;
extern const uint16_t _TEMPHUM9_READ_ID;

                                                                       /** @} */
/** @defgroup TEMPHUM9_TYPES Types */                             /** @{ */



                                                                       /** @} */
#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup TEMPHUM9_INIT Driver Initialization */              /** @{ */

#ifdef   __TEMPHUM9_DRV_SPI__
void temphum9_spiDriverInit(T_TEMPHUM9_P gpioObj, T_TEMPHUM9_P spiObj);
#endif
#ifdef   __TEMPHUM9_DRV_I2C__
void temphum9_i2cDriverInit(T_TEMPHUM9_P gpioObj, T_TEMPHUM9_P i2cObj, uint8_t slave);
#endif
#ifdef   __TEMPHUM9_DRV_UART__
void temphum9_uartDriverInit(T_TEMPHUM9_P gpioObj, T_TEMPHUM9_P uartObj);
#endif

                                                                       /** @} */
/** @defgroup TEMPHUM9_FUNC Driver Functions */                   /** @{ */


/**
 * @brief Readinig register content
 *
 * @param[in] uint16_t registerAddress_  - address of a register to read (command)
 * @param[in] uint8_t nData_             - single for one 16-bit register or dual for two 16-bit registers
 * @param[out] uint16_t registerBuffer_[] - read data is stored into this array
 *
 * This function reads one or two 16-bit registers
 */
void temphum9_readRegister( struct device *dev, uint16_t registerAddress_, uint8_t nData_, uint16_t *registerBuffer_ );

/**
 * @brief Issuing a command
 *
 * @param[in] uint16_t command_ - command to be sent to device
 *
 * This function issues (sends) command to device
 */
void temphum9_sendCommand( struct device *dev, uint16_t command_ );

/**
 * @brief Calculating temperature
 *
 * @param[in] uint8_t mode_ - mode to be used for measurement (normal or low power mode)
 *
 * This function performs temperature measurement and calculates temperature
 */
float temphum9_getTemperature( struct device *dev, uint8_t mode_ );

/**
 * @brief Calculating relative humidity
 *
 * @param[in] uint8_t mode_ - mode to be used for measurement (normal or low power mode)
 *
 * This function performs relative humidity measurement and calculates relative humidity
 */
float temphum9_getRelativeHumidity( struct device *dev, uint8_t mode_ );

/**
 * @brief Calculating temperature and relative humidity
 *
 * @param[in]  uint8_t mode_ - mode to be used for measurement (normal or low power mode)
 * @param[out] float measurementData_[] - measurend data is stored into this array
 *
 * This function performs temperature and relative humidity measurement and calculates temperature and relative humidity
 */
void temhum9_getTemperatureAndHumidity( struct device *dev, uint8_t mode_, float *measurementData);


                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/**
    @example Click_Temp_Hum_9_STM.c
    @example Click_Temp_Hum_9_TIVA.c
    @example Click_Temp_Hum_9_CEC.c
    @example Click_Temp_Hum_9_KINETIS.c
    @example Click_Temp_Hum_9_MSP.c
    @example Click_Temp_Hum_9_PIC.c
    @example Click_Temp_Hum_9_PIC32.c
    @example Click_Temp_Hum_9_DSPIC.c
    @example Click_Temp_Hum_9_AVR.c
    @example Click_Temp_Hum_9_FT90x.c
    @example Click_Temp_Hum_9_STM.mbas
    @example Click_Temp_Hum_9_TIVA.mbas
    @example Click_Temp_Hum_9_CEC.mbas
    @example Click_Temp_Hum_9_KINETIS.mbas
    @example Click_Temp_Hum_9_MSP.mbas
    @example Click_Temp_Hum_9_PIC.mbas
    @example Click_Temp_Hum_9_PIC32.mbas
    @example Click_Temp_Hum_9_DSPIC.mbas
    @example Click_Temp_Hum_9_AVR.mbas
    @example Click_Temp_Hum_9_FT90x.mbas
    @example Click_Temp_Hum_9_STM.mpas
    @example Click_Temp_Hum_9_TIVA.mpas
    @example Click_Temp_Hum_9_CEC.mpas
    @example Click_Temp_Hum_9_KINETIS.mpas
    @example Click_Temp_Hum_9_MSP.mpas
    @example Click_Temp_Hum_9_PIC.mpas
    @example Click_Temp_Hum_9_PIC32.mpas
    @example Click_Temp_Hum_9_DSPIC.mpas
    @example Click_Temp_Hum_9_AVR.mpas
    @example Click_Temp_Hum_9_FT90x.mpas
*/                                                                     /** @} */
/* -------------------------------------------------------------------------- */
/*
  __temphum9_driver.h

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the MikroElektonika.

4. Neither the name of the MikroElektonika nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------- */