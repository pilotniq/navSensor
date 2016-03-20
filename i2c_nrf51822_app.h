/*
 * Erl's generic i2c library, implementation for Nordic Semiconductor's nRF51822
 */

#ifndef I2C_NRF51822_H
#define I2C_NRF51822_H

#include "app_twi.h"

#include "i2c.h"

typedef struct sI2Cchannel
{ 
  app_twi_t channel;
} sI2Cchannel;

ret_code_t nrf51822_app_i2c_init( I2Cchannel i2cChannel,
				  nrf_twi_frequency_t frequency, 
				  uint8_t interruptPriority, 
				  uint32_t sclPin, uint32_t sdaPin );
				

#endif
