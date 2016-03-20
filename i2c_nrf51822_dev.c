/*
 * Erl's generic i2c library, implementation for Nordic Semiconductor's nRF51822
 */

#include "nrf_drv_twi.h"

#include "i2c.h"
#include "i2c_nrf51822.h"

ret_code_t nrf51822_i2c_init( I2Cchannel i2cChannel,
			      nrf_twi_frequency_t frequency, 
			      uint8_t interruptPriority, 
			      uint32_t sclPin, uint32_t sdaPin,
			      nrf_drv_twi_evt_handler_t event_handler,
			      void *pContext )
{
  nrf_drv_twi_config_t config = { .frequency = frequency, 
                                  .interrupt_priority = interruptPriority,
                                  .scl = sclPin,
				  .sda = sdaPin };
  ret_code_t retCode;
  // nrf_drv_twi_t twi = NRF_DRV_TWI_INSTANCE( MASTER_TWI_INST );

  // i2cChannel->twi = twi;

  retCode = nrf_drv_twi_init( &(i2cChannel->twi), &config, event_handler,
			      pContext );

  if( retCode != NRF_SUCCESS )
    goto FAIL;

  nrf_drv_twi_enable( &(i2cChannel->twi) );

 FAIL:
  return retCode;
}

ret_code_t i2c_read( I2Cchannel i2cChannel, uint8_t address, uint32_t length, uint8_t *data )
{
  ret_code_t retCode;

  retCode = nrf_drv_twi_rx( &(i2cChannel->twi), address, data, length, 0 );

  return retCode;
}

ret_code_t i2c_write( I2Cchannel i2cChannel, uint8_t address, uint32_t length, uint8_t *data )
{
  ret_code_t retCode;

  retCode = nrf_drv_twi_tx( &(i2cChannel->twi), address, data, length, 0 );

  return retCode;
}
