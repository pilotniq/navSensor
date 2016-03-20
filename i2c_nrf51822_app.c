#include <string.h>  // for memcpy
#include "i2c_nrf51822_app.h"

#include "i2c.h"

#define QUEUE_SIZE 15

// typedef sI2Cchannel app_twi_t;

// static app_twi_t app_twi;
static app_twi_transaction_t const *queue_buffer[ QUEUE_SIZE + 1 ];

ret_code_t nrf51822_app_i2c_init( I2Cchannel i2cChannel,
				  nrf_twi_frequency_t frequency, 
				  uint8_t interruptPriority, 
				  uint32_t sclPin, uint32_t sdaPin )
{
  nrf_drv_twi_config_t twi_config;
  app_twi_t template = APP_TWI_INSTANCE( 0 );

  // clean up later
  memcpy( &(i2cChannel->channel), &template, sizeof( template ) );
  // i2cChannel->channel = template;

  twi_config.frequency = frequency;
  twi_config.interrupt_priority = interruptPriority;
  twi_config.scl = sclPin;
  twi_config.sda = sdaPin;

  // i2cChannel->channel.twi = NRF_DRV_TWI_INSTANCE( 0 );

  return app_twi_init( &(i2cChannel->channel), &twi_config, QUEUE_SIZE, queue_buffer );
}

ret_code_t i2c_write( I2Cchannel i2cChannel, uint8_t address, uint32_t length, const uint8_t *data, bool stopAfter )
{
  app_twi_transfer_t transfer;

  transfer.p_data = (uint8_t *) data;
  transfer.length = length;
  transfer.operation = APP_TWI_WRITE_OP( address );
  transfer.flags = stopAfter ? 0 : APP_TWI_NO_STOP;

  return app_twi_perform( &(i2cChannel->channel), &transfer, 1, NULL );
}

ret_code_t i2c_read( I2Cchannel i2cChannel, uint8_t address, uint32_t length, uint8_t *data )
{
  app_twi_transfer_t transfer;

  transfer.p_data = data;
  transfer.length = length;
  transfer.operation = APP_TWI_READ_OP( address );
  transfer.flags = 0; // stopAfter ? 0 : APP_TWI_NO_STOP;

  return app_twi_perform( &(i2cChannel->channel), &transfer, 1, NULL );
}
