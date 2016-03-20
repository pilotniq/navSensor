/*
  Erl's i2c generic interface
*/

#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include "sdk_errors.h" // for ret_code_t

// init function is chip specific
// void i2c_init(void);

typedef struct sI2Cchannel *I2Cchannel;

// TODO: Make error handling generic
ret_code_t i2c_read( I2Cchannel I2Cchannel, uint8_t address, uint32_t length, uint8_t *data );
ret_code_t i2c_write( I2Cchannel I2Cchannel, uint8_t address, uint32_t length, const uint8_t *data, bool stopAfter );

#endif

