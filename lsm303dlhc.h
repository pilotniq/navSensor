/*
 * lsm303dlhc.h - driver for LSM303DLHC chip
 */

#ifndef LSM303DLHC_H 
#define LSM303DLHC_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { ACC_RATE_POWER_DOWN = 0,
               ACC_RATE_1_HZ = 1,
               ACC_RATE_10_HZ = 2,
               ACC_RATE_25_HZ = 3,
               ACC_RATE_50_HZ = 4,
               ACC_RATE_100_HZ = 5,
               ACC_RATE_200_HZ = 6,
               ACC_RATE_400_HZ = 7,
               ACC_RATE_LOW_POWER_1620KHZ = 8,
               ACC_RATE_LOW_POWER_5376KHZ = 9
} LSM303DLHC_Acc_DataRate;

typedef enum { LSM303DLHC_MAG_MODE_CONTINUOUS = 0,
               LSM303DLHC_MAG_MODE_SINGLE = 1,
               LSM303DLHC_MAG_MODE_SLEEP1 = 2,
               LSM303DLHC_MAG_MODE_SLEEP2 = 3
} LSM303DLHC_mag_mode;
	       
/*
 * Start of functions
 */
void lsm303dlhc_init( I2Cchannel i2c );
void lsm303dlhc_acc_setDataRate( LSM303DLHC_Acc_DataRate dataRate );
void lsm303dlhc_acc_read( int16_t *acc );
void lsm303dlhc_mag_setBoot( bool boot );
void lsm303dlhc_mag_setMode( LSM303DLHC_mag_mode magMode );
void lsm303dlhc_mag_read( int16_t *mag );

/* Returns 0=north, 16384 = east ... */
uint16_t lsm303dlhc_tilt_compensate( int16_t *acc, int16_t *mag );
#endif
