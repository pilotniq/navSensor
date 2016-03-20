/*
 * Driver for LSM303DLHC Compass and Accellerometer IC
 * 
 * Uses i2c bus
 */

#define ACC_ADDRESS 0b0011001
#define MAG_ADDRESS 0b0011110

// #define OUT_X_L_M   0x16
#define OUT_X_H_M   0x03
#define CTRL_REG1_A 0x20
#define CTRL_REG2_A 0x21
#define OUT_X_L_A   0x28

#define MR_REG_M  0x02


#include <stdbool.h>
#include <stdint.h>
#include <math.H>

#include "i2c.h"

#include "lsm303dlhc.h"

struct sLSM303DLHC 
{
  I2Cchannel i2cChannel;
  LSM303DLHC_Acc_DataRate dataRate;
  bool rebootMemory;
  bool lowPowerMode;
  bool xEn;
  bool yEn;
  bool zEn;
  LSM303DLHC_mag_mode magMode;
} sDevice; // sLSM303DLHC, *LSM303DLHC;

/*
 * static function prototypes
 */
static void writeReg1A();
static void writeReg_MR_REG_M();
static void writeAccReg( int reg, uint8_t value );
static void writeMagReg( int reg, uint8_t value );

// static uint32_t SquareRoot32(uint32_t a_nInput);
static uint64_t SquareRoot64(uint64_t a_nInput);
// static void vector3_cross( int16_t *v1, int16_t *v2, int32_t *v3 );
// static void vector3_normalize( int16_t *v );
static inline int16_t s16_nabs(const int16_t j);
static inline int16_t q15_div(const int16_t numer, const int16_t denom);
static inline int16_t q15_from_double(const double d);
static inline int16_t q15_mul(const int16_t j, const int16_t k);
static uint16_t fxpt_atan2(const int16_t y, const int16_t x);

/*
 * Start of code
 */
void lsm303dlhc_init( I2Cchannel i2c )
{
  // initialize to device defaults
  sDevice.i2cChannel = i2c;
  sDevice.dataRate = ACC_RATE_POWER_DOWN;
  sDevice.lowPowerMode = false;
  sDevice.zEn = true;
  sDevice.xEn = true;
  sDevice.yEn = true;
  sDevice.magMode = LSM303DLHC_MAG_MODE_SLEEP2;
}

void lsm303dlhc_acc_setDataRate( LSM303DLHC_Acc_DataRate dataRate )
{
  sDevice.dataRate = dataRate;

  writeReg1A();
}
/*
void lsm303dlhc_mag_setBoot( bool boot )
{
  sDevice.rebootMemory = true;

  writeReg5A();
}
*/
void lsm303dlhc_mag_setMode( LSM303DLHC_mag_mode magMode )
{
  sDevice.magMode = magMode;

  writeReg_MR_REG_M();
}

static void writeReg1A()
{
  writeAccReg( CTRL_REG1_A, 
	       (sDevice.dataRate << 4) | 
	       ((sDevice.lowPowerMode ? 1 : 0) << 3) |
	       ((sDevice.zEn ? 1 : 0) << 2) |
	       ((sDevice.yEn ? 1 : 0) << 1) |
	       (sDevice.xEn ? 1 : 0) );
}

static void writeReg_MR_REG_M()
{
  writeMagReg( MR_REG_M, sDevice.magMode );
}

static void writeAccReg( int reg, uint8_t value )
{
  uint8_t values[ 2 ];
  ret_code_t retCode;

  values[0] = reg;
  values[1] = value;

  retCode = i2c_write( sDevice.i2cChannel, ACC_ADDRESS, sizeof( values ), values, true );
  retCode = retCode;

}

static void writeMagReg( int reg, uint8_t value )
{
  uint8_t values[ 2 ];

  values[0] = reg;
  values[1] = value;

  i2c_write( sDevice.i2cChannel, MAG_ADDRESS, sizeof( values ), values, true );
}

// param: array of 3 int16_t
void lsm303dlhc_acc_read( int16_t *acc )
{
  uint8_t reg = OUT_X_L_A | (1 << 7 );
  uint8_t buffer[6];

  i2c_write( sDevice.i2cChannel, ACC_ADDRESS, 1, &reg, false );
  // may need endianness-fix
  i2c_read( sDevice.i2cChannel, ACC_ADDRESS, 6, (uint8_t *) buffer );

  acc[0] = ((uint16_t) buffer[1]) << 8 | buffer[0];
  acc[1] = ((uint16_t) buffer[3]) << 8 | buffer[2];
  acc[2] = ((uint16_t) buffer[5]) << 8 | buffer[4];
}

// param: array of 3 int16_t
void lsm303dlhc_mag_read( int16_t *mag )
{
  uint8_t reg = OUT_X_H_M | (1 << 7 );
  uint8_t buffer[6];

  i2c_write( sDevice.i2cChannel, MAG_ADDRESS, 1, &reg, false );
  // may need endianness-fix
  i2c_read( sDevice.i2cChannel, MAG_ADDRESS, 6, buffer );

  mag[0] = ((uint16_t) buffer[0]) << 8 | buffer[1];
  mag[2] = ((uint16_t) buffer[2]) << 8 | buffer[3];
  mag[1] = ((uint16_t) buffer[4]) << 8 | buffer[5];
}
/*
static void vector3_cross( int16_t *v1, int16_t *v2, int32_t *v3 )
{
  // can we get overflows here?
  // I'm going out on a limb here, but I'll shift the results down 16 bits.
  v3[0] = ( (int32_t) v1[1]) * v2[2] - ((int32_t) v1[2]) * v2[1];
  v3[1] = ( (int32_t) v1[2]) * v2[0] - ((int32_t) v1[0]) * v2[2];
  v3[2] = ( (int32_t) v1[0]) * v2[1] - ((int32_t) v1[1]) * v2[0];
}
*/
static void vector3_normCross( int16_t *v1, int16_t *v2, int16_t *v3 )
{
  int32_t cross[3];
  int64_t sqsum;
  int32_t sq;

  // can we get overflows here?
  // I'm going out on a limb here, but I'll shift the results down 16 bits.
  cross[0] = ( (int32_t) v1[1]) * v2[2] - ((int32_t) v1[2]) * v2[1];
  cross[1] = ( (int32_t) v1[2]) * v2[0] - ((int32_t) v1[0]) * v2[2];
  cross[2] = ( (int32_t) v1[0]) * v2[1] - ((int32_t) v1[1]) * v2[0];

  sqsum = ((((int64_t) cross[0]) * cross[0]) >> 2) + 
          ((((int64_t) cross[1]) * cross[1]) >> 2) +
          ((((int64_t) cross[2]) * cross[2]) >> 2);

  sq = SquareRoot64( sqsum );
  
  // 32 bit number * 16 bit number / 32 bit number 
  v3[0] = (((int64_t) cross[0]) * INT16_MAX/2) / sq;
  v3[1] = (((int64_t) cross[1]) * INT16_MAX/2) / sq;
  v3[2] = (((int64_t) cross[2]) * INT16_MAX/2) / sq;
}
/*
static void vector3_normalize( int16_t *v )
{
  int32_t sqsum = ((((int32_t) v[0]) * v[0]) << 2) + 
    ((((int32_t) v[1]) * v[1]) << 2) + 
    ((((int32_t) v[2]) * v[2]) << 2);
  uint16_t mag = SquareRoot32( sqsum );

  // sq will be sqrt( sum of squares ) divided by two.

  v[0] = (((int32_t) v[0]) * INT16_MAX / 2) / mag;
  v[1] = (((int32_t) v[1]) * INT16_MAX / 2) / mag;
  v[2] = (((int32_t) v[2]) * INT16_MAX / 2) / mag;
}
*/
#if 0
/**
 * from: http://stackoverflow.com/questions/1100090/looking-for-an-efficient-integer-square-root-algorithm-for-arm-thumb2
 *
 * \brief    Fast Square root algorithm
 *
 * Fractional parts of the answer are discarded. That is:
 *      - SquareRoot(3) --> 1
 *      - SquareRoot(4) --> 2
 *      - SquareRoot(5) --> 2
 *      - SquareRoot(8) --> 2
 *      - SquareRoot(9) --> 3
 *
 * \param[in] a_nInput - unsigned integer for which to find the square root
 *
 * \return Integer square root of the input value.
 */
static uint32_t SquareRoot32(uint32_t a_nInput)
{
  uint32_t op  = a_nInput;
  uint32_t res = 0;
  uint32_t one = 1uL << 30; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type

  // "one" starts at the highest power of four <= than the argument.
  while (one > op)
    {
      one >>= 2;
    }

  while (one != 0)
    {
      if (op >= res + one)
        {
	  op = op - (res + one);
	  res = res +  2 * one;
        }
      res >>= 1;
      one >>= 2;
    }
  return res;
}
#endif
/**
 * from: http://stackoverflow.com/questions/1100090/looking-for-an-efficient-integer-square-root-algorithm-for-arm-thumb2
 *
 * \brief    Fast Square root algorithm
 *
 * Fractional parts of the answer are discarded. That is:
 *      - SquareRoot(3) --> 1
 *      - SquareRoot(4) --> 2
 *      - SquareRoot(5) --> 2
 *      - SquareRoot(8) --> 2
 *      - SquareRoot(9) --> 3
 *
 * \param[in] a_nInput - unsigned integer for which to find the square root
 *
 * \return Integer square root of the input value.
 */
static uint64_t SquareRoot64(uint64_t a_nInput)
{
  uint64_t op  = a_nInput;
  uint64_t res = 0;
  uint64_t one = ((uint64_t) 1) << 62; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type

  // "one" starts at the highest power of four <= than the argument.
  while (one > op)
    {
      one >>= 2;
    }

  while (one != 0)
    {
      if (op >= res + one)
        {
	  op = op - (res + one);
	  res = res +  2 * one;
        }
      res >>= 1;
      one >>= 2;
    }
  return res;
}

static int16_t vector_dot( int16_t *v1, int16_t *v2 )
{
  int32_t result;

  // result is dot product divided by 4. range is up to 32 bit integer
  result = ((int32_t) v1[0]) * v2[0] / 4 + ( (int32_t) v1[1] * v2[1]) / 4 + ((int32_t) v1[2]) * v2[2] / 4;

  // scale result back
  return (result >> 14);
}

uint16_t lsm303dlhc_tilt_compensate( int16_t *acc, int16_t *mag )
{
  int16_t e[3], n[3], from[3];

  // essentially work with 16 bit signed fixed point here
  from[0] = 0;
  from[1] = INT16_MIN;
  from[2] = 0;
  // heading((vector<int>){0, -1, 0});

// do calibration stuff here, NIY (subtract min from max to find center for each dim)
 vector3_normCross( mag, acc, e );
 // vector3_normalize( e );
 vector3_normCross( acc, e, n );
 // vector3_normalize( n );

 int16_t a = vector_dot( e, from );
 int16_t b = vector_dot( n, from );

 // heading = atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / M_PI;
 // if (heading < 0) heading += 360;
 // return heading;

 uint16_t heading = fxpt_atan2( a, b );
 return heading;
 /*
 if( heading < 0 )
   return heading + INT16_MAX;
 else
   return (uint16_t) heading;
 */
}

static inline int16_t s16_nabs(const int16_t j) {
#if (((int16_t)-1) >> 1) == ((int16_t)-1)
  // signed right shift sign-extends (arithmetic)
  const int16_t negSign = ~(j >> 15); // splat sign bit into all 16 and complement
  // if j is positive (negSign is -1), xor will invert j and sub will add 1
  // otherwise j is unchanged
  return (j ^ negSign) - negSign;
#else
  return (j < 0 ? j : -j);
#endif
}

/**
 * Q15 (1.0.15 fixed point) division (non-saturating). Be careful when using
 * this function, as it does not behave well when the result is out-of-range.
 *
 * Value is not defined if numerator is greater than or equal to denominator.
 *
 * @param numer 16-bit signed integer representing -1 to (1 - (2**-15))
 * @param denom same format as numer; must be greater than numerator
 * @return numer / denom in same format as numer and denom
 */
static inline int16_t q15_div(const int16_t numer, const int16_t denom) {
  return ((int32_t)numer << 15) / denom;
}

/**
 * Convert floating point to Q15 (1.0.15 fixed point) format.
 *
 * @param d floating-point value within range -1 to (1 - (2**-15)), inclusive
 * @return Q15 value representing d; same range
 */
static inline int16_t q15_from_double(const double d) {
  // couldn't find lrint
  //  return lrint(d * 32768);
  return d * 32768;
}

/**
 * Q15 (1.0.15 fixed point) multiplication. Various common rounding modes are in
 * the function definition for reference (and preference).
 *
 * @param j 16-bit signed integer representing -1 to (1 - (2**-15)), inclusive
 * @param k same format as j
 * @return product of j and k, in same format
 */
static inline int16_t q15_mul(const int16_t j, const int16_t k) {
  const int32_t intermediate = j * (int32_t)k;
#if 0 // don't round
  return intermediate >> 15;
#elif 0 // biased rounding
  return (intermediate + 0x4000) >> 15;
#else // unbiased rounding
  return (intermediate + ((intermediate & 0x7FFF) == 0x4000 ? 0 : 0x4000)) >> 15;
#endif
}

// from: http://geekshavefeelings.com/posts/fixed-point-atan2
static uint16_t fxpt_atan2(const int16_t y, const int16_t x) {
  if (x == y) { // x/y or y/x would return -1 since 1 isn't representable
    if (y > 0) { // 1/8
      return 8192;
    } else if (y < 0) { // 5/8
      return 40960;
    } else { // x = y = 0
      return 0;
    }
  }
  const int16_t nabs_y = s16_nabs(y), nabs_x = s16_nabs(x);
  if (nabs_x < nabs_y) { // octants 1, 4, 5, 8
    const int16_t y_over_x = q15_div(y, x);
    const int16_t correction = q15_mul(q15_from_double(0.273 * M_1_PI), s16_nabs(y_over_x));
    const int16_t unrotated = q15_mul(q15_from_double(0.25 + 0.273 * M_1_PI) + correction, y_over_x);
    if (x > 0) { // octants 1, 8
      return unrotated;
    } else { // octants 4, 5
      return 32768 + unrotated;
    }
  } else { // octants 2, 3, 6, 7
    const int16_t x_over_y = q15_div(x, y);
    const int16_t correction = q15_mul(q15_from_double(0.273 * M_1_PI), s16_nabs(x_over_y));
    const int16_t unrotated = q15_mul(q15_from_double(0.25 + 0.273 * M_1_PI) + correction, x_over_y);
    if (y > 0) { // octants 2, 3
      return 16384 - unrotated;
    } else { // octants 6, 7
      return 49152 - unrotated;
    }
  }
}
