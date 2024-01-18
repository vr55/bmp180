
#ifndef __BMP180_H_
#define __BMP180_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "stm32f10x_i2c.h"

#define GETBIT(var, bit)	(((var) >> (bit)) & 1)
#define BMP180_DEFAULT_I2C_TIMEOUT		200

#define BMP180_ADDRESS                	(uint8_t)0x77 << 1 	//i2c address
#define BMP180_CHIP_ID_REG_ADDRESS		(uint8_t)0xD0		//
#define BMP180_CHIP_ID                	(uint8_t)0x55  		//id number

#define BMP180_VERSION_REG_ADDRESS		(uint8_t)0xD1


#define BMP180_CTL_REG_ADDRESS	  		(uint8_t)0xF4		//control register address

#define BMP180_SOFT_RESET_REG_ADDRESS	(uint8_t)0xE0
#define BMP180_SOFT_RESET_REG_VALUE		(uint8_t)0xB6

#define BMP180_DATA_REG_ADDRESS_MSB		(uint8_t)0xF6
#define BMP180_DATA_REG_ADDRESS_LSB 	(uint8_t)0xF7
#define BMP180_DATA_REG_ADDRESS_XLSB 	(uint8_t)0xF8

#define BMP180_CALIBRATION_DATA_SIZE	11					//number of calibration registers

#define BMP180_CTL_REG_SCO_BIT_POSITION	5


typedef struct
{
	  int16_t  AC1;
	  int16_t  AC2;
	  int16_t  AC3;
	  uint16_t AC4;
	  uint16_t AC5;
	  uint16_t AC6;
	  int16_t  B1;
	  int16_t  B2;
	  int16_t  MB;
	  int16_t  MC;
	  int16_t  MD;
	  int16_t  B5;
}BMP180_CALIBRATION_DATA;


enum BMP180_CTL_REG_VALUE
{
	BMP180_CTL_TEMPERATURE		= (uint8_t)0x2E,
	BMP180_CTL_PESSURE_OSS_0	= (uint8_t)0x34,			//BMP180_ACC_ULTRA_LOW_POWER
	BMP180_CTL_PESSURE_OSS_1	= (uint8_t)0x74,			//BMP180_ACC_STANDARD
	BMP180_CTL_PESSURE_OSS_2	= (uint8_t)0xB4,			//BMP180_ACC_HIGH_RES
	BMP180_CTL_PESSURE_OSS_3	= (uint8_t)0xF4,			//BMP180_ACC_ULTRA_HIGH_RES
};

enum BMP180_PRESSURE_ACCURACY {
	BMP180_ACC_ULTRA_LOW_POWER = 1,
	BMP180_ACC_STANDARD,
	BMP180_ACC_HIGH_RES,
	BMP180_ACC_ULTRA_HIGH_RES,
};

typedef struct
{
	BMP180_CALIBRATION_DATA	calibration_data;
	enum BMP180_PRESSURE_ACCURACY	pressure_accuracy: BMP180_ACC_ULTRA_HIGH_RES;
	uint8_t	chip_id;
	I2C_TypeDef *I2Cx;


}bmp180_t;


#define	BMP180_CAL_DATA_START_REG_ADDRESS          (uint8_t)0xAA  //ac1 pressure    computation

#define	BMP180_CAL_AC1_REG_ADDRESS                 (uint8_t)0xAA  //ac1 pressure    computation
#define	BMP180_CAL_AC2_REG_ADDRESS                 (uint8_t)0xAC  //ac2 pressure    computation
#define	BMP180_CAL_AC3_REG_ADDRESS                 (uint8_t)0xAE  //ac3 pressure    computation
#define	BMP180_CAL_AC4_REG_ADDRESS                 (uint8_t)0xB0  //ac4 pressure    computation
#define BMP180_CAL_AC5_REG_ADDRESS                 (uint8_t)0xB2  //ac5 temperature computation
#define	BMP180_CAL_AC6_REG_ADDRESS                 (uint8_t)0xB4  //ac6 temperature computation
#define BMP180_CAL_B1_REG_ADDRESS                  (uint8_t)0xB6  //b1  pressure    computation
#define BMP180_CAL_B2_REG_ADDRESS                  (uint8_t)0xB8  //b2  pressure    computation
#define	BMP180_CAL_MB_REG_ADDRESS                  (uint8_t)0xBA  //mb
#define	BMP180_CAL_MC_REG_ADDRESS                  (uint8_t)0xBC  //mc  temperature computation
#define	BMP180_CAL_MD_REG_ADDRESS                  (uint8_t)0xBE  //md  temperature computation


static uint8_t BMP180_CAL_REG_ADDRESS[] =
							   {
								BMP180_CAL_AC1_REG_ADDRESS,
								BMP180_CAL_AC2_REG_ADDRESS,
								BMP180_CAL_AC3_REG_ADDRESS,
								BMP180_CAL_AC4_REG_ADDRESS,
								BMP180_CAL_AC5_REG_ADDRESS,
								BMP180_CAL_AC6_REG_ADDRESS,
								BMP180_CAL_B1_REG_ADDRESS,
								BMP180_CAL_B2_REG_ADDRESS,
								BMP180_CAL_MB_REG_ADDRESS,
								BMP180_CAL_MC_REG_ADDRESS,
								BMP180_CAL_MD_REG_ADDRESS,
							   };


static	int8_t	BMP180_ReadByte( uint8_t RegAddress );
static char		BMP180_WriteByte( uint8_t RegAddress, uint8_t ByteToWrite );
static int8_t	BMP180_GetChipVersion();
static void		BMP180_ReadCalibration_Data();
static	int8_t	BMP180_GetChipId();
static int32_t	BMP180_GetUncompensatedTemperature();
static int32_t 	BMP180_GetUncompensatedPressure( enum BMP180_PRESSURE_ACCURACY Accuracy );
static char		BMP180_Probe();

char 			BMP180_Setup( I2C_TypeDef *I2Cx, bmp180_t *bmp180 );
void			BMP180_Soft_reset();
int32_t			BMP180_GetTemperature();
int32_t			BMP180_GetPressure( enum BMP180_PRESSURE_ACCURACY Accuracy  );
int32_t			BMP180_GetAltitude( uint32_t pressure );


#endif /* BMP180_H_ */
