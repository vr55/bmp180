/********************************** (C) COPYRIGHT  *****************************
 * File Name          : bmp180.c
 * Author             :	Alexander Volosenkov
 * Version            : V1.0.0
 * Date               :
 * Description        :
********************************************************************************
* Copyright (c) 2024
*
*
*******************************************************************************/


#include "bmp180.h"

static bmp180_t *p_bmp180;

/*******************************************************************************
 * @fn		BMP180_GetChipId
 *
 * @brief	Gets the chip identifier of the BMP180 sensor.
 *
 *
 * @param	none
 *
 * @return	Chip id or -1 if error
 ******************************************************************************/
int8_t BMP180_GetChipId()
{

	return BMP180_ReadByte( BMP180_CHIP_ID_REG_ADDRESS );

}

/*******************************************************************************
 * @fn		BMP180_GetChipVersion
 *
 * @brief	Gets the chip version of the BMP180 sensor.
 *
 *
 * @param	none
 *
 * @return	Chip version or -1 if error
 ******************************************************************************/
static int8_t	BMP180_GetChipVersion()
{

	return BMP180_ReadByte( BMP180_VERSION_REG_ADDRESS );

}

/*******************************************************************************
 * @fn		BMP180_Setup
 *
 * @brief	Sets up the BMP180 sensor for communication with the microcontroller.
 *
 *
 * @param	Pointer to I2C_TypeDef structure
 * 			Pointer to bmp180_t structure
 *
 * @return	1 if success, -1 if error
 ******************************************************************************/
char BMP180_Setup( I2C_TypeDef *I2Cx, bmp180_t *bmp180 )
{

	p_bmp180 = bmp180;
	p_bmp180->I2Cx = I2Cx;

	if ( BMP180_Probe() != 1 )
	{
		I2C_GenerateSTOP( p_bmp180->I2Cx, ENABLE );
		return -1;
	}

	p_bmp180->chip_id = BMP180_GetChipId();
	BMP180_ReadCalibration_Data();

	return 1;
}

/*******************************************************************************
 * @fn		BMP180_Probe
 *
 * @brief	Probes the availability of the BMP180 sensor on the I2C bus.
 *
 *
 * @param	none
 *
 * @return	1 is success, -1 if error
 ******************************************************************************/
static char	BMP180_Probe()
{
	uint8_t timeout = BMP180_DEFAULT_I2C_TIMEOUT;

	// Wait for idle I2C interface
	while( I2C_GetFlagStatus( p_bmp180->I2Cx, I2C_FLAG_BUSY ) != RESET )
	{
		if( (timeout--) == 0 )
			return -1;
	}

	// Initiate Start Sequence (wait for EV5
	timeout = BMP180_DEFAULT_I2C_TIMEOUT;
    I2C_GenerateSTART( p_bmp180->I2Cx, ENABLE );

    while ( !I2C_CheckEvent( p_bmp180->I2Cx, I2C_EVENT_MASTER_MODE_SELECT) )
    {
    	if( (timeout--) == 0 )
    		return -1;
    }

    timeout = BMP180_DEFAULT_I2C_TIMEOUT;

    I2C_Send7bitAddress( p_bmp180->I2Cx, BMP180_ADDRESS, I2C_Direction_Transmitter );

    while( !I2C_CheckEvent( p_bmp180->I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) )
    {
    	if ( (timeout--) == 0 )
    		return -1;
    }

    I2C_GenerateSTOP( p_bmp180->I2Cx, ENABLE );

    return 1;
}

/*******************************************************************************
 * @fn		BMP180_Soft_reset
 *
 * @brief	Performs a soft reset of the BMP180 sensor.
 *
 *
 * @param	none
 *
 * @return	none
 ******************************************************************************/
void BMP180_Soft_reset()
{
	BMP180_WriteByte( BMP180_SOFT_RESET_REG_ADDRESS, BMP180_SOFT_RESET_REG_VALUE );
}

/*******************************************************************************
 * @fn		BMP180_ReadCalibration_Data
 *
 * @brief	Reads the calibration data from the BMP180 sensor.
 *
 *
 * @param	none
 *
 * @return	none
 ******************************************************************************/
static void BMP180_ReadCalibration_Data()
{
	uint8_t msb, lsb;

	for( uint8_t i=0; i<sizeof(BMP180_CAL_REG_ADDRESS); i++ )
	{
		msb = BMP180_ReadByte( BMP180_CAL_REG_ADDRESS[i] );
		lsb = BMP180_ReadByte( BMP180_CAL_REG_ADDRESS[i] + 0x1 );

		switch( BMP180_CAL_REG_ADDRESS[i] )
		{
		case BMP180_CAL_AC1_REG_ADDRESS:
			p_bmp180->calibration_data.AC1 = (int16_t)(msb << 8 | lsb);
			break;
		case BMP180_CAL_AC2_REG_ADDRESS:
			p_bmp180->calibration_data.AC2 = (int16_t)(msb << 8 | lsb);
			break;
		case BMP180_CAL_AC3_REG_ADDRESS:
			p_bmp180->calibration_data.AC3 = (int16_t)(msb << 8 | lsb);
			break;
		case BMP180_CAL_AC4_REG_ADDRESS:
			p_bmp180->calibration_data.AC4 = (uint16_t)(msb << 8 | lsb);
			break;
		case BMP180_CAL_AC5_REG_ADDRESS:
			p_bmp180->calibration_data.AC5 = (uint16_t)(msb << 8 | lsb);
			break;
		case BMP180_CAL_AC6_REG_ADDRESS:
			p_bmp180->calibration_data.AC6 = (uint16_t)(msb << 8 | lsb);
			break;
		case BMP180_CAL_B1_REG_ADDRESS:
			p_bmp180->calibration_data.B1 = (int16_t)(msb << 8 | lsb);
			break;
		case BMP180_CAL_B2_REG_ADDRESS:
			p_bmp180->calibration_data.B2 = (int16_t)(msb << 8 | lsb);
			break;
		case BMP180_CAL_MB_REG_ADDRESS:
			p_bmp180->calibration_data.MB = (int16_t)(msb << 8 | lsb);
			break;
		case BMP180_CAL_MC_REG_ADDRESS:
			p_bmp180->calibration_data.MC = (int16_t)(msb << 8 | lsb);
			break;
		case BMP180_CAL_MD_REG_ADDRESS:
			p_bmp180->calibration_data.MD = (int16_t)(msb << 8 | lsb);
			break;
		}
	}

}

/*******************************************************************************
 * @fn		BMP180_GetUncompensatedTemperature
 *
 * @brief	Gets the uncompensated temperature from the BMP180 sensor.
 *
 *
 * @param	none
 *
 * @return	Uncompensated temperature
 ******************************************************************************/
static int32_t	BMP180_GetUncompensatedTemperature()
{
	enum BMP180_CTL_REG_VALUE bmp180_ctl_temperature = BMP180_CTL_TEMPERATURE;
	_Bool is_measure_in_progress = 1;
	uint8_t data, msb, lsb;

	//Write command to control register
	//TODO:: return code
	BMP180_WriteByte( BMP180_CTL_REG_ADDRESS, bmp180_ctl_temperature );

	//wait while measurement is complete
	while( is_measure_in_progress )
	{
		data = BMP180_ReadByte( BMP180_CTL_REG_ADDRESS );
		is_measure_in_progress = GETBIT( data, BMP180_CTL_REG_SCO_BIT_POSITION );
	}

	//read uncompensated temperature
	msb = BMP180_ReadByte( BMP180_DATA_REG_ADDRESS_MSB );
	lsb = BMP180_ReadByte( BMP180_DATA_REG_ADDRESS_LSB );

	return (int32_t)(msb << 8 | lsb);
}

/*******************************************************************************
 * @fn		BMP180_GetUncompensatedPressure
 *
 * @brief	Gets the uncompensated pressure from the BMP180 sensor with
 * 			the specified accuracy.
 *
 *
 * @param	BMP180_PRESSURE_ACCURACY enum
 *
 * @return	Uncompensated pressure
 ******************************************************************************/
static int32_t BMP180_GetUncompensatedPressure( enum BMP180_PRESSURE_ACCURACY Accuracy )
{
	enum BMP180_CTL_REG_VALUE	bmp180_ctl_reg_value;
	_Bool is_measure_in_progress = 1;
	uint8_t data, msb, lsb, xlsb, oss;
	int32_t up;

	switch( Accuracy )
	{
	case BMP180_ACC_ULTRA_LOW_POWER:
		bmp180_ctl_reg_value = BMP180_CTL_PESSURE_OSS_0;
		oss = 0;
		break;
	case BMP180_ACC_STANDARD:
		bmp180_ctl_reg_value = BMP180_CTL_PESSURE_OSS_1;
		oss = 1;
		break;
	case BMP180_ACC_HIGH_RES:
		bmp180_ctl_reg_value = BMP180_CTL_PESSURE_OSS_2;
		oss = 2;
		break;
	case BMP180_ACC_ULTRA_HIGH_RES:
		bmp180_ctl_reg_value = BMP180_CTL_PESSURE_OSS_3;
		oss = 3;
		break;
	}

	//Write command to control register
	//TODO:: return code
	BMP180_WriteByte( BMP180_CTL_REG_ADDRESS, bmp180_ctl_reg_value );

	//wait while measurement is complete
	while( is_measure_in_progress )
	{
		data = BMP180_ReadByte( BMP180_CTL_REG_ADDRESS );
		is_measure_in_progress = GETBIT( data, BMP180_CTL_REG_SCO_BIT_POSITION );
	}

	//read uncompensated pressure
	msb 	= BMP180_ReadByte( BMP180_DATA_REG_ADDRESS_MSB );
	lsb 	= BMP180_ReadByte( BMP180_DATA_REG_ADDRESS_LSB );
	xlsb 	= BMP180_ReadByte( BMP180_DATA_REG_ADDRESS_XLSB );

	up = ( msb << 16 ) + ( lsb << 8 ) + xlsb;

	return (int32_t)( up >> (8 - oss) );

}

/*******************************************************************************
 * @fn		BMP180_GetPressure
 *
 * @brief	Gets the compensated pressure from the BMP180 sensor with
 * 			the specified accuracy.
 *
 *
 * @param	BMP180_PRESSURE_ACCURACY enum
 *
 * @return	pressure if Pascals
 ******************************************************************************/
int32_t BMP180_GetPressure( enum BMP180_PRESSURE_ACCURACY Accuracy  )
{
	uint8_t	oss = 1; //oversampling setting

	int32_t x1, x2, x3, b3, b6, p, up;

	int32_t	ac1 = (int32_t)p_bmp180->calibration_data.AC1,
			ac2 = (int32_t)p_bmp180->calibration_data.AC2,
		    b2 = (int32_t)p_bmp180->calibration_data.B2,
			ac3 = (int32_t)p_bmp180->calibration_data.AC3,
		    b1 = (int32_t)p_bmp180->calibration_data.B1,
			b5 = (int32_t)p_bmp180->calibration_data.B5;

	uint32_t	b4, b7;
	uint32_t	ac4 = (uint32_t)p_bmp180->calibration_data.AC4;

	up = BMP180_GetUncompensatedPressure( Accuracy );

	switch( Accuracy )
	{
	case BMP180_ACC_ULTRA_LOW_POWER:
		oss = 0;
		break;
	case BMP180_ACC_STANDARD:
		oss = 1;
		break;
	case BMP180_ACC_HIGH_RES:
		oss = 2;
		break;
	case BMP180_ACC_ULTRA_HIGH_RES:
		oss = 3;
		break;
	}

	b6 = b5 - 4000;

	x1 =  (b2 * ((b6 * b6) >> 12)) >> 11;
	x2 =  ( ac2 * b6 ) >> 11;
	x3 = x1 + x2;
	b3 = (( (ac1 * 4 + x3) << oss) + 2) >> 2;

	x1 = ( ac3 * b6 ) >> 13;
	x2 = ( b1 * ( (b6 * b6) >> 12)) >> 16;
	x3 = ( (x1 + x2) + 2 ) >> 2;

	b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15 ;
	b7 = ( up - b3 ) * ( 50000 >> oss );

	if ( b7 < 0x80000000 )
		p = ( b7 * 2 ) / b4;
	else
		p = ( b7 / b4 ) << 1;

	x1 = ( p >> 8 ) * ( p >> 8 );
	x1 = ( x1 * 3038 ) >> 16;
	x2 = ( -7357 * p ) >> 16;

	p = p + ( ( x1 + x2 + (int32_t)3791 ) >> 4 );

	return p;

}

/*******************************************************************************
 * @fn		BMP180_GetTemperature
 *
 * @brief	Gets the compensated temperature from the BMP180 sensor.
 *
 *
 * @param	none
 *
 * @return	temperature in Celsius degrees
 ******************************************************************************/
int32_t BMP180_GetTemperature()
{
	int32_t		x1, x2, b5, t, ut;
	uint16_t 	ac5 = p_bmp180->calibration_data.AC5,
				ac6 = p_bmp180->calibration_data.AC6;
	int16_t		mc 	= p_bmp180->calibration_data.MC,
				md 	= p_bmp180->calibration_data.MD;

	ut = BMP180_GetUncompensatedTemperature();

	x1 = ( ut - ac6 ) * ac5 >> 15;

	x2 = ( mc << 11 ) / ( x1 + md );

	b5 = x1 + x2;

	p_bmp180->calibration_data.B5 = b5;

	t =((b5 + 8) >> 4) / 10;

	return t;

}

/*******************************************************************************
 * @fn		BMP180_GetAltitude
 *
 * @brief	Gets the altitude based on the measured pressure from
 * 			the BMP180 sensor.
 *
 *
 * @param	Pressure
 *
 * @return	altitude
 ******************************************************************************/
int32_t BMP180_GetAltitude( uint32_t pressure )
{
	float f = 44330 * ( 1 - powf( pressure / 101325.0, 1.0/5.255 ) );
	return (int32_t)f;
}

/*******************************************************************************
 * @fn		BMP180_ReadByte
 *
 * @brief	Reads a byte of data from the BMP180 sensor over the I2C bus.
 *
 *
 * @param
 *
 * @return	Byte data or -1 if error
 ******************************************************************************/
static int8_t	BMP180_ReadByte( uint8_t RegAddress )
{
	uint8_t timeout = BMP180_DEFAULT_I2C_TIMEOUT;
	uint8_t result;

	// Wait for idle I2C interface
	while( I2C_GetFlagStatus( p_bmp180->I2Cx, I2C_FLAG_BUSY ) != RESET )
	{
		if( (timeout--) == 0 )
			return -1;
	}

	// Generate start
	timeout = BMP180_DEFAULT_I2C_TIMEOUT;
    I2C_GenerateSTART( p_bmp180->I2Cx, ENABLE );
    while ( !I2C_CheckEvent( p_bmp180->I2Cx, I2C_EVENT_MASTER_MODE_SELECT) )
	{
		if( (timeout--) == 0 )
			return -1;
	}

    // Send device address
    timeout = BMP180_DEFAULT_I2C_TIMEOUT;
    I2C_Send7bitAddress( p_bmp180->I2Cx, BMP180_ADDRESS, I2C_Direction_Transmitter );
    while( !I2C_CheckEvent( p_bmp180->I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) )
	{
		if( (timeout--) == 0 )
			return -1;
	}

    //I2C Send Data
    timeout = BMP180_DEFAULT_I2C_TIMEOUT;
	I2C_SendData( p_bmp180->I2Cx, RegAddress );
	while( !I2C_CheckEvent( p_bmp180->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) )
	{
		if( (timeout--) == 0 )
			return -1;
	}

	timeout = BMP180_DEFAULT_I2C_TIMEOUT;
    I2C_GenerateSTART( p_bmp180->I2Cx, ENABLE );
    while ( !I2C_CheckEvent( p_bmp180->I2Cx, I2C_EVENT_MASTER_MODE_SELECT ) )
	{
		if( (timeout--) == 0 )
			return -1;
	}

    timeout = BMP180_DEFAULT_I2C_TIMEOUT;
    I2C_Send7bitAddress( p_bmp180->I2Cx, BMP180_ADDRESS, I2C_Direction_Receiver );
    while ( !I2C_CheckEvent( p_bmp180->I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) )
	{
		if( (timeout--) == 0 )
			return -1;
	}

    timeout = BMP180_DEFAULT_I2C_TIMEOUT;
    while( I2C_GetFlagStatus( p_bmp180->I2Cx, I2C_FLAG_RXNE ) ==  RESET )
	{
		if( (timeout--) == 0 )
			return -1;
	}

    I2C_NACKPositionConfig( p_bmp180->I2Cx, I2C_NACKPosition_Current );
    I2C_AcknowledgeConfig( p_bmp180->I2Cx, DISABLE );

    // read data from I2C data register and return data byte
    result = I2C_ReceiveData(p_bmp180->I2Cx);

    timeout = BMP180_DEFAULT_I2C_TIMEOUT;
    while( !I2C_CheckEvent( p_bmp180->I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) )
	{
		if( (timeout--) == 0 )
			return -1;
	}

    timeout = BMP180_DEFAULT_I2C_TIMEOUT;
    I2C_GenerateSTOP( p_bmp180->I2Cx, ENABLE );
    while( I2C_GetFlagStatus( p_bmp180->I2Cx, I2C_FLAG_RXNE ) ==  RESET )
	{
		if( (timeout--) == 0 )
			return -1;
	}

    // Wait for stop
    //while( I2C_GetFlagStatus( p_bmp180->I2Cx, I2C_FLAG_STOPF ) );

    return result;
}

/*******************************************************************************
 * @fn		BMP180_WriteByte
 *
 * @brief	Writes a byte of data to the BMP180 sensor over the I2C bus.
 *
 *
 * @param	RegAddress - address of register
 * @param	ByteToWrite - byte should be written
 *
 * @return	1 if success, -1 if error
 ******************************************************************************/
static char	BMP180_WriteByte( uint8_t RegAddress, uint8_t ByteToWrite )
{
	uint8_t timeout = BMP180_DEFAULT_I2C_TIMEOUT;

	while( I2C_GetFlagStatus( p_bmp180->I2Cx, I2C_FLAG_BUSY ) != RESET )
	{
		if( (timeout--) == 0 )
			return -1;
	}

	timeout = BMP180_DEFAULT_I2C_TIMEOUT;
	I2C_GenerateSTART( p_bmp180->I2Cx, ENABLE );
	while( !I2C_CheckEvent( p_bmp180->I2Cx, I2C_EVENT_MASTER_MODE_SELECT ) )
	{
		if( (timeout--) == 0 )
			return -1;
	}

	//Send device i2c address
	timeout = BMP180_DEFAULT_I2C_TIMEOUT;
	I2C_Send7bitAddress( p_bmp180->I2Cx, BMP180_ADDRESS, I2C_Direction_Transmitter );
	while( !I2C_CheckEvent( p_bmp180->I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) )
	{
		if( (timeout--) == 0 )
			return -1;
	}

	//Send Register address
	timeout = BMP180_DEFAULT_I2C_TIMEOUT;
	I2C_SendData( p_bmp180->I2Cx, RegAddress );
	while( !I2C_CheckEvent( p_bmp180->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) )
	{
		if( (timeout--) == 0 )
			return -1;
	}

	//Send data
	if( I2C_GetFlagStatus( p_bmp180->I2Cx, I2C_FLAG_TXE ) !=  RESET )
	{
		I2C_SendData( p_bmp180->I2Cx, ByteToWrite );
	}

	timeout = BMP180_DEFAULT_I2C_TIMEOUT;
	while( !I2C_CheckEvent( p_bmp180->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) )
	{
		if( (timeout--) == 0 )
			return -1;
	}
	I2C_GenerateSTOP( p_bmp180->I2Cx, ENABLE );

	return 1;

}
