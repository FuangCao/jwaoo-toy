//MPU6050 I2C library for DAS14580 - Main source file
/* Includes */

#include <stdint.h>
#include "global_io.h"
#include "gpio.h"
#include "user_periph_setup.h"
#include "i2c.h"
#include "mpu6050.h"
#include "uart.h"

/** @defgroup MPU6050_Library
* @{
*/

 static double mpu6050_bias_accel[3];
 static double mpu6050_bias_gyro[3];

void delay_ms(unsigned int num)
{
  unsigned int i = 100*num;
	while(i--)
	{

  }
}

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void MPU6050_Initialize(void) 
{
	println("MPU6050_Initialize");

    MPU6050_SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
	  delay_ms(100);
    MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_250); //every 131 means 1 degree/second
	  delay_ms(100);
    MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);// every 16384 means 1g
	  delay_ms(100);
    MPU6050_SetSleepModeStatus(DISABLE); 
	  delay_ms(100);
	  BZERO(mpu6050_bias_accel);
	  BZERO(mpu6050_bias_gyro);

}

void MPU6050_DeInitialize(void) 
{
	MPU6050_SetSleepModeStatus(ENABLE);  //allow the MPU6050 to enter sleep mode
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, FALSE otherwise
 */
bool MPU6050_TestConnection(void) 
{
    if(MPU6050_GetDeviceID() == MPU6050_CHIP_ID) //0b110100; 8-bit representation in hex = 0x34
      return TRUE;
    else
      return FALSE;
}
// MPU6050_RA_CHIP_ID register

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100).
 * @return Device ID (should be 0x68, 104 dec, 150 oct)
 * @see MPU6050_RA_CHIP_ID
 * @see MPU6050_RA_CHIP_ID_BIT
 * @see MPU6050_RA_CHIPD_ID_LENGTH
 */
uint8_t MPU6050_GetDeviceID()
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CHIP_ID, MPU6050_RA_CHIP_ID_BIT, MPU6050_RA_CHIPD_ID_LENGTH, &tmp);
	println("READ MPU6050 ChipID: 0X%02X",tmp&0x00FF);
    return tmp; 
}
/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see MPU6050_GetClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void MPU6050_SetClockSource(uint8_t source) 
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see MPU6050_GetFullScaleGyroRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_SetFullScaleGyroRange(uint8_t range) 
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

// GYRO_CONFIG register

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
uint8_t MPU6050_GetFullScaleGyroRange() 
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, &tmp);
    return tmp;
}
/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
uint8_t MPU6050_GetFullScaleAccelRange() 
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, &tmp);
    return tmp;
}
/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see MPU6050_GetFullScaleAccelRange()
 */
void MPU6050_SetFullScaleAccelRange(uint8_t range) 
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
bool MPU6050_GetSleepModeStatus() 
{
    uint8_t tmp;
    MPU6050_ReadBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, &tmp);
    if(tmp == 0x00)
      return FALSE;
    else
      return TRUE;    
}
/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see MPU6050_GetSleepModeStatus()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void MPU6050_SetSleepModeStatus(uint8_t NewState) 
{
    MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, NewState);
}

void MPU6050_TestDumpRawAccelGyro(void)
{
	
	int16_t AccelGyro[6];
	double temp=0.0;
	
	MPU6050_GetRawAccelGyro(AccelGyro);
	MPU6050_ReadTemp(&temp);//read the temperature in the MPU6050
	MPU6050_DumpAccelGyroTemperature(AccelGyro,temp);
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param AccelGyro 16-bit signed integer array of length 6
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
void MPU6050_GetRawAccelGyro(int16_t* AccelGyro) 
{
    uint8_t tmpBuffer[14]; 
    MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, tmpBuffer, MPU6050_RA_ACCEL_XOUT_H, 14); 
    /* Get acceleration */
    for(int i=0; i<3; i++) 
      AccelGyro[i]=((int16_t)((uint16_t)tmpBuffer[2*i] << 8) + tmpBuffer[2*i+1]);
   /* Get Angular rate */
    for(int i=4; i<7; i++)
      AccelGyro[i-1]=((int16_t)((uint16_t)tmpBuffer[2*i] << 8) + tmpBuffer[2*i+1]);        

}

/** Write multiple bits in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 */
void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) 
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);  
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing byte
    tmp |= data; // combine data with existing byte
    MPU6050_I2C_ByteWrite(slaveAddr,&tmp,regAddr);   
}
/** write a single bit in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 */
void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) 
{
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);  
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    MPU6050_I2C_ByteWrite(slaveAddr,&tmp,regAddr); 
}
/** Read multiple bits from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) 
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1); 
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    tmp &= mask;
    tmp >>= (bitStart - length + 1);
    *data = tmp;
}

/** Read a single bit from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) 
{
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);  
    *data = tmp & (1 << bitNum);
}

/**
* @brief  Writes one byte to the  MPU6050.
* @param  slaveAddr : slave address MPU6050_DEFAULT_ADDRESS
* @param  pBuffer : pointer to the buffer  containing the data to be written to the MPU6050.
* @param  writeAddr : address of the register in which the data will be written
* @return None
*/

void MPU6050_I2C_ByteWrite(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t writeAddr)
{

	/*fixed me: slaveAddr is not used in this implementation. 
	  reason is the DAS14580 current I2C controller set the 
	  slave address to I2C_TAR_REG during init.
	*/
	if(pBuffer)
		i2c_write_u8(slaveAddr, writeAddr, *pBuffer);
}

/**
* @brief  Reads a block of data from the MPU6050.
* @param  slaveAddr  : slave address MPU6050_DEFAULT_ADDRESS
* @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
* @param  readAddr : MPU6050's internal address to read from.
* @param  NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1  only for the Mgnetometer readinf).
* @return None
*/

void MPU6050_I2C_BufferRead(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t readAddr, uint16_t NumByteToRead)
{
	 /*fixed me: slaveAddr is not used in this implementation. 
	  reason is the DAS14580 current I2C controller set the 
	  slave address to I2C_TAR_REG during init.
	*/
  if((NumByteToRead>0)&&(pBuffer))
	  i2c_read_data(slaveAddr, readAddr, pBuffer, NumByteToRead);
}

void MPU6050ReadTempRaw(short *tempData)
{
	uint8_t buf[2];
  MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,(uint8_t *)buf,MPU6050_RA_TEMP_OUT_H,2);     //Read temperature
  *tempData = (buf[0] << 8) | buf[1];
}

void MPU6050_ReadTemp(double*Temperature)
{
	short temp3;
	uint8_t buf[2];
	
	MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,(uint8_t *)buf,MPU6050_RA_TEMP_OUT_H,2);     //Read temperature raw
  temp3= (buf[0] << 8) | buf[1];
	*Temperature=(((double) (temp3 + 13200.0)) / 280.0)-13.0;
}

void MPU6050_Calibration( void) //Run the MPU6050 calibration
{
	unsigned int i,j;
	int16_t AccelGyro[6];
  unsigned int num=200; // we read the test data 200 times when the board is horizentally placed in a flat surface
	double bias_gyro[3], bias_accel[3];
	long int bias_gyro_max[3],bias_accel_max[3];
	long int bias_gyro_min[3],bias_accel_min[3];
	double bias_gyro_mean[3],bias_accel_mean[3];
	
	BZERO(bias_gyro);	
	BZERO(bias_accel);
	BZERO(bias_gyro_max);
	BZERO(bias_accel_max);
	BZERO(bias_gyro_min);
	BZERO(bias_accel_min);
	BZERO(bias_gyro_mean);
	BZERO(bias_accel_mean);
	
	println("------------------------------");
	println("!!!MPU6050 calibration started, please place the board in the stable flat surface , don't touch or shake it");
		
	for(i=0;i<10;i++)
	{
	MPU6050_GetRawAccelGyro(AccelGyro);
	delay_ms(8);
	}
	
	MPU6050_GetRawAccelGyro(AccelGyro);
	
	bias_accel[0]=AccelGyro[0];
	bias_accel[1]=AccelGyro[1];
	bias_accel[2]=AccelGyro[2];
	
	bias_gyro[0]=AccelGyro[3];
	bias_gyro[1]=AccelGyro[4];
	bias_gyro[2]=AccelGyro[5];
	
	BCOPY(bias_accel_max,bias_accel);
	BCOPY(bias_gyro_max,bias_gyro);

	BCOPY(bias_accel_min,bias_accel);
	BCOPY(bias_gyro_min,bias_gyro);
	
	for (i=1;i<num;i++)
	{
	memset(AccelGyro,0,sizeof(AccelGyro));

	MPU6050_GetRawAccelGyro(AccelGyro);
	
	bias_accel[0]+=(double)AccelGyro[0];
	bias_accel[1]+=(double)AccelGyro[1];
	bias_accel[2]+=(double)(AccelGyro[2]-16384.0);
	
	bias_gyro[0]+=(double)AccelGyro[3];
	bias_gyro[1]+=(double)AccelGyro[4];
	bias_gyro[2]+=(double)AccelGyro[5];
		
	MPU6050_DumpCalibratedAccelGyro(AccelGyro);
	
	for(j=0;j<3;j++)
		{
		bias_accel_max[j]=GET_MAX(bias_accel_max[j],AccelGyro[j]);
		bias_gyro_max[j]=GET_MAX(bias_gyro_max[j],AccelGyro[j+3]);
    
    bias_accel_min[j]=GET_MIN(bias_accel_min[j],AccelGyro[j]);
		bias_gyro_min[j]=GET_MIN(bias_gyro_min[j],AccelGyro[j+3]);
		}
	
	}
	
  println("Calibration Raw Data sum over %d tests!",num);
	
	MPU6050_DumpCalibratedAccelGyro2(bias_accel,bias_gyro);
	
	for(j=0;j<3;j++)
		{
	  bias_accel_mean[j]=(bias_accel[j]/num);
		bias_gyro_mean[j]=(bias_gyro[j]/num);	
			
		bias_accel_mean[j]=bias_accel_mean[j]/16384.0;
		bias_gyro_mean[j]=bias_gyro_mean[j]/131.0;
			
		}
	
	println("Calibration done!");
	
	BCOPY(mpu6050_bias_accel,bias_accel_mean);
	BCOPY(mpu6050_bias_gyro,bias_gyro_mean);
	
	
	println("--------------Calibration bias result---------------");
  MPU6050_DumpCalibratedAccelGyro2(mpu6050_bias_accel,mpu6050_bias_gyro);
	
}

void MPU6050_DumpCalibratedAccelGyro(int16_t *AccelGyro)
{
	double accel[3],gyro[3];
	accel[0]=AccelGyro[0]/16384.0-mpu6050_bias_accel[0];
	accel[1]=AccelGyro[1]/16384.0-mpu6050_bias_accel[1];;
	accel[2]=AccelGyro[2]/16384.0-mpu6050_bias_accel[2];
	
	gyro[0]=AccelGyro[3]/131.0-mpu6050_bias_gyro[0];
	gyro[1]=AccelGyro[4]/131.0-mpu6050_bias_gyro[1];
	gyro[2]=AccelGyro[5]/131.0-mpu6050_bias_gyro[2];
		
	println("Accel/Gyro:  %8.5f\t%8.5f\t%8.5f\t%8.5f\t%8.5f\t%8.5f",accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2]);
		
}

void MPU6050_DumpCalibratedAccelGyro2(double *accel,double *gyro)
{
	println("Accel/Gyro:  %8.5f\t%8.5f\t%8.5f\t%8.5f\t%8.5f\t%8.5f",accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2]);
}

void MPU6050_DumpAccelGyroTemperature(int16_t *AccelGyro,double temperature)
{
	double accel[3],gyro[3];
	accel[0]=AccelGyro[0]/16384.0-mpu6050_bias_accel[0];
	accel[1]=AccelGyro[1]/16384.0-mpu6050_bias_accel[1];;
	accel[2]=AccelGyro[2]/16384.0-mpu6050_bias_accel[2];
	
	gyro[0]=AccelGyro[3]/131.0-mpu6050_bias_gyro[0];
	gyro[1]=AccelGyro[4]/131.0-mpu6050_bias_gyro[1];
	gyro[2]=AccelGyro[5]/131.0-mpu6050_bias_gyro[2];
		
	println("Accel:  [ %-8.3lf\t%-8.3lf\t%-8.3lf ]\t  Gyro:  [ %-8.3lf\t%-8.3lf\t%-8.3lf ]\t  Temperature:  [ %-5.2lf ]",accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2],temperature);
		
}

/**
 * @}
 */
