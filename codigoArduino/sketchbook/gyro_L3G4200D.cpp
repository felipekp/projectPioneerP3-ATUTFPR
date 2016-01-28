
#include "gyro_L3G4200D.h"

//GYRO COMMANDS

#define SPI_CLOCK		4000000 //4Mhz

// sensibilidade * pi / 180
#define DPS_TO_RADPS 	0.0001527163;

const byte GYRO_REG1_SR_200HZ		= 0b01000000;
const byte GYRO_REG1_25HZ_CUTOFF	= 0b00010000;
const byte GYRO_REG1_ENABLE_XYZ		= 0b00001111;

const byte GYRO_ENABLE_DRDY			= 0b00001000;

const byte GYRO_READ_MASK 			= 0b10000000;
const byte GYRO_WRITE_MASK 			= 0b01111111;
const byte GYRO_MULTIBYTE_MASK 		= 0b01000000;
const byte lastByte 				= 0b10111111;

int gyro_csPin;

byte gyro_axisData[6];

void gyro_init(int csPin)
{
	gyro_csPin = csPin;

	byte defaultConfig = GYRO_REG1_SR_200HZ |
						 GYRO_REG1_25HZ_CUTOFF |
						 GYRO_REG1_ENABLE_XYZ;

	gyro_write(GYRO_CTRL_REG1, 1, &defaultConfig);

	byte interruptConfig = GYRO_ENABLE_DRDY;
	gyro_write(GYRO_CTRL_REG3, 1, &interruptConfig);
}

void gyro_write(byte regAddress, int num, byte* values)
{
	SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
	digitalWrite(gyro_csPin, LOW); //chip enable

	regAddress &= GYRO_WRITE_MASK;

	if(num > 1) regAddress |= GYRO_MULTIBYTE_MASK; 

	SPI.transfer(regAddress);
	for(int i=0; i < num; i++)
		SPI.transfer(values[i]);

	digitalWrite(gyro_csPin, HIGH);
	SPI.endTransaction();
}

void gyro_read(byte regAddress, byte* values, int num)
{
	SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
	digitalWrite(gyro_csPin, LOW); //chip enable

	regAddress |= GYRO_READ_MASK;

	if(num > 1) regAddress |= GYRO_MULTIBYTE_MASK;

	SPI.transfer(regAddress);
	for(int i=0; i < num; i++)
		values[i] = SPI.transfer(0x00);	

	digitalWrite(gyro_csPin, HIGH);
	SPI.endTransaction();
}

void gyro_read_xyz(float *x, float *y, float *z)
{
	gyro_read(GYRO_AXIS_REG, gyro_axisData, 6);

	*x = (float)((((int)gyro_axisData[1]<<8)|(int)gyro_axisData[0]))*DPS_TO_RADPS;
	*y = (float)((((int)gyro_axisData[3]<<8)|(int)gyro_axisData[2]))*DPS_TO_RADPS;
	*z = (float)((((int)gyro_axisData[5]<<8)|(int)gyro_axisData[4]))*DPS_TO_RADPS;
}
