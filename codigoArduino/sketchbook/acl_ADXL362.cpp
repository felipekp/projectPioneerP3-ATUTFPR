
#include "acl_ADXL362.h"

#define SPI_CLOCK		4000000 //4Mhz

#define G_TO_MS2		0.00980665 // DEFAULT FOR READING IS +-2g, and, in this configuration, the sensibility is approximatelly 1000LSB per g.

const byte ACL_PWR_DEFAULT_CONTROL_CMD	= 0b00010010; //low noise and mesuarement mode
const byte ACL_INT_DRDY_CMD				= 0b00000001;


int acl_csPin;

byte acl_axisData[6];

void acl_init(int csPin)
{
	acl_csPin = csPin;

	acl_reset();

	byte defaultConfig = ACL_PWR_DEFAULT_CONTROL_CMD;
	acl_write(ACL_PWR_CONTROL_REG, 1, &defaultConfig);

	byte interruptConfig = ACL_INT_DRDY_CMD;
	acl_write(ACL_INTMAP1_REG, 1, &interruptConfig);
}

void acl_reset()
{
        byte cmd = ACL_RESET_CMD;
	acl_write(ACL_RESET_REG, 1, &cmd);
	// tem delay obrigatorio
 	delay(10);
}

void acl_write(byte regAddress, int num, byte* values)
{
	SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
	//TODO: ver se é MSBFIRST ou LSBFIRST
	digitalWrite(acl_csPin,LOW);

	SPI.transfer(ACL_WRITE); //fala que vou escrever
	
	SPI.transfer(regAddress); //fala qual o reg destino inicial
	
	for(int i=0; i < num; i++)
		SPI.transfer(values[i]);

	digitalWrite(acl_csPin, HIGH);
	SPI.endTransaction();
}

void acl_read(byte regAddress, byte* values, int num)
{
	SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
	//TODO: ver se é MSBFIRST ou LSBFIRST
	digitalWrite(acl_csPin,LOW);

	SPI.transfer(ACL_READ); //fala que vou escrever
	
	SPI.transfer(regAddress); //fala qual o reg destino inicial
	int i = 0;
	for(i=0; i < num; i++)
		values[i] = SPI.transfer(0x00);

	digitalWrite(acl_csPin, HIGH);
	SPI.endTransaction();
}

void acl_read_xyz(float *x, float *y, float *z)
{
	acl_read(ACL_AXIS_REG, acl_axisData, 6);
       
	*x = (float)((((int)acl_axisData[1]<<8)|(int)acl_axisData[0]))*G_TO_MS2;
	*y = (float)((((int)acl_axisData[3]<<8)|(int)acl_axisData[2]))*G_TO_MS2;
        *z = (float)((((int)acl_axisData[5]<<8)|(int)acl_axisData[4]))*G_TO_MS2;
}
