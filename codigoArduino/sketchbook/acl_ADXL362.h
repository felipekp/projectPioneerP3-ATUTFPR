/**
 * Funções e constantes para acesso do acelerometro ADXL362
 */
#ifndef ACL_ADXL362_H
#define ACL_ADXL362_H

#include "Arduino.h"
#include <SPI.h>

//Most used registers in ACL ADXL362
const byte ACL_RESET_REG 				= 0x1F;
const byte ACL_RESET_CMD 				= 0x52;
const byte ACL_READ 					= 0x0B;
const byte ACL_WRITE 					= 0x0A;
const byte ACL_AXIS_REG					= 0x0E;

const byte ACL_PWR_CONTROL_REG 			= 0x2D;
//const byte ACL_PWR_DEFAULT_CONTROL_CMD	= 0x12; //low noise and mesuarement mode

const byte ACL_INTMAP1_REG				= 0x2A;
//const byte ACL_INT_DRDY_CMD				= 0x01;

/**
 * Configurações padrões para o ACL ADXL362
 * Sample Frequency 			= 100Hz
 * Antialising					= 1/4 = 25Hz = ODR/4
 * Power Mode					= Low noise
 * Full scale					= +-2g
 * Interrupt DRDY				= ON
 * Interrupt PIN 				= INTMAP1 
 */
void acl_init(int csPin);

/**
 * Reseta o valor dos registradores acelerometro; função tem delay de 10ms
 */
void acl_reset();

/**
 * 
 */
void acl_write(byte regAddress, int num, byte* values);

/**
 * 
 */
void acl_read(byte regAddress, byte* values, int num);

/**
 * 
 */
void acl_read_xyz(float *x, float *y, float *z);

#endif
