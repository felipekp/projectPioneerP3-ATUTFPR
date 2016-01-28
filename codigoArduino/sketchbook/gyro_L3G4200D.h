/**
 * Funções e constantes para acesso do gyroscopio L3G4200D
 */
#ifndef GYRO_L3G4200D_H
#define GYRO_L3G4200D_H

#include "Arduino.h"
#include <SPI.h>

//Most used registers in GYRO L3G4200D
const byte GYRO_CTRL_REG1 	= 0x20;
const byte GYRO_CTRL_REG3	= 0x22;
const byte GYRO_AXIS_REG	= 0x28;

/**
 * Configurações padrões para o GYRO L3G4200D
 * Sample Frequency 			= 200Hz
 * Low Pass Filter Cut-off		= 25Hz
 * Power Mode					= NORMAL
 * Full scale					= 250dps
 * Interrupt DRDY				= ON
 */
void gyro_init(int csPin);

/**
 * Escreve um vetor de valores a partir do endereço especificado
 * 
 * regAddress 	: endereço inicial especificado, cada valor values[i] é escrito em regAddress + i.
 * num 		: tamanho do vetor de valores
 * values 		: vetor de valores 
 */
void gyro_write(byte regAddress, int num, byte* values);

/**
 * Le um vetor de valores a partir do endereço especificado
 * 
 * regAddress 	: endereço inicial especificado, cada valor values[i] é lido de regAddress + i.
 * values 		: vetor de valores 
 * num 		: tamanho do vetor de valores
 */
void gyro_read(byte regAddress, byte* values, int num);

/**
 * Função auxiliar para ler a velocidade nos eixos em rad/s
 */
void gyro_read_xyz(float *x, float *y, float *z);


#endif
