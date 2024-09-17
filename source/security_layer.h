/*
 * security_layer.h
 *
 *  Created on: 10 sep. 2024
 *      Author: Gustavo Flores
 */

#ifndef SECURITY_LAYER_H_
#define SECURITY_LAYER_H_

#include "stdint.h"

typedef enum {
	packageSent_OK,
	packageSent_ERROR,
	packageReceive_OK,
	crc32_ERROR
} SL_result;

void ENET_Initialization(void);

SL_result sendPackageWithSecurityLayer(uint8_t* message);
SL_result receivePackageWithSecurityLayer(void);

void encryptPackage(uint8_t* toEncrypt);
void decryptPackage(uint8_t* toDecrypt);

uint32_t calculateCRC32(uint8_t* toCalculate, size_t length);

#endif /* SECURITY_LAYER_H_ */
