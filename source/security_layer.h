/*
 * security_layer.h
 *
 *  Created on: 10 sep. 2024
 *      Author: Gustavo Flores
 */

#ifndef SECURITY_LAYER_H_
#define SECURITY_LAYER_H_

#include "stdint.h"
#include "security_layer_cfg.h"

typedef enum {
	packageSent_OK,
	packageSent_ERROR,
	packageReceive_OK,
	crc32_ERROR
} SL_result;

typedef struct {
	uint8_t frame[HeaderETH + DataLength];
	size_t length;
} framesTxRx;

void ENET_Initialization(void);

SL_result sendPackageWithSecurityLayer(framesTxRx* message);
SL_result receivePackageWithSecurityLayer(void);

void encryptPackage(framesTxRx* toEncrypt);
void decryptPackage(framesTxRx* toDecrypt);

uint32_t calculateCRC32(uint8_t* toCalculate, size_t length);

#endif /* SECURITY_LAYER_H_ */
