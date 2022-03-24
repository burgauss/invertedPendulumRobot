#pragma once
#include <stdint.h>

uint8_t i2cWrite(const uint8_t IMUAddress,uint8_t registerAddress, uint8_t data, bool sendStop);
uint8_t i2cWrite(const uint8_t IMUAddress,uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
uint8_t i2cRead(const uint8_t IMUAddress,uint8_t registerAddress, uint8_t *data, uint8_t nbytes);