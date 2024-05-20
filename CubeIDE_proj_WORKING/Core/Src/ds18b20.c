#include "ds18b20.h"
#include <stdio.h>
#include <string.h>
#include <stdio.h>

void DS18B20_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DS18B20_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct);
}

uint8_t DS18B20_Reset(void) {
    uint8_t response = 0;
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
    HAL_Delay(480);
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);
    HAL_Delay(80);
    response = HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN);
    HAL_Delay(400);
    return response;
}

void DS18B20_WriteBit(uint8_t bit) {
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
    HAL_Delay(bit ? 1 : 60);
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);
    HAL_Delay(bit ? 60 : 1);
}

uint8_t DS18B20_ReadBit(void) {
    uint8_t bit = 0;
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);
    HAL_Delay(15);
    bit = HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN);
    HAL_Delay(45);
    return bit;
}

void DS18B20_WriteByte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        DS18B20_WriteBit(byte & 0x01);
        byte >>= 1;
    }
}

uint8_t DS18B20_ReadByte(void) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte |= (DS18B20_ReadBit() << i);
    }
    return byte;
}

float DS18B20_ReadTemperature(void) {
    uint8_t temp_lsb, temp_msb;
    int16_t temp;
    char debug_msg[64];

    if (DS18B20_Reset() == 0) {
        return 0.0;
    }

    DS18B20_WriteByte(0xCC); // SKIP ROM
    DS18B20_WriteByte(0x44); // CONVERT T

    HAL_Delay(750); // Wait for conversion

    if (DS18B20_Reset() == 0) {
        return 0.0;
    }

    DS18B20_WriteByte(0xCC); // SKIP ROM
    DS18B20_WriteByte(0xBE); // READ SCRATCHPAD

    temp_lsb = DS18B20_ReadByte();
    temp_msb = DS18B20_ReadByte();

    temp = (temp_msb << 8) | temp_lsb;
    float temperature = (float)temp / 16.0;

    return temperature;
}
