//
// Created by 34147 on 2024/1/15.
//

#ifndef ENGINEER_CHASSIS_ABOARD_CRC_H
#define ENGINEER_CHASSIS_ABOARD_CRC_H
#ifdef __cplusplus
extern "C" {
#endif

#include "string.h"
#include "stm32h7xx_hal.h"

void crc16_update(uint16_t *currect_crc, const uint8_t *src, uint32_t len);

uint8_t verify_crc8_check_sum(uint8_t *pchMessage, uint16_t dwLength);

uint8_t verify_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength);

uint8_t get_crc8_check_sum(uint8_t *pchMessage, uint16_t dwLength, uint8_t ucCRC8);

uint16_t get_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);

void append_crc8_check_sum(uint8_t *pchMessage, uint16_t dwLength);

void append_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength);

extern const uint8_t CRC8_INIT;
extern uint16_t CRC_INIT;

#ifdef __cplusplus
}
#endif
#endif //ENGINEER_CHASSIS_ABOARD_CRC_H
