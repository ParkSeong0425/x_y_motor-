/*
 * fram.c
 *
 *  Created on: Jul 2, 2026
 *      Author: HWNOT
 */
#include "fram.h"

extern SPI_HandleTypeDef hspi3;

// MB85RS64PNF 명령어
#define WREN  0x06   // 쓰기 허용
#define WRITE 0x02   // 데이터 쓰기
#define READ  0x03   // 데이터 읽기

// CS 핀 (CubeMX에서 라벨을 FRAM_CS로 지정, 다르면 여기만 수정)
#define CS_LOW()  HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET)

void fram_read(uint16_t addr, void *buf, uint16_t len) {
	uint8_t cmd[3] = { READ, addr >> 8, addr & 0xFF };
	CS_LOW();
	HAL_SPI_Transmit(&hspi3, cmd, 3, 100);
	HAL_SPI_Receive(&hspi3, buf, len, 100);
	CS_HIGH();
}

void fram_write(uint16_t addr, void *buf, uint16_t len) {
	uint8_t wren = WREN;
	uint8_t cmd[3] = { WRITE, addr >> 8, addr & 0xFF };

	CS_LOW();                                   // WREN은 단독 프레임
	HAL_SPI_Transmit(&hspi3, &wren, 1, 100);
	CS_HIGH();

	CS_LOW();
	HAL_SPI_Transmit(&hspi3, cmd, 3, 100);
	HAL_SPI_Transmit(&hspi3, buf, len, 100);
	CS_HIGH();
}
