#include "motor.h"
#include "button.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

Motor motorX = { &huart4, rs485_GPIO_Port, rs485_Pin };
Motor motorY = { &huart5, rs4852_GPIO_Port, rs4852_Pin };

#define R_MODE    0x0200
#define R_DIR     0x0202
#define R_ENABLE  0x0303
#define R_DI2     0x0304
#define R_DI5     0x030A
#define R_DI5L    0x030B
#define R_START   0x0305
#define R_DI4     0x0308
#define R_MUL     0x0309
#define R_SRC     0x0500
#define R_RUN     0x1100
#define R_REG     0x1101
#define R_BEGIN   0x1102
#define R_TYPE    0x1104
#define R_POS     0x110C
#define R_SPEED   0x110E
#define R_ACC     0x110F
#define R_WAIT    0x1110
#define R_REALPOS 0x0B07
#define R_DI      0x0B03

#define MOTOR_ID   0x01
#define ARRIVE_GAP 100      // 5는 0.006mm라 Y(수직축)는 정착 못해서 타임아웃 남
#define ACCDEC     1000
#define WAIT_MS    500

void print(const char *s) {
	HAL_UART_Transmit(&huart3, (uint8_t*) s, strlen(s), 100);
}

static uint16_t crc16(uint8_t *data, uint16_t len) {
	uint16_t crc = 0xFFFF;
	for (uint16_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t j = 0; j < 8; j++)
			crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
	}
	return crc;
}

static void bus_tx(Motor *m) {
	HAL_GPIO_WritePin(m->port, m->pin, GPIO_PIN_SET);
	HAL_Delay(1);
}

static void bus_rx(Motor *m) {
	while (__HAL_UART_GET_FLAG(m->uart, UART_FLAG_TC) == RESET)
		;
	for (volatile uint32_t i = 0; i < 100; i++)
		;
	HAL_GPIO_WritePin(m->port, m->pin, GPIO_PIN_RESET);
}

static void bus_clear(Motor *m) {
	__HAL_UART_CLEAR_OREFLAG(m->uart);
	__HAL_UART_CLEAR_FEFLAG(m->uart);
	__HAL_UART_CLEAR_NEFLAG(m->uart);
	__HAL_UART_CLEAR_PEFLAG(m->uart);
	while (__HAL_UART_GET_FLAG(m->uart, UART_FLAG_RXNE) != RESET) {
		volatile uint8_t d = m->uart->Instance->DR;
		(void) d;
	}
}

static HAL_StatusTypeDef bus_xfer(Motor *m, uint8_t *tx, uint16_t tlen,
		uint8_t *rx, uint16_t rlen) {
	bus_clear(m);
	bus_tx(m);
	HAL_StatusTypeDef r = HAL_UART_Transmit(m->uart, tx, tlen, 100);
	bus_rx(m);
	if (r == HAL_OK)
		r = HAL_UART_Receive(m->uart, rx, rlen, 500);
	HAL_Delay(20);
	return r;
}

static int write16(Motor *m, uint16_t reg, uint16_t val) {
	uint8_t tx[8], rx[8];
	tx[0] = MOTOR_ID;
	tx[1] = 0x06;
	tx[2] = reg >> 8;
	tx[3] = reg & 0xFF;
	tx[4] = val >> 8;
	tx[5] = val & 0xFF;
	uint16_t crc = crc16(tx, 6);
	tx[6] = crc & 0xFF;
	tx[7] = crc >> 8;
	if (bus_xfer(m, tx, 8, rx, 8) != HAL_OK)
		return 0;
	return (rx[0] == MOTOR_ID && rx[1] == 0x06);
}

int motor_write(Motor *m, uint16_t reg, uint16_t val) {
	return write16(m, reg, val);
}

static int write32(Motor *m, uint16_t reg, int val) {
	uint8_t tx[13], rx[8];
	uint32_t raw = (uint32_t) val;
	uint16_t low = raw & 0xFFFF;
	uint16_t high = raw >> 16;
	tx[0] = MOTOR_ID;
	tx[1] = 0x10;
	tx[2] = reg >> 8;
	tx[3] = reg & 0xFF;
	tx[4] = 0x00;
	tx[5] = 0x02;
	tx[6] = 0x04;
	tx[7] = low >> 8;
	tx[8] = low & 0xFF;
	tx[9] = high >> 8;
	tx[10] = high & 0xFF;
	uint16_t crc = crc16(tx, 11);
	tx[11] = crc & 0xFF;
	tx[12] = crc >> 8;
	if (bus_xfer(m, tx, 13, rx, 8) != HAL_OK)
		return 0;
	return (rx[0] == MOTOR_ID && rx[1] == 0x10);
}

static uint16_t read16(Motor *m, uint16_t reg) {
	uint8_t tx[8], rx[7];
	tx[0] = MOTOR_ID;
	tx[1] = 0x03;
	tx[2] = reg >> 8;
	tx[3] = reg & 0xFF;
	tx[4] = 0x00;
	tx[5] = 0x01;
	uint16_t crc = crc16(tx, 6);
	tx[6] = crc & 0xFF;
	tx[7] = crc >> 8;
	if (bus_xfer(m, tx, 8, rx, 7) != HAL_OK)
		return 0xFFFF;
	if (rx[0] != MOTOR_ID || rx[1] != 0x03)
		return 0xFFFF;
	return (rx[3] << 8) | rx[4];
}

static int read32(Motor *m, uint16_t reg, int *out) {
	uint8_t tx[8], rx[9];
	tx[0] = MOTOR_ID;
	tx[1] = 0x03;
	tx[2] = reg >> 8;
	tx[3] = reg & 0xFF;
	tx[4] = 0x00;
	tx[5] = 0x02;
	uint16_t crc = crc16(tx, 6);
	tx[6] = crc & 0xFF;
	tx[7] = crc >> 8;
	if (bus_xfer(m, tx, 8, rx, 9) != HAL_OK)
		return 0;
	if (rx[0] != MOTOR_ID || rx[1] != 0x03 || rx[2] != 0x04)
		return 0;
	uint16_t low = (rx[3] << 8) | rx[4];
	uint16_t high = (rx[5] << 8) | rx[6];
	*out = (int) (((uint32_t) high << 16) | low);
	return 1;
}

int motor_check(Motor *m) {
	return read16(m, R_DI) != 0xFFFF;
}

int motor_init(Motor *m, int dir) {
	write16(m, R_MODE, 1);
	HAL_Delay(20);
	write16(m, R_DIR, dir);
	HAL_Delay(20);
	write16(m, R_DI1, 0);
	HAL_Delay(20);
	write16(m, R_DI2, 1);
	HAL_Delay(20);
	write16(m, R_DI3, 0);
	HAL_Delay(20);
	write16(m, R_DI4, 0);
	HAL_Delay(20);
	write16(m, R_DI5, 0);
	HAL_Delay(20);
	write16(m, R_DI1, 18);
	HAL_Delay(20);
	write16(m, R_DI1L, 0);
	HAL_Delay(20);
	write16(m, R_DI3, 19);
	HAL_Delay(20);
	write16(m, R_DI3L, 0);
	HAL_Delay(20);
	write16(m, R_DI5, 34);
	HAL_Delay(20);
	write16(m, R_DI5L, 0);
	HAL_Delay(20);
	write16(m, R_DI4, 28);
	HAL_Delay(20);
	write16(m, R_MUL, 0);
	HAL_Delay(20);
	write16(m, R_SRC, 2);
	HAL_Delay(20);
	write16(m, R_RUN, 0);
	HAL_Delay(20);
	write16(m, R_REG, 1);
	HAL_Delay(20);
	write16(m, R_BEGIN, 0);
	HAL_Delay(20);
	write16(m, JOG_SPEED, JOG_RPM);
	HAL_Delay(20);
	return write16(m, R_START, 1);
}

// 드라이버 REALPOS는 전원 끄면 0으로 리셋됨
// off = FRAM에 저장한 마지막 위치 - 지금 REALPOS -> 예전 좌표계 그대로 이어짐
void motor_sync(Motor *m, int last) {
	int raw = 0;
	read32(m, R_REALPOS, &raw);
	m->off = last - raw;
}

int motor_pos(Motor *m, int *out) {
	int raw = 0;
	if (!read32(m, R_REALPOS, &raw))
		return 0;
	*out = raw + m->off;
	return 1;
}

int motor_move(Motor *m, int rpm, int target) {
	int now = 0;
	if (!motor_pos(m, &now) && !motor_pos(m, &now))
		return 0;                       // 위치 읽기 실패 -> 엉뚱한 delta로 가지 않게 중단
	int delta = target - now;

	write16(m, R_MUL, 0);
	write16(m, R_TYPE, 1);
	write32(m, R_POS, delta);
	write16(m, R_SPEED, rpm);
	write16(m, R_ACC, ACCDEC);
	write16(m, R_WAIT, WAIT_MS);
	if (write16(m, R_MUL, 1))
		return 1;
	return write16(m, R_MUL, 1);        // 트리거 실패하면 1번 재시도
}

int motor_stop(Motor *m) {
	return write16(m, R_MUL, 0);
}

int motor_begin(Motor *m, int v) {
	return write16(m, R_BEGIN, v);
}

int motor_di5(Motor *m, int v) {
	return write16(m, R_DI5L, v);
}

int motor_wait(Motor *mx, int xtarget, Motor *my, int ytarget, int timeout) {
	uint32_t t0 = HAL_GetTick();
	int nowx = 0, nowy = 0;
	int xdone = 0, ydone = 0;

	while (HAL_GetTick() - t0 < (uint32_t) timeout) {
		if (pause || estop) {
			motor_stop(mx);
			motor_stop(my);
			print("Stop\r\n");
			return 0;
		}
		if (!xdone && motor_pos(mx, &nowx)) {
			int gap = nowx - xtarget;
			if (gap < 0)
				gap = -gap;
			if (gap <= ARRIVE_GAP)
				xdone = 1;
		}
		if (!ydone && motor_pos(my, &nowy)) {
			int gap = nowy - ytarget;
			if (gap < 0)
				gap = -gap;
			if (gap <= ARRIVE_GAP)
				ydone = 1;
		}
		if (xdone && ydone) {
			print("Done\r\n");
			return 1;
		}
	}
	motor_stop(mx);
	motor_stop(my);
	print("Timeout\r\n");
	return 0;
}
