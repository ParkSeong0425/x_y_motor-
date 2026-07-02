#include "motor.h"
#include <stdio.h>
#include <string.h>

// 외부 usart.c에 선언된 UART 주변장치 핸들 참조
extern UART_HandleTypeDef huart3;  // PC 디버그 통신용 (터미널)
extern UART_HandleTypeDef huart4;  // X축 모터 RS485 통신용
extern UART_HandleTypeDef huart5;   // Y축 모터 RS485

// X, Y 모터 구조체 초기화 (UART 핸들, RS485 방향 제어 포트 및 핀 매핑)
Motor motorX = { &huart4, rs485_GPIO_Port, rs485_Pin };
Motor motorY = { &huart5, rs4852_GPIO_Port, rs4852_Pin };
// ===== 모드버스 레지스터 매핑 (주소 = 그룹번호 * 256 + 인덱스) =====
#define R_MODE    0x0200  // 제어 모드 설정 (1: 위치 제어 모드)
#define R_DIR     0x0202  // ★ 추가: 회전 방향 선택 (0: CCW 정방향, 1: CW 정방향)
#define R_DI1     0x0302  // DI1 디지털 입력 기능 설정 (0: no effect)
#define R_ENABLE  0x0303
#define R_DI2     0x0304  // DI2 디지털 입력 기능 설정 (1: 서보ON)
#define R_START   0x0305  // 서보ON
#define R_DI3     0x0306  // DI3 디지털 입력 기능 설정 (0: no effect)
#define R_ENABLE2 0x0307
#define R_DI4     0x0308  // DI4 디지털 입력 기능 설정 (28: 위치 모드 )
#define R_MUL     0x0309  // 위치 모드 ON
#define R_DI5     0x0310  // DI5 디지털 입력 기능 설정 (34: 긴급정지 )
#define R_ESTOP   0x0311  //
#define R_SRC     0x0500  // 위치 지령 소스 설정 (2: 다단 위치 운전 모드)
#define R_RUN     0x1100  // 운전 방식 모드 (0: 1회 운전 후 정지)
#define R_REG     0x1101  // 래지스터 개수
#define R_BEGIN   0x1102  // 시작할때 위치 (1: 첨부터 , 0: 멈춘 곳부터)
#define R_TYPE    0x1104  // 구간1 이동 타입 설정 (1: 절대위치, 0: 상대위치)
#define R_POS     0x110C  // 구간1 목표 위치 데이터 (32비트 크기)
#define R_SPEED   0x110E  // 구간1 운전 속도 (RPM 단위)
#define R_ACC     0x110F  // 구간1 가감속 시간 (ms 단위)
#define R_WAIT    0x1110  // 구간1 목표 도달 후 대기 시간 (ms 단위)
#define R_REALPOS 0x0B07  // 모터의 현재 실제 절대 위치 피드백 (32비트 크기)
#define R_DI      0x0B03  // 디지털 입력(DI) 상태 모니터링 레지스터

#define MOTOR_ID   0x01   // 국번 (모터 아이디 고정값 1)
#define ARRIVE_GAP 5      // 목표 위치와 현재 위치의 오차가 5 이내면 도착 판단
#define ACCDEC     1000   // 가감속 시간 기본값 (1000ms)
#define WAIT_MS    500    // 대기 시간 기본값 (500ms)

// ===== PC 터미널 출력 함수 =====
void print(const char *s) {
	HAL_UART_Transmit(&huart3, (uint8_t*) s, strlen(s), 100);
}

// ===== 모드버스 CRC16 체크섬 계산 함수 =====
static uint16_t crc16(uint8_t *data, uint16_t len) {
	uint16_t crc = 0xFFFF;
	for (uint16_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t j = 0; j < 8; j++)
			crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
	}
	return crc;
}

// ===== RS485 송수신 방향 전환 제어 =====
static void bus_tx(Motor *m) {
	HAL_GPIO_WritePin(m->port, m->pin, GPIO_PIN_SET);
	HAL_Delay(1); // 드라이버 안정화 시간
}

static void bus_rx(Motor *m) {
	// 1. 하드웨어에서 마지막 바이트의 물리적 전송이 완료될 때까지 대기
	while (__HAL_UART_GET_FLAG(m->uart, UART_FLAG_TC) == RESET)
		;

	// ★ 핵심 수정: 180MHz 고속 MCU에서 i < 20은 너무 짧아 스톱 비트가 잘립니다.
	// 딜레이 카운트를 늘려 마지막 스톱 비트까지 선로에 완전히 태운 후 수신 모드로 전환합니다.
	for (volatile uint32_t i = 0; i < 100; i++)
		;

	HAL_GPIO_WritePin(m->port, m->pin, GPIO_PIN_RESET);
}

static void bus_clear(Motor *m) {
	__HAL_UART_CLEAR_OREFLAG(m->uart);
	__HAL_UART_CLEAR_FEFLAG(m->uart);
	__HAL_UART_CLEAR_NEFLAG(m->uart);
	__HAL_UART_CLEAR_PEFLAG(m->uart);

	// 강제 상태 변경 코드(gState/RxState)는 삭제

	while (__HAL_UART_GET_FLAG(m->uart, UART_FLAG_RXNE) != RESET) {
		volatile uint8_t d = m->uart->Instance->DR;
		(void) d;
	}
}
static HAL_StatusTypeDef bus_xfer(Motor *m, uint8_t *tx, uint16_t tlen,
		uint8_t *rx, uint16_t rlen) {
	bus_clear(m); // 전송 전 무조건 상태 초기화 및 버퍼 비우기
	bus_tx(m);
	HAL_StatusTypeDef r = HAL_UART_Transmit(m->uart, tx, tlen, 100);
	bus_rx(m);

	if (r == HAL_OK) {
		r = HAL_UART_Receive(m->uart, rx, rlen, 500); // 500ms 타임아웃 대기
	}
	HAL_Delay(20);
	return r;
}

// ===== 16비트 단일 레지스터 쓰기 =====
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

// ===== 32비트 다중 레지스터 쓰기 =====
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

// ===== 16비트 단일 레지스터 읽기 =====
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

// ===== 32비트 연속 레지스터 읽기 =====
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

// ===== 모터 초기 파라미터 셋팅 및 서보 ON 설정 =====
int motor_init(Motor *m, int dir) {
	if (!write16(m, R_MODE, 1))
		return 0;
	if (!write16(m, R_DIR, dir))
		return 0;
	if (!write16(m, R_DI1, 0))
		return 0;
	if (!write16(m, R_DI2, 1))
		return 0;      // DI2 기능 = 서보ON
	if (!write16(m, R_DI4, 28))
		return 0;     // DI4 기능 = 이동 트리거
	if (!write16(m, R_MUL, 0))
		return 0;      // 트리거 상태 초기 클리어
	if (!write16(m, R_SRC, 2))
		return 0;
	if (!write16(m, R_RUN, 0))
		return 0;
	if (!write16(m, R_REG, 1))
		return 0;
	if (!write16(m, R_BEGIN, 0))
		return 0;

	return write16(m, R_START, 1);   // 서보ON, 이후 계속 유지
}

int motor_pos(Motor *m, int *out) {
	return read32(m, R_REALPOS, out);
}

// ===== 목표 위치 구동 명령 실행 API =====
int motor_move(Motor *m, int speed_pct, int target) {
	int rpm = speed_pct * 10;

	int now = 0;
	if (!motor_pos(m, &now))
		return 0;
	int delta = target - now;

	if (!write16(m, R_MUL, 0))       // 트리거 클리어
		return 0;
	HAL_Delay(20);

	if (!write16(m, R_TYPE, 1))
		return 0;
	HAL_Delay(20);

	if (!write32(m, R_POS, delta))
		return 0;
	if (!write16(m, R_SPEED, rpm))
		return 0;
	if (!write16(m, R_ACC, ACCDEC))
		return 0;
	if (!write16(m, R_WAIT, WAIT_MS))
		return 0;

	return write16(m, R_MUL, 1);     // 이동 트리거 발동
}
// ===== X, Y 두 모터를 같이 지켜보며 둘 다 도착할 때까지 대기 =====
int motor_wait(Motor *mx, int xtarget, Motor *my, int ytarget, int timeout) {
	uint32_t t0 = HAL_GetTick();
	int nowx = 0, nowy = 0;
	int xdone = 0, ydone = 0;

	while (HAL_GetTick() - t0 < (uint32_t) timeout) {
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
		HAL_Delay(200);
	}
	print("Timeout\r\n");
	return 0;
}
