#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

// 모터 한 축 = RS485 통신 포트 + 방향(DE) 제어 핀
typedef struct {
	UART_HandleTypeDef *uart;  // RS485 통신 UART
	GPIO_TypeDef *port;        // 방향 핀 포트
	uint16_t pin;              // 방향 핀
} Motor;

extern Motor motorX;  // X축 (UART4, PC12)
extern Motor motorY;  // Y축 (UART5, PD13)

// PC 디버그 메시지 출력
void print(const char *s);

// 통신 확인. 응답 있으면 1
int motor_check(Motor *m);

// 1회 초기화 (위치모드 + 다단속도 + 서보 ON). 성공 1
int motor_init(Motor *m, int dir);

// 현재 절대위치 읽기. 성공 1
int motor_pos(Motor *m, int *out);

// 절대위치로 이동. speed_pct: 속도 퍼센트(10 = 100rpm). 성공 1
int motor_move(Motor *m, int speed_pct, int pos);

// 목표 도착까지 대기. 도착 1, 타임아웃 0
int motor_wait(Motor *mx, int xtarget, Motor *my, int ytarget, int timeout);

#endif
