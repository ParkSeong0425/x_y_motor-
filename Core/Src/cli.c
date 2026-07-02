#include "cli.h"
#include "motor.h"
#include "fram.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

#define HOME_XPOS 0
#define HOME_YPOS 0
#define HOME_RPM 100
#define MOVE_TIMEOUT 10000

// FRAM 주소 0번지에 저장되는 데이터
typedef struct {
	int x[4];    // X축 열 위치 (1~4열)
	int y[4];    // Y축 단 위치 (1~4단)
} Data;

static Data data;
static int loaded = 0;

static void read_line(char *buf, int max) {
	int i = 0;
	while (i < max - 1) {
		uint8_t c;
		if (HAL_UART_Receive(&huart3, &c, 1, HAL_MAX_DELAY) != HAL_OK)
			continue;
		if (c == '\r' || c == '\n') {
			if (i > 0)
				break;
			continue;
		}
		if (c == 0x08 || c == 0x7F) {        // 백스페이스: 한 글자 지움
			if (i > 0) {
				i--;
				print("\b \b");
			}
			continue;
		}
		buf[i++] = c;
	}
	buf[i] = 0;
}

static void show(void) {
	char log[80];
	sprintf(log, "x %d %d %d %d\r\ny %d %d %d %d\r\n", data.x[0], data.x[1],
			data.x[2], data.x[3], data.y[0], data.y[1], data.y[2], data.y[3]);
	print(log);
}

static void print_menu(void) {
	print("\r\nInput\r\n");
	print("x v1 v2 v3 v4\r\n");
	print("y v1 v2 v3 v4\r\n");
	print("go x(열) y(단) rpm\r\n");
	print("show\r\n");
	print("reset\r\n");
	print("home\r\n");
	print(": ");
}

// X, Y 모터에 이동 명령을 같이 보내고 같이 도착을 기다림
static void move_xy(int rpm, int xt, int yt) {
	motor_move(&motorX, rpm, xt);
	motor_move(&motorY, rpm, yt);
	motor_wait(&motorX, xt, &motorY, yt, MOVE_TIMEOUT);
}

void cli_run(void) {
	char line[48];
	int a, b, c, d;

	if (!loaded) {                       // 첫 실행 때 FRAM 데이터 로드
		fram_read(0, &data, sizeof(data));
		loaded = 1;
		print("\r\n");
		show();
		print_menu();
	}
	print(": ");
	read_line(line, sizeof(line));
	print("\r\n");

	if (sscanf(line, "go %d %d %d", &a, &b, &c) == 3) {   // go 열 단 rpm
		move_xy(c, data.x[a - 1], data.y[b - 1]);
		print("Done\r\n");
		return;
	}

	if (sscanf(line, "x %d %d %d %d", &a, &b, &c, &d) == 4) {
		data.x[0] = a;
		data.x[1] = b;
		data.x[2] = c;
		data.x[3] = d;
		fram_write(0, &data, sizeof(data));
		print("save\r\n");
		return;
	}

	if (sscanf(line, "y %d %d %d %d", &a, &b, &c, &d) == 4) {
		data.y[0] = a;
		data.y[1] = b;
		data.y[2] = c;
		data.y[3] = d;
		fram_write(0, &data, sizeof(data));
		print("save\r\n");
		return;
	}

	if (strcmp(line, "show") == 0) {
		show();
		return;
	}

	if (strcmp(line, "reset") == 0) {
		memset(&data, 0, sizeof(data));
		fram_write(0, &data, sizeof(data));
		show();
		return;
	}

	if (strcmp(line, "home") == 0) {
		print("Homing...\r\n");
		move_xy(HOME_RPM, HOME_XPOS, HOME_YPOS);
		print("Done\r\n");
		return;
	}

	print("Wrong input\r\n");
}
