#include "cli.h"
#include "motor.h"
#include "fram.h"
#include "button.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

#define MOVE_TIMEOUT 60000   // 10초는 홈 이동(먼 거리) 중에 무조건 걸림

#define DATA_VER 0x0001   // 고정값, 데이터 구조 바뀔 때만 손으로 올림

#define REV_MM  160
#define REV_CNT 131072
#define PULSE(v) ((v) * REV_CNT)
#define MM(v)    ((v) * REV_MM / REV_CNT)

#define X_MAX (60 * REV_CNT)
#define Y_MAX (-(64 * REV_CNT))

typedef struct {
	int ver;
	int x[4];
	int y[4];   // raw pulse 그대로 저장 (부호 반전 없음)
	int home_x;
	int home_y;
	int last_x;   // 마지막 위치 (전원 꺼도 기억)
	int last_y;
} Data;

static Data data;
static int loaded = 0;

// 현재 위치를 FRAM에 저장 -> 전원 꺼도 위치 안 잃어버림
static void save_last(void) {
	motor_pos(&motorX, &data.last_x);
	motor_pos(&motorY, &data.last_y);
	fram_write(0, &data, sizeof(data));
}

int cli_homed(void) {
	return data.home_x != 0 || data.home_y != 0;
}

int cli_getc(uint8_t *c) {
	return HAL_UART_Receive(&huart3, c, 1, 100) == HAL_OK;
}

static void read_line(char *buf, int max) {
	int i = 0;
	while (i < max - 1) {
		uint8_t c;
		if (!cli_getc(&c)) {
			button_run();
			continue;
		}
		if (c == '\r' || c == '\n') {
			if (i > 0)
				break;
			continue;
		}
		if (c == 0x08 || c == 0x7F) {
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
	char log[120];
	sprintf(log, "x %dmm %dmm %dmm %dmm\r\ny %dmm %dmm %dmm %dmm\r\n",
			MM(data.x[0]), MM(data.x[1]), MM(data.x[2]), MM(data.x[3]),
			MM(data.y[0]), MM(data.y[1]), MM(data.y[2]), MM(data.y[3]));
	print(log);
}

static void print_menu(void) {
	print("\r\nInput\r\n");
	print("x4 save -> x3 save -> x2 save -> x1 save\r\n");
	print("y1 save -> y2 save -> y3 save -> y4 save\r\n");
	print("go x(열) y(단)\r\n");
	print("show\r\n");
	print("reset\r\n");
	print("stop\r\n");
	print("xhome | yhome | home\r\n");
	print(": ");
}

void cli_set_x(int n, int pos) {
	data.x[n - 1] = pos;   // raw pulse
	save_last();
}

void cli_set_y(int n, int pos) {
	data.y[n - 1] = pos;   // raw pulse
	save_last();
}

// X, Y 모터에 이동 명령을 같이 보내고 같이 도착을 기다림
// 트리거 자체가 실패하면 기다릴 필요 없이 바로 취소
static int move_xy(int xrpm, int yrpm, int xt, int yt) {
	int okx = motor_move(&motorX, xrpm, xt);
	int oky = motor_move(&motorY, yrpm, yt);

	if (!okx)
		print("X cmd fail\r\n");
	if (!oky)
		print("Y cmd fail\r\n");
	if (!okx || !oky)
		return 0;

	int ok = motor_wait(&motorX, xt, &motorY, yt, MOVE_TIMEOUT);
	save_last();
	return ok;
}

int cli_go_home(void) {
	pause = 0;
	return move_xy(100, 100, data.home_x, data.home_y);
}

int cli_go(int col, int dan, int xrpm, int yrpm) {
	int xt = data.x[col - 1];
	int yt = data.y[dan - 1];

	if (xt == 0 || yt == 0) {
		print("Need save\r\n");
		return 0;
	}
	return move_xy(xrpm, yrpm, xt, yt);
}

void cli_run(void) {
	char line[48];
	int a, b;

	if (!loaded) {
		button_init();
		HAL_Delay(1000);
		motor_init(&motorX, 0);
		HAL_Delay(200);
		motor_init(&motorY, 1);
		fram_read(0, &data, sizeof(data));
		if (data.ver != DATA_VER) {
			memset(&data, 0, sizeof(data));
			data.ver = DATA_VER;
			fram_write(0, &data, sizeof(data));
		}
		motor_sync(&motorX, data.last_x);   // 전원 꺼지기 전 좌표계로 복원
		motor_sync(&motorY, data.last_y);
		loaded = 1;
		print("\r\n");
		show();
		print_menu();
	}

	print(": ");
	read_line(line, sizeof(line));
	print("\r\n");

	if ((sscanf(line, "x%d save", &a) == 1 || sscanf(line, "x%dsave", &a) == 1)
			&& a >= 1 && a <= 4) {
		button_save_x(a);
		show();
		return;
	}

	if ((sscanf(line, "y%d save", &a) == 1 || sscanf(line, "y%dsave", &a) == 1)
			&& a >= 1 && a <= 4) {
		button_save_y(a);
		show();
		return;
	}

	if (sscanf(line, "go %d %d", &a, &b) == 2) {
		cli_go(a, b, 750, 1000);
		return;
	}

	if (strcmp(line, "show") == 0) {
		show();
		return;
	}

	if (strcmp(line, "reset") == 0) {
		memset(&data, 0, sizeof(data));
		data.ver = DATA_VER;
		save_last();
		show();
		return;
	}

	if (strcmp(line, "stop") == 0) {
		pause = 1;
		HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_RESET);
		return;
	}

	if (strcmp(line, "xhome") == 0) {
		int p = 0;
		if (button_home_x(&p)) {
			data.home_x = p;
			save_last();
		}
		return;
	}

	if (strcmp(line, "yhome") == 0) {
		int p = 0;
		if (button_home_y(&p)) {
			data.home_y = p;
			save_last();
		}
		return;
	}

	if (strcmp(line, "home") == 0) {
		cli_go_home();
		return;
	}

	print("Wrong input\r\n");
}
