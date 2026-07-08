#include "cli.h"
#include "motor.h"
#include "fram.h"
#include "button.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

#define MOVE_TIMEOUT 60000

#define DATA_VER 0x0003   // 구조체 바뀔 때만 손으로 올림 (home_ok 필드 추가됨)

#define REV_MM  160
#define REV_CNT 131072
#define MM(v) ((v) * REV_MM / REV_CNT)

typedef struct {
	int ver;
	int x[4];
	int y[4];   // raw pulse
	int home_x;   // 항상 0 (원점복귀 성공 시 드라이버가 0으로 클리어)
	int home_y;   // 항상 0
	int last_x;
	int last_y;
	int home_ok;  // bit0=X home 완료, bit1=Y home 완료
} Data;

static Data data;
static int loaded = 0;
static int x_alive = 1;
static int y_alive = 1;

static void save_all(void) {
	fram_write(0, &data, sizeof(data));
}

static void save_last(void) {
	motor_pos(&motorX, &data.last_x);
	motor_pos(&motorY, &data.last_y);
	save_all();
}

// 모터 드라이버가 (STM32 전원과 별개로) 껐다 켜진 걸 감지.
// 예전처럼 FRAM 위치로 억지로 offset을 재계산하지 않고,
// 그냥 "다시 home 해야 함"으로 표시한다. 원점센서 기반 원점복귀를 쓰므로
// 이렇게 하는 게 훨씬 안전하다 (매번 실제 센서 기준으로 다시 잡기 때문).
void cli_check_power(void) {
	static uint32_t last_check = 0;
	uint32_t now = HAL_GetTick();
	if (now - last_check < 2000)
		return;
	last_check = now;

	int alive = motor_check(&motorX);
	if (alive && !x_alive) {
		data.home_ok &= ~0x01;
		save_all();
		print("X repowered, redo xhome\r\n");
	}
	x_alive = alive;

	alive = motor_check(&motorY);
	if (alive && !y_alive) {
		data.home_ok &= ~0x02;
		save_all();
		print("Y repowered, redo yhome\r\n");
	}
	y_alive = alive;
}

int cli_homed(void) {
	return (data.home_ok & 0x03) == 0x03;
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
	char log[160];
	sprintf(log, "x %dmm %dmm %dmm %dmm\r\n"
			"y %dmm %dmm %dmm %dmm\r\n"
			"home_ok=%d last_x=%d last_y=%d\r\n", MM(data.x[0]), MM(data.x[1]),
			MM(data.x[2]), MM(data.x[3]), MM(data.y[0]), MM(data.y[1]),
			MM(data.y[2]), MM(data.y[3]), data.home_ok, data.last_x,
			data.last_y);
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
	data.x[n - 1] = pos;
	save_last();
}

void cli_set_y(int n, int pos) {
	data.y[n - 1] = pos;
	save_last();
}

static int in_range(int v, int a, int b) {
	int lo = a < b ? a : b;
	int hi = a < b ? b : a;
	return v >= lo && v <= hi;
}

// home은 항상 0이므로 유효범위는 [0, x1] / [0, y4].
// 아직 그쪽 축을 홈/저장 안 했으면(0이면) 검사를 생략한다.
static int check_range(void) {
	int nowx = 0, nowy = 0;

	if ((data.home_ok & 0x01) && data.x[0] != 0) {
		motor_pos(&motorX, &nowx);
		if (!in_range(nowx, 0, data.x[0])) {
			print("X out of range, redo xhome\r\n");
			return 0;
		}
	}
	if ((data.home_ok & 0x02) && data.y[3] != 0) {
		motor_pos(&motorY, &nowy);
		if (!in_range(nowy, 0, data.y[3])) {
			print("Y out of range, redo yhome\r\n");
			return 0;
		}
	}
	return 1;
}

// X, Y 모터에 이동 명령을 같이 보내고 같이 도착을 기다림
static int move_xy(int xrpm, int yrpm, int xt, int yt) {
	if (!check_range())
		return 0;

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

// home은 이제 저장된 좌표로 이동하는 게 아니라, 드라이버 자체의 원점복귀를 그대로 실행
int cli_go_home(void) {
	int p = 0;
	pause = 0;
	print("go home\r\n");

	if (!button_home_x(&p))
		return 0;
	data.home_x = 0;
	data.last_x = 0;
	data.home_ok |= 0x01;
	save_all();

	if (!button_home_y(&p))
		return 0;
	data.home_y = 0;
	data.last_y = 0;
	data.home_ok |= 0x02;
	save_all();

	save_last();
	return 1;
}

int cli_go(int col, int dan, int xrpm, int yrpm) {
	int xt = data.x[col - 1];
	int yt = data.y[dan - 1];

	if (!cli_homed()) {
		print("Need home\r\n");
		return 0;
	}
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
		HAL_Delay(200);

		fram_read(0, &data, sizeof(data));
		if (data.ver != DATA_VER) {
			memset(&data, 0, sizeof(data));
			data.ver = DATA_VER;
			save_all();
		}

		motor_sync(&motorX, data.last_x);
		motor_sync(&motorY, data.last_y);

		loaded = 1;
		print("\r\n");
		show();
		print_menu();
	}

	read_line(line, sizeof(line));
	print("\r\n");

	if ((sscanf(line, "x%d save", &a) == 1 || sscanf(line, "x%dsave", &a) == 1)
			&& a >= 1 && a <= 4) {
		button_save_x(a);
		show();
		print_menu();
		return;
	}

	if ((sscanf(line, "y%d save", &a) == 1 || sscanf(line, "y%dsave", &a) == 1)
			&& a >= 1 && a <= 4) {
		button_save_y(a);
		show();
		print_menu();
		return;
	}

	if (sscanf(line, "go %d %d", &a, &b) == 2) {
		cli_go(a, b, 500, 500);
		print_menu();
		return;
	}

	if (strcmp(line, "show") == 0) {
		show();
		print_menu();
		return;
	}

	if (strcmp(line, "reset") == 0) {
		memset(&data, 0, sizeof(data));
		data.ver = DATA_VER;
		save_all();
		show();
		print_menu();
		return;
	}

	if (strcmp(line, "stop") == 0) {
		pause = 1;
		motor_stop(&motorX);
		motor_stop(&motorY);

		HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_RESET);
		print_menu();
		return;
	}

	if (strcmp(line, "xhome") == 0) {
		int p = 0;
		if (button_home_x(&p)) {
			data.home_x = 0;
			data.last_x = 0;
			data.home_ok |= 0x01;
			save_last();
		}
		show();
		print_menu();
		return;
	}

	if (strcmp(line, "yhome") == 0) {
		int p = 0;
		if (button_home_y(&p)) {
			data.home_y = 0;
			data.last_y = 0;
			data.home_ok |= 0x02;
			save_last();
		}
		show();
		print_menu();
		return;
	}

	if (strcmp(line, "home") == 0) {
		cli_go_home();
		show();
		print_menu();
		return;
	}

	print("Wrong input\r\n");
	print_menu();
}
