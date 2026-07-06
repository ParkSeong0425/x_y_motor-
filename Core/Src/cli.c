#include "cli.h"
#include "motor.h"
#include "fram.h"
#include "button.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

#define MOVE_TIMEOUT 10000
#define VER 4   // 저장 단위 변경: 회전수 -> pulse 위치

#define REV_MM  160     // 풀리 원주 (mm)
#define REV_CNT 131072  // 모터 1바퀴당 pulse 수
#define PULSE(v) ((v) * REV_CNT)            // 회전수 -> pulse
#define MM(v)    ((v) * REV_MM / REV_CNT)   // pulse -> mm 표시

#define X_MAX (60 * REV_CNT)     // X 최대: 60바퀴
#define Y_MAX (-(64 * REV_CNT))  // Y 최대: -64바퀴

// FRAM 주소 0번지에 저장되는 데이터
typedef struct {
	int ver;     // FRAM 버전 표식
	int x[4];    // X축 열 위치 pulse
	int y[4];    // Y축 단 위치 pulse, 양수로 저장하고 이동 때 음수로 사용
} Data;

static Data data;
static int loaded = 0;
static int homed_x = 0;   // xhome 하기 전엔 이동 금지
static int homed_y = 0;   // yhome 하기 전엔 이동 금지

static void read_line(char *buf, int max) {
	int i = 0;
	while (i < max - 1) {
		uint8_t c;
		if (HAL_UART_Receive(&huart3, &c, 1, 50) != HAL_OK) {
			button_run();    // 입력 기다리는 동안 버튼 자동 왕복 동작
			continue;
		}
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
	print("go x(열) y(단) rpm\r\n");
	print("show\r\n");
	print("reset\r\n");
	print("stop\r\n");
	print("xhome | yhome | home\r\n");
	print(": ");
}

// X, Y 모터에 이동 명령을 같이 보내고 같이 도착을 기다림
static int move_xy(int rpm, int xt, int yt) {
	motor_move(&motorX, rpm, xt);
	motor_move(&motorY, rpm, yt);
	return motor_wait(&motorX, xt, &motorY, yt, MOVE_TIMEOUT);
}

// X축 천천히 이동, s 누른 위치를 xn에 저장
static int save_x(int n) {
	int now = 0;
	int lim = data.x[0] > 0 ? data.x[0] : X_MAX;

	pause = 0;
	print("X save... press s\r\n");
	motor_move(&motorX, 100, lim);

	while (1) {
		if (estop) {
			motor_stop(&motorX);
			return 0;
		}

		uint8_t c;
		if (HAL_UART_Receive(&huart3, &c, 1, 100) == HAL_OK) {
			if (c == 's' || c == 'S')
				break;
		}
	}

	motor_stop(&motorX);
	HAL_Delay(100);

	if (!motor_pos(&motorX, &now))
		return 0;

	if (now < 0 || now > X_MAX) {
		print("Over range\r\n");
		return 0;
	}

	data.x[n - 1] = now;
	fram_write(0, &data, sizeof(data));
	print("save\r\n");
	show();
	return 1;
}

// Y축 천천히 이동, s 누른 위치를 yn에 저장
static int save_y(int n) {
	int now = 0;
	int lim = data.y[3] > 0 ? -data.y[3] : Y_MAX;

	pause = 0;
	print("Y save... press s\r\n");
	motor_move(&motorY, 100, lim);

	while (1) {
		if (estop) {
			motor_stop(&motorY);
			return 0;
		}

		uint8_t c;
		if (HAL_UART_Receive(&huart3, &c, 1, 100) == HAL_OK) {
			if (c == 's' || c == 'S')
				break;
		}
	}

	motor_stop(&motorY);
	HAL_Delay(100);

	if (!motor_pos(&motorY, &now))
		return 0;

	if (now > 0 || now < Y_MAX) {
		print("Over range\r\n");
		return 0;
	}

	data.y[n - 1] = -now;
	fram_write(0, &data, sizeof(data));
	print("save\r\n");
	show();
	return 1;
}

// 열/단 번호로 이동
int cli_go(int col, int dan, int rpm) {
	int xt = data.x[col - 1];
	int yd = data.y[dan - 1];
	int yt = -yd;   // Y는 역방향이라 음수로 이동

	if (xt == 0 || yd == 0) {
		print("Need save\r\n");
		return 0;
	}

	if (xt < 0 || xt > X_MAX || yt > 0 || yt < Y_MAX) {
		print("Over range\r\n");
		return 0;
	}

	if (data.x[0] > 0 && xt > data.x[0]) {
		print("Over range\r\n");
		return 0;
	}

	if (data.y[3] > 0 && yd > data.y[3]) {
		print("Over range\r\n");
		return 0;
	}

	return move_xy(rpm, xt, yt);
}

void cli_run(void) {
	char line[48];
	int a, b, c, d;

	if (!loaded) {                       // 첫 실행 때 FRAM 데이터 로드 + 버튼 핀 초기화
		button_init();
		fram_read(0, &data, sizeof(data));
		if (data.ver != VER) {            // 새 코드 다운로드 후 첫 부팅: FRAM 초기화
			memset(&data, 0, sizeof(data));
			data.ver = VER;
			fram_write(0, &data, sizeof(data));
		}
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
		if (!homed_x) {
			print("Need xhome first\r\n");
			return;
		}
		save_x(a);
		return;
	}

	if ((sscanf(line, "y%d save", &a) == 1 || sscanf(line, "y%dsave", &a) == 1)
			&& a >= 1 && a <= 4) {
		if (!homed_y) {
			print("Need yhome first\r\n");
			return;
		}
		save_y(a);
		return;
	}

	if (sscanf(line, "go %d %d %d", &a, &b, &c) == 3) {
		if (!homed_x || !homed_y) {
			print("Need xhome, yhome first\r\n");
			return;
		}
		if (a < 1 || a > 4 || b < 1 || b > 4) {
			print("Range 1-4\r\n");
			return;
		}
		cli_go(a, b, c);
		return;
	}

	if (sscanf(line, "x %d %d %d %d", &a, &b, &c, &d) == 4) {
		data.x[0] = PULSE(a);
		data.x[1] = PULSE(b);
		data.x[2] = PULSE(c);
		data.x[3] = PULSE(d);
		fram_write(0, &data, sizeof(data));
		print("save\r\n");
		return;
	}

	if (sscanf(line, "y %d %d %d %d", &a, &b, &c, &d) == 4) {
		data.y[0] = PULSE(a);
		data.y[1] = PULSE(b);
		data.y[2] = PULSE(c);
		data.y[3] = PULSE(d);
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
		data.ver = VER;
		fram_write(0, &data, sizeof(data));
		homed_x = 0;
		homed_y = 0;
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
		print("X homing...\r\n");
		homed_x = motor_home(&motorX, -X_MAX);
		return;
	}

	if (strcmp(line, "yhome") == 0) {
		print("Y homing...\r\n");
		homed_y = motor_home(&motorY, -Y_MAX);
		return;
	}

	if (strcmp(line, "home") == 0) {
		if (!homed_x || !homed_y) {
			print("Need xhome, yhome first\r\n");
			return;
		}
		print("Homing...\r\n");
		move_xy(100, 0, 0);
		return;
	}

	print("Wrong input\r\n");
}
