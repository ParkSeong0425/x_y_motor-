#include "button.h"
#include "motor.h"
#include "cli.h"

#define X_RPM 500
#define Y_RPM 500
#define WAIT     500
#define DEBOUNCE 200

volatile int run = 0;
volatile int pause = 0;
volatile int estop = 0;

static int step = 0;
static volatile int fresh = 0;
static int begin = 0;
static int di5 = 0;

static int booted = 0;     // 전원 켜고 아직 홈 이동 안함
static volatile int homing = 0;   // 홈으로 이동 중

extern TIM_HandleTypeDef htim1;

void button_init(void) {
	HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_SET);
	HAL_TIM_Base_Start_IT(&htim1);
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	static uint32_t last[3];
	uint32_t now = HAL_GetTick();

	if (pin == MOTOR_START_btn_Pin) {
		if (now - last[0] < DEBOUNCE)
			return;
		last[0] = now;
		if (estop)
			return;
		pause = 0;
		if (!booted && cli_homed()) {
			booted = 1;
			homing = 1;
		} else {
			booted = 1;
			run = 1;
		}
		HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin,
				GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_RESET);
	}
	if (pin == STOP_btn_Pin) {
		if (now - last[1] < DEBOUNCE)
			return;
		last[1] = now;
		pause = 1;
		run = 0;
		HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_RESET);
	}

	if (pin == ESTOP_btn_Pin) {
		if (now - last[2] < DEBOUNCE)
			return;
		last[2] = now;
		estop = HAL_GPIO_ReadPin(ESTOP_btn_GPIO_Port, ESTOP_btn_Pin);
		if (estop) {
			run = 0;
			HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin,
					GPIO_PIN_RESET);
		} else {
			fresh = 1;
			HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET);
		}
	}
}

static void set_begin(int v) {
	if (v == begin)
		return;
	begin = v;
	motor_begin(&motorX, v);
	motor_begin(&motorY, v);
}

static void set_di5(int v) {
	if (v == di5)
		return;
	di5 = v;
	motor_di5(&motorX, v);
	motor_di5(&motorY, v);
}

// jog 레지스터 켜서 이동, s 누르면 끔 (R_DI1L: 정방향, R_DI3L: 역방향)
// s로 끝나면 1, estop이나 STOP(pause)으로 끊기면 0
static int jog_until_s(Motor *m, uint16_t jog) {
	pause = 0;
	motor_write(m, JOG_SPEED, JOG_RPM);
	motor_write(m, jog, 1);
	int ok = 0;
	while (!estop && !pause) {
		uint8_t c;
		if (cli_getc(&c) && (c == 's' || c == 'S')) {
			ok = 1;
			break;
		}
	}
	motor_write(m, jog, 0);
	HAL_Delay(100);
	return ok;
}

int button_home_x(int *out) {
	print("X home... press s\r\n");
	if (!jog_until_s(&motorX, R_DI3L))   // 역방향
		return 0;
	print("X home set\r\n");
	return motor_pos(&motorX, out);
}

int button_home_y(int *out) {
	print("Y home... press s\r\n");
	if (!jog_until_s(&motorY, R_DI1L))   // 아래로
		return 0;
	print("Y home set\r\n");
	return motor_pos(&motorY, out);
}

int button_save_x(int n) {
	print("X save... press s\r\n");
	if (!jog_until_s(&motorX, R_DI1L))   // 정방향
		return 0;
	int now = 0;
	motor_pos(&motorX, &now);
	cli_set_x(n, now);
	print("save\r\n");
	return 1;
}

int button_save_y(int n) {
	print("Y save... press s\r\n");
	if (!jog_until_s(&motorY, R_DI3L))   // 위로
		return 0;
	int now = 0;
	motor_pos(&motorY, &now);
	cli_set_y(n, now);   // raw pulse 그대로 (부호 반전 없음)
	print("save\r\n");
	return 1;
}

// 자동 왕복: go 1 1 -> 1 2 -> ... -> 4 4 -> 다시 1 1 반복
void button_run(void) {
	set_di5(estop);

	if (homing) {
		homing = 0;
		cli_go_home();
		return;
	}

	if (!run || pause || estop)
		return;

	if (fresh) {
		fresh = 0;
		step = 0;
		set_begin(1);
	} else {
		set_begin(0);
	}

	int col = step / 4 + 1;
	int dan = step % 4 + 1;

	if (!cli_go(col, dan, X_RPM, Y_RPM))
		return;

	HAL_Delay(WAIT);
	step = (step + 1) % 16;
}
