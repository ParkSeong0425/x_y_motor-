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
			// 1번째 누름: 홈으로만 이동, 램프/버튼은 안 건드림
			// (home 도착 후 button_run에서 원래 상태로 복원)
			booted = 1;
			homing = 1;
		} else {
			// 2번째 누름(또는 저장된 home이 없어 바로 운전 시작하는 경우)
			booted = 1;
			run = 1;
			HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin,
					GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_RESET);
		}
	}

	if (pin == STOP_btn_Pin) {
		if (now - last[1] < DEBOUNCE)
			return;
		last[1] = now;

		pause = 1;
		run = 0;
		// 실제 정지 명령(RS485 통신)은 여기서 보내지 않는다.
		// ISR 안에서 블로킹 통신을 하면 메인 루프와 같은 UART를 동시에 건드릴 위험이 있다.
		// motor_wait()/motor_home()이 다음 폴링에서 pause를 보고 motor_stop()을 호출한다.

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
			// 여기도 마찬가지로 ISR 안에서는 통신 안 함, 플래그만 세움
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

// jog 레지스터 켜서 이동, s 누르면 끔
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
	motor_stop(m);
	HAL_Delay(100);
	return ok;
}

int button_home_x(int *out) {
	print("X driver home\r\n");
	if (!motor_home(&motorX, R_DI3L))   // 역방향 조그로 원점센서까지
		return 0;
	*out = 0;
	print("X home set\r\n");
	return 1;
}

int button_home_y(int *out) {
	print("Y driver home\r\n");
	// DI1은 원점센서 전용이라 정방향 조그는 DI9 사용
	if (!motor_home(&motorY, R_DI9L))
		return 0;
	*out = 0;
	print("Y home set\r\n");
	return 1;
}

int button_save_x(int n) {
	print("X save... press s\r\n");
	// DI1은 이제 원점센서 전용이라 정방향 조그는 DI9 사용
	if (!jog_until_s(&motorX, R_DI9L))
		return 0;

	int now = 0;
	if (!motor_pos(&motorX, &now))
		return 0;

	cli_set_x(n, now);
	print("save\r\n");
	return 1;
}

int button_save_y(int n) {
	print("Y save... press s\r\n");
	if (!jog_until_s(&motorY, R_DI3L))
		return 0;

	int now = 0;
	if (!motor_pos(&motorY, &now))
		return 0;

	cli_set_y(n, now);
	print("save\r\n");
	return 1;
}

// 자동 왕복: go 1 1 -> 1 2 -> ... -> 4 4 -> 다시 1 1 반복
void button_run(void) {
	cli_check_power();   // 모터 드라이버가 (STM32와 별개로) 다시 살아났는지 확인
	set_di5(estop);

	if (homing) {
		homing = 0;
		if (cli_go_home()) {
			// 홈 도착 성공 -> 전원 켰을 때와 같은 램프 상태로 복원
			HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin,
					GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin,
					GPIO_PIN_SET);
			HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET);
		}
		// 실패(STOP/estop/timeout)면 그때의 램프 상태를 그대로 둠
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
