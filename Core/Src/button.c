#include "button.h"
#include "motor.h"
#include "cli.h"

#define RUN_RPM  750   // 자동 왕복 이동 속도
#define WAIT     500   // 도착 후 대기 시간 (ms)
#define DEBOUNCE 200   // 버튼 채터링 방지 (ms) - 한 번 누르면 한 번만 인식

volatile int run = 0;     // 자동 왕복 동작 중
volatile int pause = 0;   // 일시 정지 (PF9)
volatile int estop = 0;   // 비상 정지 (PF7, DI5 배선)

static int step = 0;      // 0~15: go 1 1 ~ go 4 4 진행 위치
static volatile int fresh = 0;   // estop 해제 후 첫 시작 = 처음(1,1)부터
static int begin = 0;            // 드라이브 H11_02에 마지막으로 쓴 값
static int di5 = 0;              // 드라이브 DI5 로직에 마지막으로 쓴 값

// 전원 켜질 때 1회 호출
void button_init(void) {
	HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, GPIO_PIN_SET); // PF1 = 1 (전원 LED)
	HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET);       // PF10 = 1
	HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_SET); // PF2 = 1
	HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_SET); // PA3 = 1
	HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin, GPIO_PIN_SET); // PC0 = 1
}

// 버튼 인터럽트: 플래그만 세우고 모터 통신은 button_run에서 처리
// (인터럽트 안에서 Modbus 통신을 하면 통신이 꼬임)
void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	static uint32_t last[3];
	uint32_t now = HAL_GetTick();

	if (pin == MOTOR_START_btn_Pin) {          // PG1: 시작 / 재개 (DI7 배선)
		if (now - last[0] < DEBOUNCE)
			return;
		last[0] = now;
		if (estop)                             // 비상 정지 중에는 무시
			return;
		pause = 0;
		run = 1;
		HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin,
				GPIO_PIN_RESET); // PF2 = 0
		HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET);   // PF10 = 1
		HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_RESET);
	}
	if (pin == STOP_btn_Pin) {                 // PF9: 일시정지
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

	if (pin == ESTOP_btn_Pin) {                // PF7: NC 비상정지, 토글 (DI5 배선)
		if (now - last[2] < DEBOUNCE)
			return;
		last[2] = now;
		estop = HAL_GPIO_ReadPin(ESTOP_btn_GPIO_Port, ESTOP_btn_Pin); // 눌림 = 0 = estop ON
		if (estop) {                     // 눌림: DI5L은 button_run의 set_di5가 알아서 씀
			run = 0;
			HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin,
					GPIO_PIN_RESET);
		} else {                               // 해제: 다음 PG1은 처음(1,1)부터
			fresh = 1;
			HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, GPIO_PIN_SET); // PF1 = 1 (전원 LED)
			HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET); // PF10 = 1

		}
	}
}

// H11_02 값이 바뀔 때만 드라이브에 씀 (매 이동마다 불필요한 통신 방지)
static void set_begin(int v) {
	if (v == begin)
		return;
	begin = v;
	motor_begin(&motorX, v);
	motor_begin(&motorY, v);
}

// PF7 상태에 맞춰 DI5 로직 값 동기화 (눌리면 1, 떼면 0)
static void set_di5(int v) {
	if (v == di5)
		return;
	di5 = v;
	motor_di5(&motorX, v);
	motor_di5(&motorY, v);
}

// 도는 중일 때 GREEN 램프 0.3초마다 깜빡임 (HAL_Delay 쓰면 시스템 멈추니 시간 체크로)
static void blink_green(void) {
	static uint32_t last = 0;
	if (!run)
		return;
	if (HAL_GetTick() - last < 300)
		return;
	last = HAL_GetTick();
	HAL_GPIO_TogglePin(LAMP_GREEN_GPIO_Port, LAMP_GREEN_Pin);
}

// 자동 왕복: go 1 1 -> 1 2 -> ... -> 4 4 -> 다시 1 1 반복
// cli의 입력 대기 중에 계속 호출됨
void button_run(void) {
	set_di5(estop);           // PF7 눌림 상태를 드라이브 DI5에 그대로 반영
	blink_green();             // 도는 중이면 1초마다 GREEN 램프 깜빡

	if (!run || pause || estop)
		return;

	if (fresh) {                  // estop 해제 후 시작: H11_02=1, 처음부터
		fresh = 0;
		step = 0;
		set_begin(1);
	} else {
		set_begin(0);             // 평소/일시정지 재개: H11_02=0, 멈춘 곳부터
	}

	int col = step / 4 + 1;
	int dan = step % 4 + 1;

	if (!cli_go(col, dan, RUN_RPM))   // 정지로 끊기면 step 유지 -> 재개 시 같은 목표
		return;

	HAL_Delay(WAIT);                  // 도착 후 500ms 대기
	step = (step + 1) % 16;
}
