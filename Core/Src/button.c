#include "button.h"
#include "motor.h"
#include "cli.h"

#define RUN_RPM  100
#define WAIT     500   // 도착 후 대기 시간 (ms)
#define DEBOUNCE 200   // 버튼 채터링 방지 (ms)

volatile int run = 0;
volatile int pause = 0;
volatile int estop = 0;
volatile int motor_begin_from_start = 0; // 모터 H11_02 레지스터 전송 제어용 플래그

static int step = 0;        // 0~15 자동 왕복 그리드 스텝 인덱스
static int begin_mode = 0;  // E-STOP 복구 지점 확인용 내부 플래그

// motor.c의 외부 통신 함수 참조 
extern int write16(Motor *m, uint16_t reg, uint16_t val);

void button_init(void) {
	HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, GPIO_PIN_SET); // PF1 = 1
	HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET);       // PF10 = 1
	HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin, GPIO_PIN_SET); // PF2 = 1
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	static uint32_t last[3];
	uint32_t now = HAL_GetTick();

	// 1. PG1: 기동 및 복귀 시작 명령 조작
	if (pin == MOTOR_START_btn_Pin) {
		if (now - last[0] < DEBOUNCE)
			return;
		last[0] = now;

		if (estop)                             // 비상정지 물리 해제 전에는 명령 유입 차단
			return;

		// 비상정지 해제 후 첫 구동 복귀 진입인 경우
		if (begin_mode == 1) {
			motor_begin_from_start = 1;        // 처음부터 구동 지시 모드 설정 (H11_02 = 1)
			step = 0;                          // 왕복 좌표 인덱스 go 1 1 위치로 전면 리셋
			begin_mode = 0;
		} else {
			motor_begin_from_start = 0; // 단순 일시정지 후 재개이므로 멈춘 곳부터 이어 구동 (H11_02 = 0)
		}

		pause = 0;
		run = 1;
		HAL_GPIO_WritePin(MOTOR_START_GPIO_Port, MOTOR_START_Pin,
				GPIO_PIN_RESET); // PF2 = 0 (DI7 = 1 환경 결합)
		HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET); // PF10 = 1 상태 유지
	}

	// 2. PF9: 일시 정지 조작
	if (pin == STOP_btn_Pin) {
		if (now - last[1] < DEBOUNCE)
			return;
		last[1] = now;

		pause = 1;
		HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_RESET); // PF10 = 0 (DI6 = 1 환경 결합)
	}

	// 3. PF7: NC형 비상정지 스위치 제어 루틴
	if (pin == ESTOP_btn_Pin) {
		if (now - last[2] < DEBOUNCE)
			return;
		last[2] = now;

		// NC 회로 판단: 물리 핀 상태를 직접 리드 (0 = 물리 스위치 눌림 / DI5 = 0 상태 전이)
		int pin_state = HAL_GPIO_ReadPin(ESTOP_btn_GPIO_Port, ESTOP_btn_Pin);

		if (pin_state == 0) {
			// [E-STOP 스위치 눌림 활성화]
			estop = 1;
			run = 0;

			// ★ 요구사항 반영: 0x0311 레지스터를 1로 인가하여 비상 정지 구동
			write16(&motorX, R_ESTOP, 1);
			write16(&motorY, R_ESTOP, 1);

			HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_RESET); // PF10 = 0
		} else {
			// [E-STOP 스위치 회전 해제 - 복구 상황 / DI1 = 1 상태 전이]
			estop = 0;
			begin_mode = 1; // 다음 PG1 트리거 시 처음부터(`go 1 1`) 가동하도록 예약

			// ★ 요구사항 반영: 0x0311 레지스터를 0으로 정상 클리어
			write16(&motorX, R_ESTOP, 0);
			write16(&motorY, R_ESTOP, 0);

			HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET); // PF10 = 1 복구
		}
	}
}

void button_run(void) {
	if (!run || pause || estop)
		return;

	int col = step / 4 + 1;
	int dan = step % 4 + 1;

	// 자동 이동 제어 전송
	if (!cli_go(col, dan, RUN_RPM))
		return;

	HAL_Delay(WAIT);                  // 도달 후 지연 대기
	step = (step + 1) % 16;           // 다음 시퀀스로 가동 대상 변환

	// 처음부터 복귀 구동이 1회 성공 완료되면 연속 구동을 위해 가동 플래그를 원래 상태(이어서 기동)로 복구
	motor_begin_from_start = 0;
}
