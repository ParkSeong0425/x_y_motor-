#ifndef CLI_H
#define CLI_H

// PC 입력 한 줄을 받아 명령 실행
// 형식: x <속도%> <위치>   예) x 10 5000
void cli_run(void);

// ★ 경고(Warning)를 해결하기 위해 추가해야 하는 함수 선언
int cli_go(int col, int dan, int rpm);

#endif
