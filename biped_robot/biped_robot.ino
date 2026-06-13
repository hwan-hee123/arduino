// biped_robot.ino
// 2족보행 휴머노이드 로봇 제어
// 서보 6개: 왼쪽(허리/무릎/발목) + 오른쪽(허리/무릎/발목)
// 시리얼 명령: F=전진, B=후진, L=좌회전, R=우회전, S=정지

#include <Servo.h>
#include "gait_patterns.h"

// ── 핀 번호 ──────────────────────────────────────────────────────────────────
#define PIN_LH  2    // Left  Hip   (왼쪽 허리)
#define PIN_LK  3    // Left  Knee  (왼쪽 무릎)
#define PIN_LA  4    // Left  Ankle (왼쪽 발목)
#define PIN_RH  5    // Right Hip   (오른쪽 허리)
#define PIN_RK  6    // Right Knee  (오른쪽 무릎)
#define PIN_RA  7    // Right Ankle (오른쪽 발목)

// ── 설정 ────────────────────────────────────────────────────────────────────
#define SERIAL_BAUD   9600
#define STEP_DELAY_MS 180   // 프레임 간 대기 시간 (ms) - 값을 줄이면 빠르게 걷습니다

// ── 서보 객체 ───────────────────────────────────────────────────────────────
Servo servos[6];
const int PINS[6] = { PIN_LH, PIN_LK, PIN_LA, PIN_RH, PIN_RK, PIN_RA };

char command  = 'S';   // 현재 명령 (S=정지)
int  gaitFrame = 0;    // 현재 보행 프레임 인덱스

// ── 초기화 ──────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(SERIAL_BAUD);

  for (int i = 0; i < 6; i++)
    servos[i].attach(PINS[i]);

  setServos(NEUTRAL);        // 기본 자세 (90도, 차렷 자세)
  delay(1000);

  Serial.println("=== Biped Robot Ready ===");
  Serial.println("Commands: F=Forward B=Backward L=Left R=Right S=Stop");
}

// ── 메인 루프 ───────────────────────────────────────────────────────────────
void loop() {
  // 시리얼로 명령 수신
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'S') {
      if (c != command) {
        command   = c;
        gaitFrame = 0;   // 새 명령이면 첫 프레임부터 시작
        Serial.print("Command: ");
        Serial.println(command);
      }
    }
  }

  switch (command) {
    case 'F': runGait(GAIT_FORWARD,  GAIT_FWD_FRAMES); break;
    case 'B': runGait(GAIT_BACKWARD, GAIT_BWD_FRAMES); break;
    case 'L': runGait(GAIT_LEFT,     GAIT_LFT_FRAMES); break;
    case 'R': runGait(GAIT_RIGHT,    GAIT_RGT_FRAMES); break;
    default:  setServos(NEUTRAL);                       break;
  }
}

// ── 보행 패턴 1프레임 실행 ──────────────────────────────────────────────────
void runGait(const int gait[][6], int frames) {
  setServos(gait[gaitFrame]);
  delay(STEP_DELAY_MS);
  gaitFrame = (gaitFrame + 1) % frames;
}

// ── 서보 6개 동시 제어 ──────────────────────────────────────────────────────
// angles[0]=LH  angles[1]=LK  angles[2]=LA
// angles[3]=RH  angles[4]=RK  angles[5]=RA
void setServos(const int angles[6]) {
  for (int i = 0; i < 6; i++)
    servos[i].write(constrain(angles[i], 0, 180));
}
