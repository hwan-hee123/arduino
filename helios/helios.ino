/*
 * ==========================================================
 *  HELIOS ☀  —  이족 보행 로봇 메인 스케치
 * ==========================================================
 *  다리당 3서보(엉덩이/무릎/발목), 총 6서보 기준.
 *  하드웨어 설정은 config.h 에서 변경하세요.
 *
 *  시리얼 명령 (9600 baud):
 *    n : 중립 자세로 서기
 *    w : 앞으로 걷기 (몇 걸음)
 *    s : 정지
 * ==========================================================
 */

#include <Servo.h>
#include "config.h"

// --- 서보 객체 ---
Servo sHip[2];    // [0]=왼쪽, [1]=오른쪽
Servo sKnee[2];
Servo sAnkle[2];

// 인덱스 편의를 위한 상수
#define LEFT   0
#define RIGHT  1

// 중립 각도 테이블
const int neutralHip[2]   = { NEUTRAL_L_HIP,   NEUTRAL_R_HIP   };
const int neutralKnee[2]  = { NEUTRAL_L_KNEE,  NEUTRAL_R_KNEE  };
const int neutralAnkle[2] = { NEUTRAL_L_ANKLE, NEUTRAL_R_ANKLE };

// 방향 테이블
const int dirHip[2]   = { DIR_L_HIP,   DIR_R_HIP   };
const int dirKnee[2]  = { DIR_L_KNEE,  DIR_R_KNEE  };
const int dirAnkle[2] = { DIR_L_ANKLE, DIR_R_ANKLE };

bool walking = false;

// ---------- 유틸 ----------
int clampAngle(int a) {
  if (a < SERVO_MIN_ANGLE) return SERVO_MIN_ANGLE;
  if (a > SERVO_MAX_ANGLE) return SERVO_MAX_ANGLE;
  return a;
}

// 관절에 (중립 기준) 오프셋을 적용해서 씀
void setHip(int leg, int offset)   { sHip[leg].write(clampAngle(neutralHip[leg]   + dirHip[leg]   * offset)); }
void setKnee(int leg, int offset)  { sKnee[leg].write(clampAngle(neutralKnee[leg]  + dirKnee[leg]  * offset)); }
void setAnkle(int leg, int offset) { sAnkle[leg].write(clampAngle(neutralAnkle[leg]+ dirAnkle[leg] * offset)); }

// ---------- 자세 ----------
void standNeutral() {
  for (int leg = 0; leg < 2; leg++) {
    setHip(leg, 0);
    setKnee(leg, 0);
    setAnkle(leg, 0);
  }
}

// ---------- 걸음걸이 (아주 단순한 오픈루프 보행) ----------
// 한쪽 다리를 들어 앞으로, 반대쪽은 뒤로 밀며 무게중심 이동을 흉내냅니다.
// 실제 보행 안정화는 IMU(MPU6050) 추가 후 튜닝이 필요합니다.
void takeStep(int swingLeg) {
  int standLeg = (swingLeg == LEFT) ? RIGHT : LEFT;

  // 1) 무게중심을 지지발 쪽으로 (발목으로 살짝 기울임)
  setAnkle(swingLeg, +GAIT_LIFT_AMP / 2);
  setAnkle(standLeg, +GAIT_LIFT_AMP / 2);
  delay(GAIT_STEP_DELAY_MS * 4);

  // 2) 스윙 다리 들기 (무릎 굽힘)
  setKnee(swingLeg, GAIT_LIFT_AMP);
  delay(GAIT_STEP_DELAY_MS * 3);

  // 3) 스윙 다리 앞으로 / 지지 다리 뒤로
  setHip(swingLeg, +GAIT_SWING_AMP);
  setHip(standLeg, -GAIT_SWING_AMP);
  delay(GAIT_STEP_DELAY_MS * 4);

  // 4) 스윙 다리 내리기
  setKnee(swingLeg, 0);
  delay(GAIT_STEP_DELAY_MS * 3);

  // 5) 발목 원위치
  setAnkle(swingLeg, 0);
  setAnkle(standLeg, 0);
  delay(GAIT_STEP_DELAY_MS * 2);
}

void walkForward(int steps) {
  for (int i = 0; i < steps && walking; i++) {
    takeStep(LEFT);
    if (!walking) break;
    takeStep(RIGHT);
  }
  standNeutral();
}

// ---------- 셋업 / 루프 ----------
void setup() {
  Serial.begin(9600);

  sHip[LEFT].attach(PIN_L_HIP);
  sKnee[LEFT].attach(PIN_L_KNEE);
  sAnkle[LEFT].attach(PIN_L_ANKLE);
  sHip[RIGHT].attach(PIN_R_HIP);
  sKnee[RIGHT].attach(PIN_R_KNEE);
  sAnkle[RIGHT].attach(PIN_R_ANKLE);

  standNeutral();

  Serial.println(F("HELIOS ready. Commands: n=neutral, w=walk, s=stop"));
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'n':
        walking = false;
        standNeutral();
        Serial.println(F("-> neutral"));
        break;
      case 'w':
        walking = true;
        Serial.println(F("-> walking 4 steps"));
        walkForward(4);
        walking = false;
        break;
      case 's':
        walking = false;
        Serial.println(F("-> stop"));
        break;
    }
  }
}
