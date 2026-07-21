/*
 * ==========================================================
 *  HELIOS ☀  —  휴머노이드 로봇 메인 스케치
 * ==========================================================
 *  PCA9685 16채널 서보 드라이버 기준.
 *  구성: 머리 + 허리 + 양팔(각3) + 양다리(각4) = 16 서보
 *
 *  필요 라이브러리 (라이브러리 매니저에서 설치):
 *    - Adafruit PWM Servo Driver Library
 *
 *  하드웨어 설정은 config.h 에서 변경하세요.
 *
 *  시리얼 명령 (9600 baud):
 *    n : 중립(차렷) 자세
 *    w : 앞으로 걷기
 *    a : 손 흔들기 (인사)
 *    s : 정지
 * ==========================================================
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// 중립각/방향 테이블
const int neutral[NUM_SERVOS]   = NEUTRAL_ANGLES;
const int servoDir[NUM_SERVOS]  = SERVO_DIRECTIONS;

bool walking = false;

// ---------- 저수준: 각도 → 서보 출력 ----------
int clampAngle(int a) {
  if (a < SERVO_MIN_ANGLE) return SERVO_MIN_ANGLE;
  if (a > SERVO_MAX_ANGLE) return SERVO_MAX_ANGLE;
  return a;
}

// 절대 각도(0~180)를 채널에 직접 씀
void writeAngle(uint8_t ch, int angle) {
  angle = clampAngle(angle);
  int pulse = map(angle, 0, 180, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
  pwm.setPWM(ch, 0, pulse);
}

// 중립각 기준 오프셋을 방향 적용해서 씀 (걸음/동작에 사용)
void moveJoint(uint8_t ch, int offset) {
  writeAngle(ch, neutral[ch] + servoDir[ch] * offset);
}

// ---------- 자세 ----------
void standNeutral() {
  for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
    writeAngle(ch, neutral[ch]);
  }
}

// ---------- 걸음걸이 (단순 오픈루프 보행) ----------
// 한 걸음: 무게중심 이동 → 스윙다리 들기 → 앞으로 → 내리기 → 복귀
// swingRight=true 이면 오른다리를 내딛습니다.
void takeStep(bool swingRight) {
  // 지지 다리 / 스윙 다리 채널 선택
  uint8_t sw_hip   = swingRight ? CH_R_HIP   : CH_L_HIP;
  uint8_t sw_thigh = swingRight ? CH_R_THIGH : CH_L_THIGH;
  uint8_t sw_calf  = swingRight ? CH_R_CALF  : CH_L_CALF;
  uint8_t sw_ankle = swingRight ? CH_R_ANKLE : CH_L_ANKLE;

  uint8_t st_hip   = swingRight ? CH_L_HIP   : CH_R_HIP;
  uint8_t st_ankle = swingRight ? CH_L_ANKLE : CH_R_ANKLE;

  // 1) 무게중심을 지지발 쪽으로 (고관절/발목으로 기울임)
  moveJoint(sw_hip,   +GAIT_SHIFT_AMP);
  moveJoint(st_hip,   +GAIT_SHIFT_AMP);
  moveJoint(sw_ankle, -GAIT_SHIFT_AMP);
  moveJoint(st_ankle, -GAIT_SHIFT_AMP);
  delay(GAIT_STEP_DELAY_MS * 5);

  // 2) 스윙 다리 들기 (종아리 굽힘)
  moveJoint(sw_calf, +GAIT_LIFT_AMP);
  delay(GAIT_STEP_DELAY_MS * 3);

  // 3) 스윙 다리 앞으로 (허벅지)
  moveJoint(sw_thigh, +GAIT_SWING_AMP);
  delay(GAIT_STEP_DELAY_MS * 4);

  // 4) 스윙 다리 내리기
  moveJoint(sw_calf, 0);
  delay(GAIT_STEP_DELAY_MS * 3);

  // 5) 무게중심 복귀 + 허벅지 복귀
  moveJoint(sw_hip, 0);
  moveJoint(st_hip, 0);
  moveJoint(sw_ankle, 0);
  moveJoint(st_ankle, 0);
  moveJoint(sw_thigh, 0);
  delay(GAIT_STEP_DELAY_MS * 3);
}

void walkForward(int steps) {
  for (int i = 0; i < steps && walking; i++) {
    takeStep(true);   // 오른다리
    if (!walking) break;
    takeStep(false);  // 왼다리
  }
  standNeutral();
}

// ---------- 손 흔들기 (인사) ----------
void waveHand() {
  moveJoint(CH_R_SHO_ROLL, 60);   // 오른팔 옆으로 올림
  delay(300);
  for (int i = 0; i < 3; i++) {
    moveJoint(CH_R_ELBOW, 30);
    delay(250);
    moveJoint(CH_R_ELBOW, -10);
    delay(250);
  }
  moveJoint(CH_R_ELBOW, 0);
  moveJoint(CH_R_SHO_ROLL, 0);
}

// ---------- 셋업 / 루프 ----------
void setup() {
  Serial.begin(9600);

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(100);

  standNeutral();

  Serial.println(F("HELIOS ready. Commands: n=neutral, w=walk, a=wave, s=stop"));
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
      case 'a':
        Serial.println(F("-> wave"));
        waveHand();
        break;
      case 's':
        walking = false;
        Serial.println(F("-> stop"));
        break;
    }
  }
}
