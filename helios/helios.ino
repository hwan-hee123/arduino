/*
 * ==========================================================
 *  HELIOS ☀  —  휴머노이드 로봇 (단일 파일 버전)
 * ==========================================================
 *  PCA9685 16채널 서보 드라이버 기준.
 *  구성: 머리 + 허리 + 양팔(각3) + 양다리(각4) = 16 서보
 *
 *  ▶ 이 파일 하나만 아두이노 IDE에 복붙하면 됩니다. (config.h 필요 없음)
 *
 *  필요 라이브러리 (라이브러리 매니저에서 설치):
 *    - Adafruit PWM Servo Driver Library
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

// ==========================================================
//  ▼▼▼ 설정 (하드웨어에 맞게 이 부분만 수정) ▼▼▼
// ==========================================================

// ----- PCA9685 -----
#define PCA9685_ADDR   0x40
#define SERVO_FREQ     50      // 아날로그 서보 = 50Hz
#define SERVO_PULSE_MIN  150   // 약 0도 (서보에 맞게 미세조정)
#define SERVO_PULSE_MAX  600   // 약 180도

// ----- 채널 번호 (배선표 그대로) -----
#define CH_HEAD        0   // 머리
#define CH_WAIST       1   // 허리
#define CH_R_SHO_PITCH 2   // 오른쪽 어깨 (위아래)
#define CH_R_SHO_ROLL  3   // 오른쪽 어깨 (옆으로 벌림)
#define CH_R_ELBOW     4   // 오른쪽 팔꿈치
#define CH_L_SHO_PITCH 5   // 왼쪽 어깨 (위아래)
#define CH_L_SHO_ROLL  6   // 왼쪽 어깨 (옆으로 벌림)
#define CH_L_ELBOW     7   // 왼쪽 팔꿈치
#define CH_R_HIP       8   // 오른쪽 고관절
#define CH_R_THIGH     9   // 오른쪽 허벅지
#define CH_R_CALF      10  // 오른쪽 종아리 (무릎)
#define CH_R_ANKLE     11  // 오른쪽 발목
#define CH_L_HIP       12  // 왼쪽 고관절
#define CH_L_THIGH     13  // 왼쪽 허벅지
#define CH_L_CALF      14  // 왼쪽 종아리 (무릎)
#define CH_L_ANKLE     15  // 왼쪽 발목
#define NUM_SERVOS     16

// ----- 각도 안전 한계 -----
#define SERVO_MIN_ANGLE  0
#define SERVO_MAX_ANGLE  180

// ----- 중립(차렷) 자세 각도 [채널 0~15] -----
// 목/허리/어깨상하/양발목 = 90도, 나머지 = 0도로 조립함
const int neutral[NUM_SERVOS] = {
  90, // 0  머리(목)
  90, // 1  허리
  90, // 2  R 어깨 상하
  0,  // 3  R 어깨 벌림
  0,  // 4  R 팔꿈치
  90, // 5  L 어깨 상하
  0,  // 6  L 어깨 벌림
  0,  // 7  L 팔꿈치
  0,  // 8  R 고관절
  0,  // 9  R 허벅지
  0,  // 10 R 종아리
  90, // 11 R 발목
  0,  // 12 L 고관절
  0,  // 13 L 허벅지
  0,  // 14 L 종아리
  90  // 15 L 발목
};

// ----- 서보 방향 (반대로 움직이면 부호 뒤집기) -----
const int servoDir[NUM_SERVOS] = {
  +1, // 0  머리
  +1, // 1  허리
  +1, // 2  R 어깨 상하
  +1, // 3  R 어깨 벌림
  +1, // 4  R 팔꿈치
  -1, // 5  L 어깨 상하
  -1, // 6  L 어깨 벌림
  -1, // 7  L 팔꿈치
  +1, // 8  R 고관절
  +1, // 9  R 허벅지
  +1, // 10 R 종아리
  +1, // 11 R 발목
  -1, // 12 L 고관절
  -1, // 13 L 허벅지
  -1, // 14 L 종아리
  -1  // 15 L 발목
};

// ----- 걸음걸이 파라미터 -----
#define GAIT_STEP_DELAY_MS  20   // 프레임 간 지연(작을수록 빠름)
#define GAIT_SWING_AMP      20   // 다리 앞뒤 스윙(허벅지)
#define GAIT_LIFT_AMP       20   // 발 들기(종아리)
#define GAIT_SHIFT_AMP      12   // 무게중심 좌우 이동(고관절/발목)

// ==========================================================
//  ▲▲▲ 설정 끝 ▲▲▲
// ==========================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);
bool walking = false;

// ---------- 저수준: 각도 → 서보 출력 ----------
int clampAngle(int a) {
  if (a < SERVO_MIN_ANGLE) return SERVO_MIN_ANGLE;
  if (a > SERVO_MAX_ANGLE) return SERVO_MAX_ANGLE;
  return a;
}

void writeAngle(uint8_t ch, int angle) {
  angle = clampAngle(angle);
  int pulse = map(angle, 0, 180, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
  pwm.setPWM(ch, 0, pulse);
}

void moveJoint(uint8_t ch, int offset) {
  writeAngle(ch, neutral[ch] + servoDir[ch] * offset);
}

// ---------- 자세 ----------
void standNeutral() {
  for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
    writeAngle(ch, neutral[ch]);
  }
}

// ---------- 걸음걸이 ----------
void takeStep(bool swingRight) {
  uint8_t sw_hip   = swingRight ? CH_R_HIP   : CH_L_HIP;
  uint8_t sw_thigh = swingRight ? CH_R_THIGH : CH_L_THIGH;
  uint8_t sw_calf  = swingRight ? CH_R_CALF  : CH_L_CALF;
  uint8_t sw_ankle = swingRight ? CH_R_ANKLE : CH_L_ANKLE;
  uint8_t st_hip   = swingRight ? CH_L_HIP   : CH_R_HIP;
  uint8_t st_ankle = swingRight ? CH_L_ANKLE : CH_R_ANKLE;

  // 1) 무게중심을 지지발 쪽으로
  moveJoint(sw_hip,   +GAIT_SHIFT_AMP);
  moveJoint(st_hip,   +GAIT_SHIFT_AMP);
  moveJoint(sw_ankle, -GAIT_SHIFT_AMP);
  moveJoint(st_ankle, -GAIT_SHIFT_AMP);
  delay(GAIT_STEP_DELAY_MS * 5);

  // 2) 스윙 다리 들기
  moveJoint(sw_calf, +GAIT_LIFT_AMP);
  delay(GAIT_STEP_DELAY_MS * 3);

  // 3) 스윙 다리 앞으로
  moveJoint(sw_thigh, +GAIT_SWING_AMP);
  delay(GAIT_STEP_DELAY_MS * 4);

  // 4) 스윙 다리 내리기
  moveJoint(sw_calf, 0);
  delay(GAIT_STEP_DELAY_MS * 3);

  // 5) 복귀
  moveJoint(sw_hip, 0);
  moveJoint(st_hip, 0);
  moveJoint(sw_ankle, 0);
  moveJoint(st_ankle, 0);
  moveJoint(sw_thigh, 0);
  delay(GAIT_STEP_DELAY_MS * 3);
}

void walkForward(int steps) {
  for (int i = 0; i < steps && walking; i++) {
    takeStep(true);
    if (!walking) break;
    takeStep(false);
  }
  standNeutral();
}

// ---------- 손 흔들기 ----------
void waveHand() {
  moveJoint(CH_R_SHO_ROLL, 60);
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
