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
#define SERVO_PULSE_MAX  560   // 약 180도 (MG90S에 맞게 조정)

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
// 조립한 홈 자세와 동일. 오른쪽 0도 ↔ 왼쪽 180도 (좌우 거울 장착)
const int neutral[NUM_SERVOS] = {
  90,  // 0  머리(목)
  90,  // 1  허리
  90,  // 2  R 어깨 상하
  0,   // 3  R 어깨 벌림
  0,   // 4  R 팔꿈치
  90,  // 5  L 어깨 상하
  180, // 6  L 어깨 벌림
  180, // 7  L 팔꿈치
  0,   // 8  R 고관절
  0,   // 9  R 허벅지
  180, // 10 R 종아리 (재조립: 직립=180)
  90,  // 11 R 발목
  90,  // 12 L 고관절 (재조립: 홈=90)
  180, // 13 L 허벅지
  0,   // 14 L 종아리 (재조립: 직립=0)
  90   // 15 L 발목
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

// ----- 채널별 안전 가동 범위 (측정값). 벗어나는 명령은 자동으로 막힘 -----
const int limitMin[NUM_SERVOS] = {
  0, 0, 0, 0, 0, 0, 0, 70,
  0, 0, 100, 40, 0, 100, 0, 0
};
const int limitMax[NUM_SERVOS] = {
  179, 180, 180, 180, 130, 180, 180, 180,
  90, 80, 180, 180, 90, 180, 80, 140
};

// ----- 걸음걸이 파라미터 (측정 범위 안에서. 테스트하며 조정) -----
#define GAIT_STEP_DELAY_MS  20   // 프레임 간 지연(작을수록 빠름)
#define KNEE_LIFT    45   // 무릎 굽혀 발 들기 (종아리, 범위 내 최대 80)
#define THIGH_STEP   30   // 허벅지 앞으로 내딛기 (범위 내 최대 80)
#define ANKLE_SHIFT  20   // 발목으로 무게 좌우 이동

// ==========================================================
//  ▲▲▲ 설정 끝 ▲▲▲
// ==========================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);
bool walking = false;

// ---------- 저수준: 각도 → 서보 출력 ----------
void writeAngle(uint8_t ch, int angle) {
  // 채널별 안전 범위로 제한
  if (angle < limitMin[ch]) angle = limitMin[ch];
  if (angle > limitMax[ch]) angle = limitMax[ch];
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

// ---------- 걸음걸이용 동작 함수 (관절 특성 반영) ----------
// 각 관절의 홈·굽힘 방향이 달라서, 각도를 직접 계산해서 씀.
//  (측정값: R종아리 직립180→굽힘100 / L종아리 직립0→굽힘80)
//           (R허벅지 홈0→앞80 / L허벅지 홈180→앞100)

// 무릎 굽힘: bend=0 펴짐(직립), bend 커질수록 굽힘
void kneeBend(bool isRight, int bend) {
  if (isRight) writeAngle(CH_R_CALF, 180 - bend);  // 오른종아리: 180에서 감소
  else         writeAngle(CH_L_CALF,   0 + bend);  // 왼종아리: 0에서 증가
}

// 허벅지 내딛기: fwd=0 중립, fwd 커질수록 앞으로
void thighStep(bool isRight, int fwd) {
  if (isRight) writeAngle(CH_R_THIGH,   0 + fwd);  // 오른허벅지: 0에서 증가
  else         writeAngle(CH_L_THIGH, 180 - fwd);  // 왼허벅지: 180에서 감소
}

// 발목 무게이동: amt>0 = 한쪽으로 기울여 무게 이동
//  (몸이 안 기울고 비틀리면 아래 두 줄 중 왼발목 부호를 (90 - amt)로 바꾸세요)
void ankleShift(int amt) {
  writeAngle(CH_R_ANKLE, 90 + amt);   // 오른발목
  writeAngle(CH_L_ANKLE, 90 + amt);   // 왼발목
}

// ---------- 한 걸음 ----------
// swingRight=true → 오른다리를 들어 앞으로 내딛음
void takeStep(bool swingRight) {
  int shiftDir = swingRight ? -1 : +1;  // 드는 다리 반대(지지발)로 무게 이동

  // 1) 무게중심을 지지발로 (발목으로 기울임)
  ankleShift(shiftDir * ANKLE_SHIFT);
  delay(GAIT_STEP_DELAY_MS * 6);

  // 2) 스윙 다리 들기 (무릎 굽힘)
  kneeBend(swingRight, KNEE_LIFT);
  delay(GAIT_STEP_DELAY_MS * 4);

  // 3) 스윙 다리 앞으로 (허벅지)
  thighStep(swingRight, THIGH_STEP);
  delay(GAIT_STEP_DELAY_MS * 4);

  // 4) 스윙 발 내리기 (무릎 펴기)
  kneeBend(swingRight, 0);
  delay(GAIT_STEP_DELAY_MS * 4);

  // 5) 무게중심 중앙 복귀
  ankleShift(0);
  delay(GAIT_STEP_DELAY_MS * 4);

  // 6) 허벅지 중립 복귀 (몸이 앞으로 나아감)
  thighStep(swingRight, 0);
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

// ---------- 지르기(펀치) 콤보 ----------
// 팔 동작은 moveJoint 오프셋으로 좌우 자동 대칭 처리 (servoDir 반영).
#define GUARD_ELBOW    100  // 가드 시 팔꿈치 굽힘
#define GUARD_SHOULDER  35  // 가드 시 어깨 살짝 들기
#define PUNCH_ELBOW     10  // 지를 때 팔꿈치 펴기
#define PUNCH_SHOULDER  70  // 지를 때 어깨 앞으로
#define WAIST_TWIST     40  // 지를 때 몸통 회전 각도

// 몸통 회전 (머리는 반대로 돌려 정면 고정). 방향 반대면 부호 바꾸기.
void torsoTwist(int dir) {
  int w = 90 + dir * WAIST_TWIST;
  writeAngle(CH_WAIST, w);
  writeAngle(CH_HEAD, 180 - w);   // 머리를 반대로 돌려 정면 유지
}

void armGuard(bool isRight) {
  moveJoint(isRight ? CH_R_SHO_PITCH : CH_L_SHO_PITCH, GUARD_SHOULDER);
  moveJoint(isRight ? CH_R_ELBOW     : CH_L_ELBOW,     GUARD_ELBOW);
  torsoTwist(0);
}
// 오른손 지르기 → 몸통 왼쪽 / 왼손 지르기 → 몸통 오른쪽
void armPunch(bool isRight) {
  moveJoint(isRight ? CH_R_SHO_PITCH : CH_L_SHO_PITCH, PUNCH_SHOULDER);
  moveJoint(isRight ? CH_R_ELBOW     : CH_L_ELBOW,     PUNCH_ELBOW);
  torsoTwist(isRight ? +1 : -1);
}

// 가드 → 오른손 지르기 → 가드 → 왼손 지르기 → 가드 → 차렷
void punchCombo() {
  armGuard(true); armGuard(false);   // 1) 양손 가드
  delay(400);
  armPunch(true);  delay(250);       // 2) 오른손 지르기
  armGuard(true);  delay(250);       // 3) 오른손 가드 복귀
  armPunch(false); delay(250);       // 4) 왼손 지르기
  armGuard(false); delay(250);       // 5) 왼손 가드 복귀
  delay(200);
  standNeutral();                    // 6) 차렷
}

// ---------- 무릎 구부리기 (스쿼트) ----------
#define SQUAT_KNEE   50   // 무릎 굽힘 정도
#define SQUAT_THIGH  30   // 허벅지 굽힘(균형용)
#define SQUAT_STEPS   4   // 부드럽게 나눌 단계 수

void squatPose(int knee, int thigh) {
  kneeBend(true, knee);  kneeBend(false, knee);
  thighStep(true, thigh); thighStep(false, thigh);
}
void squat() {
  for (int i = 1; i <= SQUAT_STEPS; i++) {       // 내려가기
    squatPose(SQUAT_KNEE * i / SQUAT_STEPS, SQUAT_THIGH * i / SQUAT_STEPS);
    delay(150);
  }
  delay(500);                                     // 앉은 자세 유지
  for (int i = SQUAT_STEPS - 1; i >= 0; i--) {   // 올라오기
    squatPose(SQUAT_KNEE * i / SQUAT_STEPS, SQUAT_THIGH * i / SQUAT_STEPS);
    delay(150);
  }
  standNeutral();
}

// ---------- 셋업 / 루프 ----------
void setup() {
  Serial.begin(9600);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(100);
  standNeutral();
  Serial.println(F("HELIOS ready. Commands: n=neutral, w=walk, a=wave, p=punch, k=squat, s=stop"));
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
      case 'p':
        Serial.println(F("-> punch"));
        punchCombo();
        break;
      case 'k':
        Serial.println(F("-> squat"));
        squat();
        break;
      case 's':
        walking = false;
        Serial.println(F("-> stop"));
        break;
    }
  }
}
