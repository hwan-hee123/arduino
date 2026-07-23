/*
 * ==========================================================
 *  HELIOS ☀  —  조립용 홈 위치 고정 코드
 * ==========================================================
 *  보드: Arduino Nano ESP32
 *  서보: PCA9685 16채널
 *
 *  ▶ 용도: 켜자마자 모든 서보를 홈 위치(0/90도)로 보내서 잡고 있음.
 *          이 상태에서 관절(혼)을 하나씩 끼워 조립하면
 *          모든 관절이 이 코드 기준에 정확히 맞게 됩니다.
 *
 *  ▶ 홈 위치:
 *      목·허리·어깨상하·양발목 = 90도
 *      나머지(어깨벌림·팔꿈치·고관절·허벅지·종아리) = 0도
 *
 *  ▶ 사용법:
 *      1) 업로드 → 16개 서보가 홈 위치로 이동해서 고정됨
 *      2) 전원 켠 채로 관절을 하나씩 그 위치에 맞춰 조립
 *      3) 조립 끝나면 helios_ble.ino(걷기/동작 코드)로 교체
 *
 *  ⚠ 서보 전원(V+ 5V) 반드시 연결. 조립 중 서보에 무리한 힘 주지 마세요.
 * ==========================================================
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PCA9685_ADDR   0x40
#define SERVO_FREQ     50
#define SERVO_PULSE_MIN  150   // 0도
#define SERVO_PULSE_MAX  600   // 180도
#define NUM_SERVOS     16

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// 홈(조립 기준) 위치 [채널 0~15]
// 좌우 서보가 거울처럼 장착돼서, 왼쪽은 (180 - 오른쪽각도)로 두어야 대칭이 됨.
const int home[NUM_SERVOS] = {
  90,  // 0  머리(목)
  90,  // 1  허리
  90,  // 2  R 어깨 상하
  0,   // 3  R 어깨 벌림
  0,   // 4  R 팔꿈치
  90,  // 5  L 어깨 상하  (180-90=90)
  180, // 6  L 어깨 벌림  (오른쪽 0의 거울)
  180, // 7  L 팔꿈치     (오른쪽 0의 거울)
  0,   // 8  R 고관절
  0,   // 9  R 허벅지
  0,   // 10 R 종아리
  90,  // 11 R 발목
  180, // 12 L 고관절     (오른쪽 0의 거울)
  180, // 13 L 허벅지     (오른쪽 0의 거울)
  180, // 14 L 종아리     (오른쪽 0의 거울)
  90   // 15 L 발목       (180-90=90)
};

void writeAngle(int ch, int angle) {
  if (angle < 0)   angle = 0;
  if (angle > 180) angle = 180;
  int pulse = map(angle, 0, 180, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
  pwm.setPWM(ch, 0, pulse);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(100);

  for (int ch = 0; ch < NUM_SERVOS; ch++) {
    writeAngle(ch, home[ch]);
  }

  Serial.println(F("HELIOS home position set. 이제 관절을 하나씩 조립하세요."));
}

void loop() {
  // 홈 위치를 계속 유지 (PCA9685가 알아서 잡아줌)
}
