/*
 * ==========================================================
 *  HELIOS ☀  —  카메라 미러링 (상체 팔 따라하기) — USB 버전
 * ==========================================================
 *  보드: Arduino Nano ESP32
 *  서보: PCA9685 16채널
 *
 *  ▶ 컴퓨터의 mirror.py 가 웹캠으로 사람 팔 각도를 계산해서
 *    USB 시리얼로 "채널:각도" 여러 개를 한 줄로 보냄.
 *    이 코드는 그걸 받아서 해당 서보를 그 각도로 움직임.
 *    (블루투스 대신 USB 케이블로 통신 → 훨씬 안정적)
 *
 *  ▶ 받는 메시지 형식 (한 줄, 줄바꿈으로 끝):
 *      "2:120,4:90,5:60,7:100\n"
 *
 *  ▶ 안전: 채널별 측정 범위(limitMin/Max)로 자동 제한됨.
 *
 *  필요 라이브러리: Adafruit PWM Servo Driver
 *  ⚠ mirror.py 는 이 코드와 통신하려고 같은 USB 포트를 씀.
 *     그래서 이 코드를 올린 뒤에는 아두이노 시리얼 모니터를 닫아두세요.
 * ==========================================================
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PCA9685_ADDR   0x40
#define SERVO_FREQ     50
#define SERVO_PULSE_MIN  150
#define SERVO_PULSE_MAX  560
#define NUM_SERVOS     16

// 채널별 안전 가동 범위 (측정값)
const int limitMin[NUM_SERVOS] = {
  0, 0, 0, 0, 0, 0, 0, 70,
  0, 0, 100, 40, 0, 100, 0, 0
};
const int limitMax[NUM_SERVOS] = {
  179, 180, 180, 180, 130, 180, 180, 180,
  90, 80, 180, 180, 90, 180, 80, 140
};

// 차렷(중립) 자세 — 시작 시 이 자세로
const int neutral[NUM_SERVOS] = {
  90, 90, 90, 0, 0, 90, 180, 180,
  0, 0, 180, 90, 90, 180, 0, 90
};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

void writeAngle(int ch, int angle) {
  if (ch < 0 || ch >= NUM_SERVOS) return;
  if (angle < limitMin[ch]) angle = limitMin[ch];
  if (angle > limitMax[ch]) angle = limitMax[ch];
  int pulse = map(angle, 0, 180, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
  pwm.setPWM(ch, 0, pulse);
}

void standNeutral() {
  for (int ch = 0; ch < NUM_SERVOS; ch++) writeAngle(ch, neutral[ch]);
}

// "2:120,4:90,..." 파싱해서 각 서보에 적용
void applyLine(String line) {
  int start = 0;
  while (start < (int)line.length()) {
    int comma = line.indexOf(',', start);
    if (comma < 0) comma = line.length();
    String pair = line.substring(start, comma);   // "2:120"
    int colon = pair.indexOf(':');
    if (colon > 0) {
      int ch = pair.substring(0, colon).toInt();
      int ang = pair.substring(colon + 1).toInt();
      writeAngle(ch, ang);
    }
    start = comma + 1;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(100);
  standNeutral();
  Serial.println("HELIOS-MIRROR (USB) ready");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) applyLine(line);
  }
}
