/*
 * ==========================================================
 *  HELIOS ☀  —  휴머노이드 로봇 (BLE 블루투스 버전)
 * ==========================================================
 *  보드: Arduino Nano ESP32  (BLE 내장)
 *  서보: PCA9685 16채널 드라이버
 *
 *  ▶ 이 파일 하나만 아두이노 IDE에 복붙 → 업로드.
 *
 *  필요 라이브러리:
 *    - Adafruit PWM Servo Driver Library   (라이브러리 매니저에서 설치)
 *    - ESP32 BLE (BLEDevice.h)             ← ESP32 보드 패키지에 기본 포함
 *
 *  ▶ 휴대폰 사용법 (nRF Connect 앱):
 *    1) 앱에서 SCAN → "HELIOS" 장치 찾기 → CONNECT
 *    2) Nordic UART Service (UUID 6E40...) 펼치기
 *    3) RX 특성(...0002...)의 화살표(⬆ Write) 누르기
 *    4) 데이터 타입을 "Text"로 바꾸고 아래 글자 입력 → SEND
 *         n = 중립(차렷)  |  w = 걷기  |  a = 손흔들기  |  s = 정지
 * ==========================================================
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ==========================================================
//  ▼▼▼ 설정 (하드웨어에 맞게 이 부분만 수정) ▼▼▼
// ==========================================================

#define PCA9685_ADDR   0x40
#define SERVO_FREQ     50
#define SERVO_PULSE_MIN  150
#define SERVO_PULSE_MAX  600

// 채널 번호 (배선표 그대로)
#define CH_HEAD        0
#define CH_WAIST       1
#define CH_R_SHO_PITCH 2
#define CH_R_SHO_ROLL  3
#define CH_R_ELBOW     4
#define CH_L_SHO_PITCH 5
#define CH_L_SHO_ROLL  6
#define CH_L_ELBOW     7
#define CH_R_HIP       8
#define CH_R_THIGH     9
#define CH_R_CALF      10
#define CH_R_ANKLE     11
#define CH_L_HIP       12
#define CH_L_THIGH     13
#define CH_L_CALF      14
#define CH_L_ANKLE     15
#define NUM_SERVOS     16

#define SERVO_MIN_ANGLE  0
#define SERVO_MAX_ANGLE  180

// 중립(차렷) 각도 [채널 0~15] — 실제 조립값
// 목/허리/어깨상하/양발목 = 90도, 나머지 = 0도로 조립함
const int neutral[NUM_SERVOS] = {
  90, // 0  목(머리)
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

// 서보 방향 (반대로 움직이면 부호 뒤집기)
const int servoDir[NUM_SERVOS] = {
  +1, +1, +1, +1, +1, -1, -1, -1,
  +1, +1, +1, +1, -1, -1, -1, -1
};

#define GAIT_STEP_DELAY_MS  20
#define GAIT_SWING_AMP      20
#define GAIT_LIFT_AMP       20
#define GAIT_SHIFT_AMP      12

// BLE UUID (Nordic UART Service)
#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // 폰 → 로봇
#define NUS_TX_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // 로봇 → 폰

// ==========================================================
//  ▲▲▲ 설정 끝 ▲▲▲
// ==========================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);
bool walking = false;

volatile char pendingCmd = 0;   // BLE 콜백이 여기에 명령을 넣음
bool deviceConnected = false;
BLECharacteristic *txChar;

// ---------- 서보 저수준 ----------
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
  for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) writeAngle(ch, neutral[ch]);
}

// ---------- 걸음걸이 ----------
void takeStep(bool swingRight) {
  uint8_t sw_hip   = swingRight ? CH_R_HIP   : CH_L_HIP;
  uint8_t sw_thigh = swingRight ? CH_R_THIGH : CH_L_THIGH;
  uint8_t sw_calf  = swingRight ? CH_R_CALF  : CH_L_CALF;
  uint8_t sw_ankle = swingRight ? CH_R_ANKLE : CH_L_ANKLE;
  uint8_t st_hip   = swingRight ? CH_L_HIP   : CH_R_HIP;
  uint8_t st_ankle = swingRight ? CH_L_ANKLE : CH_R_ANKLE;

  moveJoint(sw_hip, +GAIT_SHIFT_AMP);  moveJoint(st_hip, +GAIT_SHIFT_AMP);
  moveJoint(sw_ankle, -GAIT_SHIFT_AMP); moveJoint(st_ankle, -GAIT_SHIFT_AMP);
  delay(GAIT_STEP_DELAY_MS * 5);
  moveJoint(sw_calf, +GAIT_LIFT_AMP);   delay(GAIT_STEP_DELAY_MS * 3);
  moveJoint(sw_thigh, +GAIT_SWING_AMP); delay(GAIT_STEP_DELAY_MS * 4);
  moveJoint(sw_calf, 0);                delay(GAIT_STEP_DELAY_MS * 3);
  moveJoint(sw_hip, 0);  moveJoint(st_hip, 0);
  moveJoint(sw_ankle, 0); moveJoint(st_ankle, 0);
  moveJoint(sw_thigh, 0); delay(GAIT_STEP_DELAY_MS * 3);
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
  moveJoint(CH_R_SHO_ROLL, 60); delay(300);
  for (int i = 0; i < 3; i++) {
    moveJoint(CH_R_ELBOW, 30);  delay(250);
    moveJoint(CH_R_ELBOW, -10); delay(250);
  }
  moveJoint(CH_R_ELBOW, 0);
  moveJoint(CH_R_SHO_ROLL, 0);
}

// ---------- 폰으로 상태 문자 보내기 ----------
void notifyPhone(const char *msg) {
  if (deviceConnected && txChar) {
    txChar->setValue((uint8_t *)msg, strlen(msg));
    txChar->notify();
  }
}

// ---------- BLE 콜백 ----------
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) { deviceConnected = true; }
  void onDisconnect(BLEServer *s) {
    deviceConnected = false;
    walking = false;
    BLEDevice::startAdvertising();   // 다시 광고해서 재연결 가능하게
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) {
    String v = c->getValue().c_str();
    for (unsigned int i = 0; i < v.length(); i++) {
      char ch = v[i];
      if (ch=='n'||ch=='w'||ch=='a'||ch=='s') pendingCmd = ch;
    }
  }
};

// ---------- 명령 처리 ----------
void handleCommand(char cmd) {
  switch (cmd) {
    case 'n': walking = false; standNeutral();       notifyPhone("neutral\n"); break;
    case 'w': walking = true;  notifyPhone("walk\n"); walkForward(4); walking = false; break;
    case 'a': notifyPhone("wave\n"); waveHand();      break;
    case 's': walking = false; notifyPhone("stop\n"); break;
  }
}

// ---------- 셋업 / 루프 ----------
void setup() {
  Serial.begin(9600);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(100);
  standNeutral();

  // BLE 시작
  BLEDevice::init("HELIOS");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(NUS_SERVICE_UUID);

  // RX: 폰 → 로봇 (Write)
  BLECharacteristic *rxChar = service->createCharacteristic(
    NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxChar->setCallbacks(new RxCallbacks());

  // TX: 로봇 → 폰 (Notify)
  txChar = service->createCharacteristic(
    NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println(F("HELIOS BLE ready. Advertising as 'HELIOS'."));
}

void loop() {
  // 유선 시리얼로도 명령 가능
  if (Serial.available()) {
    char c = Serial.read();
    if (c=='n'||c=='w'||c=='a'||c=='s') pendingCmd = c;
  }
  // BLE로 받은 명령 처리
  if (pendingCmd) {
    char c = pendingCmd;
    pendingCmd = 0;
    handleCommand(c);
  }
}
