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
 *         n = 중립(차렷)
 *         w = 앞으로 걷기
 *         b = 뒤로 걷기
 *         d = 옆으로 걷기(오른쪽)
 *         r = 제자리 회전
 *         a = 손 흔들기
 *         p = 지르기(펀치)
 *         k = 스쿼트
 *         s = 정지
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
#define SERVO_PULSE_MAX  560   // MG90S에 맞게 조정

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

// 중립(차렷) 각도 [채널 0~15] — 조립한 홈 자세와 동일
// 오른쪽 0도 ↔ 왼쪽 180도 (좌우 서보가 거울로 장착됨)
const int neutral[NUM_SERVOS] = {
  98,  // 0  목(머리) (정면 보정)
  94,  // 1  허리 (정면 보정)
  94,  // 2  R 어깨 상하 (보정)
  5,   // 3  R 어깨 벌림 (보정)
  0,   // 4  R 팔꿈치
  90,  // 5  L 어깨 상하
  178, // 6  L 어깨 벌림 (보정)
  180, // 7  L 팔꿈치
  0,   // 8  R 고관절
  0,   // 9  R 허벅지
  170, // 10 R 종아리 (보정)
  100, // 11 R 발목 (보정)
  80,  // 12 L 고관절 (보정)
  170, // 13 L 허벅지 (보정)
  0,   // 14 L 종아리 (재조립: 직립=0)
  85   // 15 L 발목 (보정)
};

// 서보 방향 (반대로 움직이면 부호 뒤집기)
const int servoDir[NUM_SERVOS] = {
  +1, +1, +1, +1, +1, -1, -1, -1,
  +1, +1, +1, +1, -1, -1, -1, -1
};

// 채널별 안전 가동 범위 (측정값). 이 범위를 벗어나는 명령은 자동으로 막힘.
const int limitMin[NUM_SERVOS] = {
  17, 8, 0, 0, 0, 0, 0, 70,   // ch0/ch1: 홈(98/94) 기준 좌우 대칭 하한
  0, 0, 100, 40, 0, 100, 0, 0
};
const int limitMax[NUM_SERVOS] = {
  179, 180, 180, 180, 130, 180, 180, 180,
  90, 80, 180, 180, 90, 180, 80, 140
};

// 걸음걸이 파라미터 (테스트하며 조정) — 크라우치(무릎 굽힌) 걷기
#define GAIT_STEP_DELAY_MS  20   // 프레임 간 지연(작을수록 빠름)
#define CROUCH_KNEE   35   // 크라우치: 무릎 굽힘 (걷는 기본 앉은 자세) ★핵심
#define CROUCH_THIGH  18   // 크라우치: 허벅지 굽힘 (균형)
#define CROUCH_ANKLE  18   // 크라우치: 발목 보정 (발 평평하게)
#define STEP_LIFT     25   // 스윙 발 추가로 들기 (무릎)
#define STEP_STRIDE   25   // 보폭 (허벅지 앞뒤)
#define STEP_LEAN     22   // 무게 좌우 이동 (고관절)
// ▼ 방향 보정 — 테스트해서 반대로 가면 부호(+1/-1)를 뒤집으세요
#define LEAN_DIR     (+1)  // 무게이동 방향
#define ANKLE_P_DIR  (+1)  // 발목 pitch 방향
#define TURN_DIR     (+1)  // 회전 방향

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
  for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) writeAngle(ch, neutral[ch]);
}

// ---------- 다리 관절 제어 (현재 neutral 기준, 좌우 대칭) ----------
// 무릎 굽힘: bend>=0 (0=중립). R종아리 170→감소, L종아리 0→증가
void legKnee(bool R, int bend) {
  int ch = R ? CH_R_CALF : CH_L_CALF;
  writeAngle(ch, neutral[ch] + (R ? -bend : +bend));
}
// 허벅지 앞뒤: fwd>0 앞으로, <0 뒤로. R 허벅지 증가, L 허벅지 감소
void legThigh(bool R, int fwd) {
  int ch = R ? CH_R_THIGH : CH_L_THIGH;
  writeAngle(ch, neutral[ch] + (R ? +fwd : -fwd));
}
// 발목 앞뒤(pitch): 발 평평 유지용. p>0
void legAnkle(bool R, int p) {
  int ch = R ? CH_R_ANKLE : CH_L_ANKLE;
  writeAngle(ch, neutral[ch] + (R ? +p : -p) * ANKLE_P_DIR);
}
// 무게 좌우 이동(고관절). amt>0 = 오른쪽으로 기울임 (양쪽 같은 방향=몸 기울임)
void legLean(int amt) {
  writeAngle(CH_R_HIP, neutral[CH_R_HIP] + amt * LEAN_DIR);
  writeAngle(CH_L_HIP, neutral[CH_L_HIP] + amt * LEAN_DIR);
}
// 한 다리 옆으로 벌리기(고관절). out>0 = 바깥쪽 (옆걸음용)
void legSplay(bool R, int out) {
  int ch = R ? CH_R_HIP : CH_L_HIP;
  writeAngle(ch, neutral[ch] + (R ? +out : -out));
}

// ---------- 크라우치(앉은) 기본 자세 ----------
void crouch() {
  legKnee(true, CROUCH_KNEE);  legKnee(false, CROUCH_KNEE);
  legThigh(true, CROUCH_THIGH); legThigh(false, CROUCH_THIGH);
  legAnkle(true, CROUCH_ANKLE); legAnkle(false, CROUCH_ANKLE);
}

// ---------- 앞/뒤 한 걸음 (dir=+1 앞, -1 뒤 / swingR=오른다리) ----------
void stepLeg(bool swingR, int dir) {
  int st = swingR ? -1 : +1;                 // 지지발 쪽으로 기울임
  legLean(st * STEP_LEAN);  delay(GAIT_STEP_DELAY_MS * 6);   // 1) 무게 지지발로
  legKnee(swingR, CROUCH_KNEE + STEP_LIFT); delay(GAIT_STEP_DELAY_MS * 4); // 2) 발 들기
  legThigh(swingR, CROUCH_THIGH * 0 + dir * STEP_STRIDE);    // 3) 허벅지 방향대로
  delay(GAIT_STEP_DELAY_MS * 4);
  legKnee(swingR, CROUCH_KNEE); delay(GAIT_STEP_DELAY_MS * 4); // 4) 발 내리기
  legLean(0); delay(GAIT_STEP_DELAY_MS * 4);                   // 5) 무게 중앙
  legThigh(true, 0); legThigh(false, 0);                      // 6) 허벅지 복귀(몸 이동)
  delay(GAIT_STEP_DELAY_MS * 3);
}
void walk(int dir, int steps) {   // dir: +1 앞 / -1 뒤
  crouch(); delay(350);
  for (int i = 0; i < steps && walking; i++) {
    stepLeg(true, dir);  if (!walking) break;
    stepLeg(false, dir);
  }
  standNeutral();
}

// ---------- 옆으로 걷기 (toRight=true 오른쪽) ----------
void strafe(bool toRight, int steps) {
  crouch(); delay(350);
  for (int i = 0; i < steps && walking; i++) {
    int st = toRight ? -1 : +1;                // 진행 반대쪽(지지발)으로 기울임
    legLean(st * STEP_LEAN); delay(GAIT_STEP_DELAY_MS * 6);   // 1) 무게 지지발로
    legKnee(toRight, CROUCH_KNEE + STEP_LIFT); delay(GAIT_STEP_DELAY_MS * 4); // 2) 진행발 들기
    legSplay(toRight, STEP_STRIDE); delay(GAIT_STEP_DELAY_MS * 4);  // 3) 진행발 옆으로
    legKnee(toRight, CROUCH_KNEE); delay(GAIT_STEP_DELAY_MS * 4);   // 4) 내리기
    legLean(-st * STEP_LEAN); delay(GAIT_STEP_DELAY_MS * 5);        // 5) 무게 진행발로
    legKnee(!toRight, CROUCH_KNEE + STEP_LIFT); delay(GAIT_STEP_DELAY_MS * 4); // 6) 반대발 들기
    legSplay(toRight, 0); legSplay(!toRight, 0); delay(GAIT_STEP_DELAY_MS * 4); // 7) 모으기
    legKnee(!toRight, CROUCH_KNEE); legLean(0); delay(GAIT_STEP_DELAY_MS * 3);
  }
  standNeutral();
}

// ---------- 제자리 회전 (고관절 벌림 + 허리 비틀기 조합) ----------
// ※ 전용 회전(yaw) 관절이 없어서, 발 벌림+허리로 "비틀며 끌기" 방식 (약하게 돎)
void turnStep(int steps) {
  crouch(); delay(350);
  for (int i = 0; i < steps && walking; i++) {
    // 지지발(오른)로 무게 → 왼발 들어 벌렸다 모으며 허리 비틀기
    legLean(-STEP_LEAN * TURN_DIR); delay(GAIT_STEP_DELAY_MS * 6);
    legKnee(false, CROUCH_KNEE + STEP_LIFT); delay(GAIT_STEP_DELAY_MS * 4);
    writeAngle(CH_WAIST, neutral[CH_WAIST] + 25 * TURN_DIR);   // 허리 비틀기
    legSplay(false, STEP_STRIDE); delay(GAIT_STEP_DELAY_MS * 4);
    legKnee(false, CROUCH_KNEE); delay(GAIT_STEP_DELAY_MS * 4);
    legLean(STEP_LEAN * TURN_DIR); delay(GAIT_STEP_DELAY_MS * 5);
    legKnee(true, CROUCH_KNEE + STEP_LIFT); delay(GAIT_STEP_DELAY_MS * 4);
    writeAngle(CH_WAIST, neutral[CH_WAIST]);                   // 허리 복귀(몸 끌림)
    legSplay(false, 0); delay(GAIT_STEP_DELAY_MS * 4);
    legKnee(true, CROUCH_KNEE); legLean(0); delay(GAIT_STEP_DELAY_MS * 3);
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

// ---------- 지르기(펀치) 콤보 ----------
// 팔 동작은 moveJoint 오프셋으로 좌우 자동 대칭 처리 (servoDir 반영).
// 실제 로봇에서 방향이 어색하면 아래 값의 부호/크기를 조정하세요.
#define GUARD_ELBOW    100  // 가드 시 팔꿈치 굽힘
#define GUARD_SHOULDER  35  // 가드 시 어깨 살짝 들기
#define PUNCH_ELBOW     10  // 지를 때 팔꿈치 펴기 (작을수록 곧게)
#define PUNCH_SHOULDER  70  // 지를 때 어깨 앞으로
#define WAIST_TWIST     40  // 지를 때 몸통 회전 각도

// 몸통 회전 (머리는 반대로 돌려 정면 고정)
//  dir>0 한쪽 / dir<0 반대쪽 / dir=0 정면. 방향이 반대면 부호를 바꾸세요.
void torsoTwist(int dir) {
  int w = 90 + dir * WAIST_TWIST;
  writeAngle(CH_WAIST, w);
  writeAngle(CH_HEAD, 180 - w);   // 머리를 반대로 돌려 정면 유지
}

// 한 팔 가드 자세 (몸통 정면)
void armGuard(bool isRight) {
  moveJoint(isRight ? CH_R_SHO_PITCH : CH_L_SHO_PITCH, GUARD_SHOULDER);
  moveJoint(isRight ? CH_R_ELBOW     : CH_L_ELBOW,     GUARD_ELBOW);
  torsoTwist(0);
}
// 한 팔 지르기 (앞으로 뻗기 + 몸통 회전)
//  오른손 지르기 → 몸통 왼쪽 / 왼손 지르기 → 몸통 오른쪽
void armPunch(bool isRight) {
  moveJoint(isRight ? CH_R_SHO_PITCH : CH_L_SHO_PITCH, PUNCH_SHOULDER);
  moveJoint(isRight ? CH_R_ELBOW     : CH_L_ELBOW,     PUNCH_ELBOW);
  torsoTwist(isRight ? +1 : -1);
}

// 가드 → 오른손 지르기 → 가드 → 왼손 지르기 → 가드 → 차렷
void punchCombo() {
  armGuard(true); armGuard(false);   // 1) 양손 가드 올리기
  delay(400);

  armPunch(true);  delay(250);       // 2) 오른손 지르기
  armGuard(true);  delay(250);       // 3) 오른손 가드 복귀

  armPunch(false); delay(250);       // 4) 왼손 지르기
  armGuard(false); delay(250);       // 5) 왼손 가드 복귀

  delay(200);
  standNeutral();                    // 6) 차렷 자세로
}

// ---------- 무릎 구부리기 (스쿼트) ----------
#define SQUAT_KNEE   30   // 얕은 스쿼트: 중립에서 무릎 30도 굽힘
#define SQUAT_THIGH  14   // 기존 무릎/허벅지 비율 유지 (30 * 30 / 65 반올림)
#define SQUAT_STEPS   4   // 부드럽게 나눌 단계 수

// 양 다리를 amt(0~SQUAT_KNEE)만큼 굽힌 자세로
void squatPose(int knee, int thigh) {
  legKnee(true, knee);  legKnee(false, knee);
  legThigh(true, thigh); legThigh(false, thigh);
}

// 앉았다 일어서기
void squat() {
  // 내려가기 (단계별로 부드럽게)
  for (int i = 1; i <= SQUAT_STEPS; i++) {
    squatPose(SQUAT_KNEE * i / SQUAT_STEPS, SQUAT_THIGH * i / SQUAT_STEPS);
    delay(150);
  }
  delay(500);   // 앉은 자세 유지
  // 올라오기
  for (int i = SQUAT_STEPS - 1; i >= 0; i--) {
    squatPose(SQUAT_KNEE * i / SQUAT_STEPS, SQUAT_THIGH * i / SQUAT_STEPS);
    delay(150);
  }
  standNeutral();
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
      if (ch=='n'||ch=='w'||ch=='b'||ch=='d'||ch=='r'||ch=='a'||ch=='s'||ch=='p'||ch=='k') pendingCmd = ch;
    }
  }
};

// ---------- 명령 처리 ----------
void handleCommand(char cmd) {
  switch (cmd) {
    case 'n': walking = false; standNeutral();       notifyPhone("neutral\n"); break;
    case 'w': walking = true;  notifyPhone("forward\n");  walk(+1, 4); walking = false; break;
    case 'b': walking = true;  notifyPhone("backward\n"); walk(-1, 4); walking = false; break;
    case 'd': walking = true;  notifyPhone("strafe\n");   strafe(true, 4); walking = false; break;
    case 'r': walking = true;  notifyPhone("turn\n");     turnStep(4); walking = false; break;
    case 'a': notifyPhone("wave\n"); waveHand();      break;
    case 'p': notifyPhone("punch\n"); punchCombo();   break;
    case 'k': notifyPhone("squat\n"); squat();        break;
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
