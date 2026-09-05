/*
 * ==========================================================
 *  HELIOS ☀  —  카메라 미러링 (상체 팔 따라하기) 테스트
 * ==========================================================
 *  보드: Arduino Nano ESP32 (BLE)
 *  서보: PCA9685 16채널
 *
 *  ▶ 컴퓨터의 mirror.py 가 웹캠으로 사람 팔 각도를 계산해서
 *    블루투스로 "채널:각도" 여러 개를 한 번에 보냄.
 *    이 코드는 그걸 받아서 해당 서보를 그 각도로 움직임.
 *
 *  ▶ 받는 메시지 형식 (쉼표로 구분):
 *      "2:120,4:90,5:60,7:100"
 *      → 2번=120도, 4번=90도, 5번=60도, 7번=100도
 *
 *  ▶ 안전: 채널별 측정 범위(limitMin/Max)로 자동 제한됨.
 *
 *  필요 라이브러리: Adafruit PWM Servo Driver, ESP32 BLE(기본 포함)
 *  블루투스 이름: HELIOS-MIRROR
 * ==========================================================
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

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

// 차렷(중립) 자세 — 연결 시작 시 이 자세로
const int neutral[NUM_SERVOS] = {
  90, 90, 90, 0, 0, 90, 180, 180,
  0, 0, 180, 90, 90, 180, 0, 90
};

// BLE UUID (Nordic UART Service)
#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);
BLECharacteristic *txChar;
bool deviceConnected = false;
String bleLine = "";
volatile bool bleHasLine = false;

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

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) { deviceConnected = true; }
  void onDisconnect(BLEServer *s) {
    deviceConnected = false;
    standNeutral();
    BLEDevice::startAdvertising();
  }
};
class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) {
    bleLine = c->getValue().c_str();
    bleHasLine = true;
  }
};

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(100);
  standNeutral();

  BLEDevice::init("HELIOS-MIRROR");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService *service = server->createService(NUS_SERVICE_UUID);
  BLECharacteristic *rxChar = service->createCharacteristic(
    NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxChar->setCallbacks(new RxCallbacks());
  txChar = service->createCharacteristic(
    NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());
  service->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println(F("HELIOS-MIRROR ready. Waiting for pose data..."));
}

void loop() {
  if (bleHasLine) {
    bleHasLine = false;
    applyLine(bleLine);
  }
}
