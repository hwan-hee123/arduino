/*
 * ==========================================================
 *  HELIOS ☀  —  캘리브레이션 모드 (관절 하나씩 맞추기)
 * ==========================================================
 *  보드: Arduino Nano ESP32
 *  서보: PCA9685 16채널
 *
 *  ▶ 목적: 서보를 하나씩 원하는 각도로 보내서
 *          - 조립 위치(0/90도)로 되돌리고
 *          - 각 관절의 안전한 가동 범위를 찾는다
 *
 *  ▶ 명령 형식 (시리얼 모니터 9600 또는 nRF Connect 텍스트):
 *      "9 45"     → 9번 채널을 45도로
 *      "all 90"   → 전체를 90도로
 *      "?"        → 현재 각도 목록 출력 (시리얼)
 *
 *  ⚠ 시작하면 서보에 힘이 들어가지 않습니다(limp).
 *     명령을 보낸 채널만 그 각도로 힘이 걸립니다.
 *     → 관절을 하나씩 천천히 조금씩 움직여서 안전 범위를 확인하세요.
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
#define SERVO_PULSE_MAX  600
#define NUM_SERVOS     16

// BLE UUID (Nordic UART Service)
#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// 채널 이름 (출력용)
const char *chName[NUM_SERVOS] = {
  "0 head", "1 waist",
  "2 R-sho-updn", "3 R-sho-open", "4 R-elbow",
  "5 L-sho-updn", "6 L-sho-open", "7 L-elbow",
  "8 R-hip", "9 R-thigh", "10 R-calf", "11 R-ankle",
  "12 L-hip", "13 L-thigh", "14 L-calf", "15 L-ankle"
};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

int  curAngle[NUM_SERVOS];      // 마지막으로 보낸 각도 (-1 = 아직 안 보냄/limp)
bool deviceConnected = false;
BLECharacteristic *txChar;
String bleLine = "";            // BLE로 받은 명령 버퍼
bool bleHasLine = false;

// ---------- 서보 출력 ----------
void writeAngle(int ch, int angle) {
  if (ch < 0 || ch >= NUM_SERVOS) return;
  if (angle < 0)   angle = 0;
  if (angle > 180) angle = 180;
  int pulse = map(angle, 0, 180, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
  pwm.setPWM(ch, 0, pulse);
  curAngle[ch] = angle;
}

// ---------- 폰으로 응답 ----------
void notifyPhone(const String &msg) {
  Serial.print(msg);
  if (deviceConnected && txChar) {
    txChar->setValue((uint8_t *)msg.c_str(), msg.length());
    txChar->notify();
  }
}

// ---------- 현재 각도 출력 ----------
void printAll() {
  String s = "--- angles ---\n";
  for (int i = 0; i < NUM_SERVOS; i++) {
    s += chName[i];
    s += " = ";
    s += (curAngle[i] < 0 ? String("limp") : String(curAngle[i]));
    s += "\n";
  }
  notifyPhone(s);
}

// ---------- 명령 한 줄 처리 ----------
// "9 45"  |  "all 90"  |  "?"
void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "?") { printAll(); return; }

  int sp = line.indexOf(' ');
  if (sp < 0) { notifyPhone("format: <ch> <angle>\n"); return; }

  String a = line.substring(0, sp);
  String b = line.substring(sp + 1);
  a.trim(); b.trim();
  int angle = b.toInt();

  if (a == "all") {
    for (int i = 0; i < NUM_SERVOS; i++) writeAngle(i, angle);
    notifyPhone("all -> " + String(angle) + "\n");
    return;
  }

  int ch = a.toInt();
  if (ch < 0 || ch >= NUM_SERVOS) { notifyPhone("bad channel\n"); return; }
  writeAngle(ch, angle);
  notifyPhone(String(chName[ch]) + " -> " + String(angle) + "\n");
}

// ---------- BLE 콜백 ----------
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) { deviceConnected = true; }
  void onDisconnect(BLEServer *s) {
    deviceConnected = false;
    BLEDevice::startAdvertising();
  }
};
class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) {
    String v = c->getValue().c_str();
    bleLine = v;
    bleHasLine = true;
  }
};

// ---------- 셋업 / 루프 ----------
void setup() {
  Serial.begin(9600);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(100);

  for (int i = 0; i < NUM_SERVOS; i++) curAngle[i] = -1;  // 전부 limp 상태로 시작

  BLEDevice::init("HELIOS-CAL");
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

  Serial.println(F("HELIOS calibration ready. Advertising as 'HELIOS-CAL'."));
  Serial.println(F("Commands:  <ch> <angle>   e.g. '9 45'   |   'all 90'   |   '?'"));
}

void loop() {
  // 시리얼 입력 (USB)
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleLine(line);
  }
  // BLE 입력
  if (bleHasLine) {
    bleHasLine = false;
    handleLine(bleLine);
  }
}
