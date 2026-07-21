/*
 * config.h — HELIOS 이족 보행 로봇 하드웨어 설정
 * ---------------------------------------------------------
 * 실제 하드웨어에 맞게 이 파일의 값만 바꾸면 됩니다.
 *
 * 기본 구성: 다리당 3개 서보(엉덩이/무릎/발목) = 총 6개
 *   - 직접 서보 연결 (Arduino Uno/Nano PWM 핀)
 *   - 서보 드라이버(PCA9685)를 쓴다면 USE_PCA9685 를 1로
 */

#ifndef HELIOS_CONFIG_H
#define HELIOS_CONFIG_H

// ===== 서보 드라이버 선택 =====
#define USE_PCA9685 0   // 0 = 아두이노 핀에 직접 연결, 1 = PCA9685 사용

// ===== 서보 핀 배치 (직접 연결 시) =====
// 왼쪽 다리
#define PIN_L_HIP    3   // 왼쪽 엉덩이
#define PIN_L_KNEE   5   // 왼쪽 무릎
#define PIN_L_ANKLE  6   // 왼쪽 발목
// 오른쪽 다리
#define PIN_R_HIP    9   // 오른쪽 엉덩이
#define PIN_R_KNEE   10  // 오른쪽 무릎
#define PIN_R_ANKLE  11  // 오른쪽 발목

#define NUM_SERVOS   6

// ===== 서보 각도 한계 (물리적 손상 방지) =====
#define SERVO_MIN_ANGLE  10
#define SERVO_MAX_ANGLE  170

// ===== 중립(서 있는) 자세 각도 =====
// 조립 후 로봇이 똑바로 서도록 이 값들을 보정하세요.
#define NEUTRAL_L_HIP    90
#define NEUTRAL_L_KNEE   90
#define NEUTRAL_L_ANKLE  90
#define NEUTRAL_R_HIP    90
#define NEUTRAL_R_KNEE   90
#define NEUTRAL_R_ANKLE  90

// ===== 서보 방향 (조립 방향에 따라 +1 또는 -1) =====
#define DIR_L_HIP    (+1)
#define DIR_L_KNEE   (+1)
#define DIR_L_ANKLE  (+1)
#define DIR_R_HIP    (-1)
#define DIR_R_KNEE   (-1)
#define DIR_R_ANKLE  (-1)

// ===== 걸음걸이 기본 파라미터 =====
#define GAIT_STEP_DELAY_MS  20    // 각 동작 프레임 간 지연(작을수록 빠름)
#define GAIT_SWING_AMP      20    // 다리 앞뒤 스윙 각도
#define GAIT_LIFT_AMP       15    // 발 들어올리는 각도

#endif // HELIOS_CONFIG_H
