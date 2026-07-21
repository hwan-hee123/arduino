/*
 * config.h — HELIOS 휴머노이드 로봇 설정 (PCA9685 16채널)
 * ---------------------------------------------------------
 * 구성: 머리1 + 허리1 + 양팔(각3) + 양다리(각4) = 16 서보
 * 서보 드라이버: PCA9685 (I2C, 기본 주소 0x40)
 *
 * 실제 조립에 맞게 NEUTRAL(중립각)과 DIR(방향)만 보정하면 됩니다.
 */

#ifndef HELIOS_CONFIG_H
#define HELIOS_CONFIG_H

// ===== PCA9685 설정 =====
#define PCA9685_ADDR   0x40
#define SERVO_FREQ     50      // 아날로그 서보 = 50Hz

// 서보 펄스 길이 (4096 분해능 기준). 서보에 따라 미세 조정.
#define SERVO_PULSE_MIN  150   // 약 0도
#define SERVO_PULSE_MAX  600   // 약 180도

// ===== 채널 번호 (사용자 배선표 그대로) =====
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

// ===== 서보 각도 안전 한계 =====
#define SERVO_MIN_ANGLE  10
#define SERVO_MAX_ANGLE  170

// ===== 중립(차렷) 자세 각도 [채널 0~15 순서] =====
// 조립 후 로봇이 똑바로 서도록 각 값을 보정하세요.
#define NEUTRAL_ANGLES { \
  90, /* 0 머리        */ \
  90, /* 1 허리        */ \
  90, /* 2 R 어깨 상하 */ \
  90, /* 3 R 어깨 벌림 */ \
  90, /* 4 R 팔꿈치    */ \
  90, /* 5 L 어깨 상하 */ \
  90, /* 6 L 어깨 벌림 */ \
  90, /* 7 L 팔꿈치    */ \
  90, /* 8 R 고관절    */ \
  90, /* 9 R 허벅지    */ \
  90, /* 10 R 종아리   */ \
  90, /* 11 R 발목     */ \
  90, /* 12 L 고관절   */ \
  90, /* 13 L 허벅지   */ \
  90, /* 14 L 종아리   */ \
  90  /* 15 L 발목     */ \
}

// ===== 서보 방향 (조립 방향에 따라 +1 또는 -1) =====
// 좌우 다리/팔은 보통 서로 반대 방향으로 장착되므로 부호가 반대인 경우가 많습니다.
#define SERVO_DIRECTIONS { \
  +1, /* 0 머리        */ \
  +1, /* 1 허리        */ \
  +1, /* 2 R 어깨 상하 */ \
  +1, /* 3 R 어깨 벌림 */ \
  +1, /* 4 R 팔꿈치    */ \
  -1, /* 5 L 어깨 상하 */ \
  -1, /* 6 L 어깨 벌림 */ \
  -1, /* 7 L 팔꿈치    */ \
  +1, /* 8 R 고관절    */ \
  +1, /* 9 R 허벅지    */ \
  +1, /* 10 R 종아리   */ \
  +1, /* 11 R 발목     */ \
  -1, /* 12 L 고관절   */ \
  -1, /* 13 L 허벅지   */ \
  -1, /* 14 L 종아리   */ \
  -1  /* 15 L 발목     */ \
}

// ===== 걸음걸이 파라미터 =====
#define GAIT_STEP_DELAY_MS  20   // 프레임 간 지연(작을수록 빠름)
#define GAIT_SWING_AMP      20   // 다리 앞뒤 스윙 각도(허벅지)
#define GAIT_LIFT_AMP       20   // 발 들기 각도(종아리)
#define GAIT_SHIFT_AMP      12   // 무게중심 좌우 이동(고관절/발목)

#endif // HELIOS_CONFIG_H
