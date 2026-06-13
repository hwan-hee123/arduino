// kinematics.h
// 역기구학 (Inverse Kinematics) 솔버
// 발 끝 목표 위치(x, y) → 허리/무릎 서보 각도 계산
// 향후 동적 보행, 장애물 회피에 활용

#pragma once
#include <math.h>

// 다리 링크 길이 (cm 단위, 실제 로봇에 맞게 수정)
const float THIGH_LEN = 7.0f;
const float SHIN_LEN  = 7.0f;

// IK 풀이 결과
struct IKResult {
  float hipAngle;   // 서보 각도 (0~180)
  float kneeAngle;  // 서보 각도 (0~180)
  bool  valid;      // 도달 가능 여부
};

// 발 끝 좌표 (x=앞뒤, y=위아래 cm) → 관절 각도 반환
// x 양수 = 앞, y 음수 = 아래 (중력 방향)
// 예: solveIK(0, -14)  → 발을 정직하게 14cm 아래
//     solveIK(3, -12)  → 발을 앞으로 3cm, 아래로 12cm
IKResult solveIK(float x, float y) {
  IKResult result = { 90.0f, 90.0f, false };

  float dist = sqrtf(x * x + y * y);
  float maxReach = THIGH_LEN + SHIN_LEN;
  float minReach = fabsf(THIGH_LEN - SHIN_LEN);

  if (dist > maxReach || dist < minReach)
    return result;   // 도달 불가

  // 코사인 법칙으로 무릎 관절각 계산
  float cosKnee = (THIGH_LEN * THIGH_LEN + SHIN_LEN * SHIN_LEN - dist * dist)
                  / (2.0f * THIGH_LEN * SHIN_LEN);
  cosKnee = constrain(cosKnee, -1.0f, 1.0f);
  float kneeJoint = acosf(cosKnee) * 180.0f / M_PI;  // 관절각 (도)

  // 허리 각도 계산 (수직 아래=0 기준)
  float alpha = atan2f(x, -y);                        // 발 방향각
  float cosAlpha2 = (THIGH_LEN * THIGH_LEN + dist * dist - SHIN_LEN * SHIN_LEN)
                    / (2.0f * THIGH_LEN * dist);
  cosAlpha2 = constrain(cosAlpha2, -1.0f, 1.0f);
  float beta = acosf(cosAlpha2);
  float hipJoint = (alpha - beta) * 180.0f / M_PI;   // 허리 관절각 (도)

  // 서보 각도로 변환 (90도 = 중립, 수직)
  result.hipAngle  = 90.0f + hipJoint;
  result.kneeAngle = 180.0f - kneeJoint;  // 서보 마운팅 방향에 따라 부호 조정
  result.valid     = true;

  return result;
}

// 사용 예시:
//   IKResult ik = solveIK(2.0, -13.0);  // 발을 앞으로 2cm, 아래로 13cm
//   if (ik.valid) {
//     servos[0].write((int)ik.hipAngle);
//     servos[1].write((int)ik.kneeAngle);
//   }
