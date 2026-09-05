"""
HELIOS ☀ — 카메라 미러링 (상체 팔 따라하기) 테스트
====================================================
웹캠으로 사람 팔 각도를 계산해서, 블루투스(BLE)로 로봇에 보냄.
로봇에는 helios_mirror.ino 를 올려둬야 함 (블루투스 이름: HELIOS-MIRROR).

미러링하는 관절 (4개):
  2번 오른어깨 상하, 4번 오른팔꿈치, 5번 왼어깨 상하, 7번 왼팔꿈치

필요 설치 (터미널에서 한 번):
  pip install opencv-python mediapipe bleak numpy

실행:
  python mirror.py

종료: 영상 창에서 q 키
"""

import asyncio
import math
import cv2
import mediapipe as mp
import numpy as np
from bleak import BleakScanner, BleakClient

# Windows: 웹캠(GUI)과 블루투스(bleak)를 같이 쓸 때 생기는 COM 충돌 해결
# ("Thread is configured for Windows GUI but callbacks are not working" 방지)
try:
    from bleak.backends.winrt.util import allow_sta
    allow_sta()
except Exception:
    pass

# ---- 설정 ----
DEVICE_NAME = "HELIOS-MIRROR"
NUS_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # 폰/PC → 로봇
SEND_HZ = 12                 # 초당 전송 횟수 (너무 빠르면 끊김)
SMOOTH = 0.5                 # 각도 부드럽게 (0=즉시, 1=아주 느림)
FLIP_LEFT_RIGHT = False      # 좌우가 반대로 움직이면 True 로

mp_pose = mp.solutions.pose
mp_draw = mp.solutions.drawing_utils


def angle_at(a, b, c):
    """점 b에서 a-b, c-b 사이 각도(도). 팔꿈치/어깨 각도 계산용."""
    a, b, c = np.array(a), np.array(b), np.array(c)
    ba, bc = a - b, c - b
    cosang = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-6)
    return math.degrees(math.acos(np.clip(cosang, -1.0, 1.0)))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def compute_servo_angles(lm, w, h):
    """MediaPipe 랜드마크 → 로봇 팔 서보 각도(채널별)."""
    def pt(i):
        return (lm[i].x * w, lm[i].y * h)

    # 오른팔: 어깨12 팔꿈치14 손목16, 엉덩이24
    # 왼팔:   어깨11 팔꿈치13 손목15, 엉덩이23
    r_elbow = angle_at(pt(12), pt(14), pt(16))   # 180=쭉, 작을수록 굽힘
    l_elbow = angle_at(pt(11), pt(13), pt(15))
    r_shldr = angle_at(pt(24), pt(12), pt(14))   # 팔을 들수록 커짐
    l_shldr = angle_at(pt(23), pt(11), pt(13))

    # 팔꿈치 굽힘량 (0=쭉, 커질수록 굽힘)
    r_bend = clamp(180 - r_elbow, 0, 130)
    l_bend = clamp(180 - l_elbow, 0, 130)

    # 어깨 들기: 사람 어깨각(약 10~160) → 서보 오프셋(0~90)
    r_lift = clamp((r_shldr - 15) * 0.7, 0, 90)
    l_lift = clamp((l_shldr - 15) * 0.7, 0, 90)

    # 로봇 서보 각도로 변환 (홈/방향 반영)
    ch2 = clamp(90 + r_lift, 0, 180)     # 오른어깨: 90에서 증가
    ch4 = clamp(0 + r_bend, 0, 130)      # 오른팔꿈치: 0에서 증가
    ch5 = clamp(90 - l_lift, 0, 180)     # 왼어깨: 90에서 감소(거울)
    ch7 = clamp(180 - l_bend, 70, 180)   # 왼팔꿈치: 180에서 감소(거울)

    if FLIP_LEFT_RIGHT:
        ch2, ch5 = clamp(90 + l_lift, 0, 180), clamp(90 - r_lift, 0, 180)
        ch4, ch7 = clamp(0 + l_bend, 0, 130), clamp(180 - r_bend, 70, 180)

    return {2: int(ch2), 4: int(ch4), 5: int(ch5), 7: int(ch7)}


async def find_device():
    print(f"'{DEVICE_NAME}' 찾는 중...")
    d = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
    if d is None:
        raise RuntimeError(f"'{DEVICE_NAME}' 를 못 찾음. 로봇 전원/블루투스 확인.")
    return d


async def main():
    device = await find_device()
    async with BleakClient(device) as client:
        print("연결됨! 웹캠 시작. (창에서 q 로 종료)")
        cap = cv2.VideoCapture(0)
        pose = mp_pose.Pose(model_complexity=0,
                            min_detection_confidence=0.5,
                            min_tracking_confidence=0.5)
        smoothed = {2: 90, 4: 0, 5: 90, 7: 180}
        interval = 1.0 / SEND_HZ
        last = 0.0

        while cap.isOpened():
            ok, frame = cap.read()
            if not ok:
                break
            frame = cv2.flip(frame, 1)  # 거울처럼 보이게
            h, w = frame.shape[:2]
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res = pose.process(rgb)

            if res.pose_landmarks:
                mp_draw.draw_landmarks(frame, res.pose_landmarks,
                                       mp_pose.POSE_CONNECTIONS)
                targets = compute_servo_angles(res.pose_landmarks.landmark, w, h)
                # 부드럽게
                for ch, val in targets.items():
                    smoothed[ch] = smoothed[ch] * SMOOTH + val * (1 - SMOOTH)

                now = asyncio.get_event_loop().time()
                if now - last >= interval:
                    last = now
                    msg = ",".join(f"{ch}:{int(a)}" for ch, a in smoothed.items())
                    try:
                        await client.write_gatt_char(NUS_RX_UUID,
                                                     msg.encode(), response=False)
                    except Exception as e:
                        print("전송 오류:", e)
                    cv2.putText(frame, msg, (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("HELIOS mirror (q=quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        print("종료.")


if __name__ == "__main__":
    asyncio.run(main())
