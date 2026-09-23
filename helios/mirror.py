"""
HELIOS ☀ — 카메라 미러링 (상체 팔 따라하기) — USB 버전
======================================================
웹캠으로 사람 팔 각도를 계산해서, USB 시리얼로 로봇에 보냄.
로봇에는 helios_mirror.ino (USB 버전) 를 올려둬야 함.

미러링하는 관절 (4개):
  2번 오른어깨 상하, 4번 오른팔꿈치, 5번 왼어깨 상하, 7번 왼팔꿈치

필요 설치 (터미널에서 한 번):
  py -3.12 -m pip install opencv-python "mediapipe==0.10.14" pyserial numpy
  (이미 mediapipe 등 깔았으면 pyserial 만 추가로: py -3.12 -m pip install pyserial)

실행:
  python mirror.py
종료: 영상 창에서 q 키

⚠ 실행 전 아두이노 IDE의 시리얼 모니터는 닫아두세요 (포트 충돌).
"""

import math
import time
import sys
import cv2
import mediapipe as mp
import numpy as np
import serial
import serial.tools.list_ports

# ---- 설정 ----
PORT = ""                    # 비워두면 자동 탐색. 안 되면 "COM5" 처럼 직접 지정
BAUD = 115200
SEND_HZ = 15                 # 초당 전송 횟수
SMOOTH = 0.5                 # 각도 부드럽게 (0=즉시, 1=아주 느림)
FLIP_LEFT_RIGHT = False      # 좌우가 반대로 움직이면 True 로

# 각 동작의 민감도 (움직임이 부족/과하면 숫자 조정)
HEAD_SCALE = 160             # 머리 좌우 회전
WAIST_SCALE = 120            # 허리 좌우 기울기
SHO_PITCH_SCALE = 110        # 어깨 상하(팔 들기)
SHO_ROLL_SCALE = 150         # 어깨 옆으로 벌림

mp_pose = mp.solutions.pose
mp_draw = mp.solutions.drawing_utils


def angle_at(a, b, c):
    """점 b에서 a-b, c-b 사이 각도(도)."""
    a, b, c = np.array(a), np.array(b), np.array(c)
    ba, bc = a - b, c - b
    cosang = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-6)
    return math.degrees(math.acos(np.clip(cosang, -1.0, 1.0)))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _arm(S, E, Wr):
    """한 팔의 (들기, 벌림, 팔꿈치굽힘) 계산."""
    eps = 1e-6
    vx, vy = E[0] - S[0], E[1] - S[1]
    armlen = math.hypot(vx, vy) + eps
    horiz = abs(vx) / armlen             # 옆으로 벌린 정도 0..1
    vert_up = (S[1] - E[1]) / armlen     # 위로 든 정도 -1..1 (y는 아래가 +)
    roll = clamp(horiz * SHO_ROLL_SCALE, 0, 180)
    lift = clamp((vert_up + 0.2) * SHO_PITCH_SCALE, 0, 90)
    bend = clamp(180 - angle_at(S, E, Wr), 0, 130)
    return lift, roll, bend


def compute_servo_angles(lm, w, h):
    def pt(i):
        return (lm[i].x * w, lm[i].y * h)

    eps = 1e-6
    Ls, Rs = pt(11), pt(12)              # 어깨 L/R
    Le, Re = pt(13), pt(14)              # 팔꿈치
    Lw, Rw = pt(15), pt(16)              # 손목
    Lh, Rh = pt(23), pt(24)             # 엉덩이
    nose = pt(0)
    sho_w = math.hypot(Ls[0] - Rs[0], Ls[1] - Rs[1]) + eps
    midS = ((Ls[0] + Rs[0]) / 2, (Ls[1] + Rs[1]) / 2)
    midH = ((Lh[0] + Rh[0]) / 2, (Lh[1] + Rh[1]) / 2)

    # 머리 좌우 회전 (코가 어깨 중심에서 벗어난 정도)
    head = clamp(90 + (nose[0] - midS[0]) / sho_w * HEAD_SCALE, 0, 179)
    # 허리 좌우 기울기 (어깨중심이 엉덩이중심에서 벗어난 정도)
    waist = clamp(90 + (midS[0] - midH[0]) / sho_w * WAIST_SCALE, 0, 180)

    r_lift, r_roll, r_bend = _arm(Rs, Re, Rw)   # 사람 오른팔
    l_lift, l_roll, l_bend = _arm(Ls, Le, Lw)   # 사람 왼팔

    if FLIP_LEFT_RIGHT:
        r_lift, r_roll, r_bend, l_lift, l_roll, l_bend = \
            l_lift, l_roll, l_bend, r_lift, r_roll, r_bend
        head = clamp(180 - head, 0, 179)
        waist = clamp(180 - waist, 0, 180)

    return {
        0: int(head),                          # 머리
        1: int(waist),                         # 허리
        2: int(clamp(90 + r_lift, 0, 180)),    # R 어깨 상하
        3: int(clamp(0 + r_roll, 0, 180)),     # R 어깨 벌림 (home 0)
        4: int(clamp(0 + r_bend, 0, 130)),     # R 팔꿈치
        5: int(clamp(90 - l_lift, 0, 180)),    # L 어깨 상하 (거울)
        6: int(clamp(180 - l_roll, 0, 180)),   # L 어깨 벌림 (home 180)
        7: int(clamp(180 - l_bend, 70, 180)),  # L 팔꿈치 (거울)
    }


def find_port():
    if PORT:
        return PORT
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("!! 시리얼 포트를 못 찾음. 로봇 USB 연결 확인.")
        sys.exit(1)
    # ESP32/Arduino 로 보이는 포트 우선
    for p in ports:
        desc = (p.description or "").lower()
        if any(k in desc for k in ("esp32", "arduino", "usb serial", "cp210", "ch340")):
            print(f"포트 자동 선택: {p.device}  ({p.description})")
            return p.device
    # 못 고르면 목록 보여주고 첫 번째 사용
    print("포트 목록:")
    for p in ports:
        print(f"  - {p.device}  ({p.description})")
    print(f"→ 첫 번째 사용: {ports[0].device} (틀리면 위 PORT 에 직접 지정)")
    return ports[0].device


def main():
    port = find_port()
    print(f"'{port}' 연결 중...")
    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = BAUD
        ser.timeout = 0.1
        ser.write_timeout = 0.5
        # ESP32가 포트 열 때 리셋/멈춤에 빠지지 않도록 DTR/RTS 내림
        ser.dtr = False
        ser.rts = False
        ser.open()
    except Exception as e:
        print("!! 포트 열기 실패:", e)
        print("   → 아두이노 시리얼 모니터가 열려 있으면 닫으세요 (포트 충돌).")
        return
    time.sleep(2.0)   # 보드 안정화 대기
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print("연결됨! 웹캠 여는 중...")

    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("!! 웹캠을 못 엶. 다른 프로그램(줌 등)이 쓰는지 확인.")
        ser.close()
        return
    # 성능: 해상도 낮추고 버퍼 최소화 (끊김 방지)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    print("웹캠 시작! (창에서 q 로 종료)")

    pose = mp_pose.Pose(model_complexity=0,
                        min_detection_confidence=0.5,
                        min_tracking_confidence=0.5)
    smoothed = {0: 90, 1: 90, 2: 90, 3: 0, 4: 0, 5: 90, 6: 180, 7: 180}
    interval = 1.0 / SEND_HZ
    last = 0.0

    while cap.isOpened():
        ok, frame = cap.read()
        if not ok:
            break
        frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = pose.process(rgb)

        if res.pose_landmarks:
            mp_draw.draw_landmarks(frame, res.pose_landmarks,
                                   mp_pose.POSE_CONNECTIONS)
            targets = compute_servo_angles(res.pose_landmarks.landmark, w, h)
            for ch, val in targets.items():
                smoothed[ch] = smoothed[ch] * SMOOTH + val * (1 - SMOOTH)

            now = time.time()
            if now - last >= interval:
                last = now
                msg = ",".join(f"{ch}:{int(a)}" for ch, a in smoothed.items())
                try:
                    ser.write((msg + "\n").encode())
                except Exception as e:
                    print("전송 오류:", e)
                cv2.putText(frame, msg, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("HELIOS mirror (q=quit)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    ser.close()
    print("종료.")


if __name__ == "__main__":
    main()
