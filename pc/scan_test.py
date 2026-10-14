"""
블루투스 스캔 테스트 — 노트북이 BLE 기기를 잡는지 확인용.
실행: python scan_test.py
주변 BLE 기기 목록을 출력함. HELIOS-MIRROR 가 목록에 뜨는지 확인.
"""
import asyncio
from bleak import BleakScanner

try:
    from bleak.backends.winrt.util import allow_sta
    allow_sta()
except Exception:
    pass


async def main():
    print("10초간 주변 BLE 기기 스캔 중...\n")
    devices = await BleakScanner.discover(timeout=10.0)
    if not devices:
        print("!! 아무 기기도 못 찾음 → 노트북 블루투스가 꺼졌거나 BLE 미지원 가능성")
        return
    print(f"찾은 기기 {len(devices)}개:")
    found = False
    for d in devices:
        name = d.name or "(이름 없음)"
        print(f"  - {name}   [{d.address}]")
        if d.name and "HELIOS" in d.name:
            found = True
    print()
    if found:
        print(">> HELIOS 기기 발견! 노트북 블루투스 정상. mirror.py 실행 가능.")
    else:
        print(">> HELIOS 는 목록에 없음. (로봇이 광고 중인지 / 다른 기기가 연결 중인지 확인)")


if __name__ == "__main__":
    asyncio.run(main())
