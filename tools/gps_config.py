#!/usr/bin/env python3
"""
FST-UEF9P (u-blox ZED-F9P) 수신기 설정 스크립트  -  기존 MRP-2000 설정 가이드와 같은 출력으로 맞춤

  python3 gps_config.py            현재 설정만 읽어서 보여줌 (아무것도 바꾸지 않음)
  python3 gps_config.py --apply    설정을 적용하고 수신기 메모리(Flash)에 저장한 뒤 다시 읽어 확인
  python3 gps_config.py --apply --keep-gnss   위성 종류 설정은 건드리지 않음

적용 내용 (USB 포트 기준)
  - 출력 주기 10 Hz
  - NMEA 문장: GGA, RMC, VTG 만 출력 (GLL, GSA, GSV 끔)
  - NMEA High precision 모드 켬 (위경도 소수 7자리)
  - USB 입력: UBX, NMEA, RTCM3 허용 (보정 신호 입력용) / USB 출력: UBX, NMEA
  - 위성: GPS + BeiDou 사용, GLONASS / Galileo 끔  (MRP-2000 가이드 권장값, 10 Hz 확보용)
GPS 드라이버(nmea_serial_driver)를 끈 상태에서 실행할 것.
"""
import struct, sys, time
try:
    import serial
except ImportError:
    print("pyserial 없음: sudo apt install python3-serial"); sys.exit(1)

PORT = '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00'

# (이름, 키 ID, 목표값)
BASE = [
    ('출력 주기 측정간격[ms] (100 = 10 Hz)', 0x30210001, 100),
    ('항법 계산 비율 (1 = 매 측정마다)',      0x30210002, 1),
    ('NMEA High precision 모드',             0x10930006, 1),
    ('USB 출력 GGA',                          0x209100bd, 1),
    ('USB 출력 RMC',                          0x209100ae, 1),
    ('USB 출력 VTG',                          0x209100b3, 1),
    ('USB 출력 GLL',                          0x209100cc, 0),
    ('USB 출력 GSA',                          0x209100c2, 0),
    ('USB 출력 GSV',                          0x209100c7, 0),
    ('USB 입력 UBX 허용',                     0x10770001, 1),
    ('USB 입력 NMEA 허용',                    0x10770002, 1),
    ('USB 입력 RTCM3 허용',                   0x10770004, 1),
    ('USB 출력 UBX 허용',                     0x10780001, 1),
    ('USB 출력 NMEA 허용',                    0x10780002, 1),
]
GNSS = [
    ('위성 GPS 사용',                         0x1031001f, 1),
    ('위성 BeiDou 사용',                      0x10310022, 1),
    ('위성 Galileo 사용',                     0x10310021, 0),
    ('위성 GLONASS 사용',                     0x10310025, 0),
]
SIZE = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8}
FMT = {1: '<B', 2: '<H', 4: '<I', 8: '<Q'}


def ksize(key):
    return SIZE[(key >> 28) & 0x7]


def ubx(cls, mid, payload=b''):
    body = bytes([cls, mid]) + struct.pack('<H', len(payload)) + payload
    a = b = 0
    for x in body:
        a = (a + x) & 255; b = (b + a) & 255
    return b'\xb5\x62' + body + bytes([a, b])


def frames(buf):
    i = 0; out = []
    while True:
        i = buf.find(b'\xb5\x62', i)
        if i < 0 or i + 8 > len(buf):
            break
        ln = struct.unpack('<H', buf[i+4:i+6])[0]
        if i + 8 + ln > len(buf):
            i += 2; continue
        a = b = 0
        for x in buf[i+2:i+6+ln]:
            a = (a + x) & 255; b = (b + a) & 255
        if (a, b) == (buf[i+6+ln], buf[i+7+ln]):
            out.append((buf[i+2], buf[i+3], buf[i+6:i+6+ln])); i += 8 + ln
        else:
            i += 2
    return out


def read_for(s, sec):
    t = time.time(); buf = b''
    while time.time() - t < sec:
        buf += s.read(4096)
    return buf


def valget(s, keys, layer=0):
    """layer: 0 = RAM(현재 동작값), 1 = BBR, 2 = Flash(저장값)"""
    payload = bytes([0, layer, 0, 0]) + b''.join(struct.pack('<I', k) for k in keys)
    s.reset_input_buffer(); s.write(ubx(0x06, 0x8B, payload))
    res = {}; buf = read_for(s, 1.0)
    for c, m, p in frames(buf):
        if (c, m) == (0x06, 0x8B) and len(p) >= 4:
            i = 4
            while i + 4 <= len(p):
                k = struct.unpack('<I', p[i:i+4])[0]; n = ksize(k)
                if i + 4 + n > len(p):
                    break
                res[k] = struct.unpack(FMT[n], p[i+4:i+4+n])[0]; i += 4 + n
    return res


def valget_safe(s, keys, layer=0):
    """묶음 조회가 거부되면(모르는 키가 섞이면 전체가 거부됨) 키를 하나씩 조회"""
    res = valget(s, keys, layer)
    if len(res) == len(keys):
        return res
    for k in keys:
        if k not in res:
            res.update(valget(s, [k], layer))
    return res


def valset(s, items, layers=0x07):
    """layers: 1 = RAM, 2 = BBR, 4 = Flash  (7 = 전부 -> 전원을 꺼도 유지)"""
    payload = bytes([0, layers, 0, 0])
    for _, k, v in items:
        payload += struct.pack('<I', k) + struct.pack(FMT[ksize(k)], v)
    s.reset_input_buffer(); s.write(ubx(0x06, 0x8A, payload))
    buf = read_for(s, 2.0)
    for c, m, p in frames(buf):
        if c == 0x05 and len(p) >= 2 and p[0] == 0x06 and p[1] == 0x8A:
            return m == 0x01      # 0x01 = ACK, 0x00 = NAK
    return None


def show(title, items, ram, flash=None):
    print(title)
    ok_all = True
    for name, k, want in items:
        r = ram.get(k); f = None if flash is None else flash.get(k)
        ok = (r == want) and (flash is None or f == want)
        ok_all &= ok
        fs = '' if flash is None else ' | 저장값 %-4s' % ('?' if f is None else f)
        print('  %s %-36s 현재값 %-4s%s | 목표 %s' % ('✅' if ok else '❌', name, '?' if r is None else r, fs, want))
    return ok_all


def measure(s, sec=4.0):
    buf = read_for(s, sec); cnt = {}
    sample = {}
    for line in buf.decode('latin-1', 'ignore').splitlines():
        if line.startswith('$') and len(line) > 6 and '*' in line:
            t = line[3:6]; cnt[t] = cnt.get(t, 0) + 1; sample[t] = line
    print('\n실제 USB 출력 (%.0f초 측정)' % sec)
    for t in sorted(cnt):
        print('  %s: %.1f Hz   예) %s' % (t, cnt[t] / sec, sample[t][:95]))
    if not cnt:
        print('  NMEA 문장이 나오지 않음')
    return cnt


def main():
    apply_ = '--apply' in sys.argv
    items = BASE + ([] if '--keep-gnss' in sys.argv else GNSS)
    keys = [k for _, k, _ in BASE + GNSS]
    s = serial.Serial(PORT, 115200, timeout=0.2)
    time.sleep(0.3)
    ram = valget_safe(s, keys, 0)
    if not ram:
        print('수신기가 응답하지 않습니다. (다른 프로그램이 포트를 쓰고 있거나 USB 의 UBX 입출력이 꺼져 있음)'); sys.exit(1)
    show('=== 현재 설정 ===', BASE + GNSS, ram)
    if not apply_:
        measure(s)
        print('\n(읽기만 했습니다. 설정을 바꾸려면 --apply 를 붙여 실행하세요)')
        return
    print('\n=== 설정 적용 (RAM + BBR + Flash) ===')
    txt = {True: 'ACK (수락)', False: 'NAK (거부)', None: '응답 없음'}
    r1 = valset(s, BASE)
    print('  주기 / NMEA / USB 설정 :', txt[r1])
    if r1 is not True:      # 묶음이 거부되면 항목별로 다시 시도해서 어느 항목이 문제인지 표시
        for it in BASE:
            print('    - %-36s %s' % (it[0], txt[valset(s, [it])]))
    if '--keep-gnss' not in sys.argv:
        r2 = valset(s, GNSS)
        print('  위성 종류 설정         :', txt[r2])
        if r2 is not True:
            for it in GNSS:
                print('    - %-36s %s' % (it[0], txt[valset(s, [it])]))
        time.sleep(2.0)   # 위성 설정 변경 시 수신기 GNSS 부가 재시작됨
    ram = valget_safe(s, keys, 0); flash = valget_safe(s, keys, 2)
    ok = show('\n=== 적용 후 확인 (현재값 = 동작 중인 값, 저장값 = 전원을 꺼도 남는 값) ===', items, ram, flash)
    cnt = measure(s)
    rate_ok = all(abs(cnt.get(t, 0) / 4.0 - 10.0) < 1.5 for t in ('GGA', 'RMC', 'VTG')) and not any(t in cnt for t in ('GLL', 'GSA', 'GSV'))
    print('\n==================== 결과:', '✅ 설정 완료' if (ok and rate_ok) else '❌ 확인 필요', '====================')


if __name__ == '__main__':
    main()
