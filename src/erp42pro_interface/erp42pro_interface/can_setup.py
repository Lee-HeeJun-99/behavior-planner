"""CAN 인터페이스(can0)를 켜는 도우미.

ROS 에 의존하지 않는 함수만 둔다 (단위 테스트: test/test_can_setup.py).

브릿지는 시작할 때 can0 이 꺼져 있으면 아래 명령으로 직접 켠다.

    sudo -n ip link set can0 up type can bitrate 500000

이 명령은 관리자 권한이 필요하다. 비밀번호를 묻지 않고 실행되도록 PC 마다 한 번 설정해 둔다.

    bash ~/wego_ws/tools/setup_can_sudo.sh
"""

import os
import subprocess

CAN_BITRATE = 500000          # ERP42 Pro CAN 통신 속도 [bps]
SYS_NET = '/sys/class/net'
IFF_UP = 0x1                  # 인터페이스가 켜져 있음을 뜻하는 표시 (flags 의 첫 비트)
SETUP_HINT = 'bash ~/wego_ws/tools/setup_can_sudo.sh'


class CanSetupError(RuntimeError):
    """can0 을 쓸 수 없는 상태. 메시지에 이유와 할 일을 담는다."""


def bringup_command(channel, bitrate=CAN_BITRATE):
    return ['ip', 'link', 'set', str(channel), 'up', 'type', 'can', 'bitrate', str(int(bitrate))]


def link_state(channel, sys_net=SYS_NET):
    """'missing' (장치 없음) / 'down' (꺼짐) / 'up' (켜짐)"""
    base = os.path.join(sys_net, str(channel))
    if not os.path.isdir(base):
        return 'missing'
    try:
        with open(os.path.join(base, 'flags')) as f:
            flags = int(f.read().strip(), 16)
    except (OSError, ValueError):
        return 'down'
    return 'up' if flags & IFF_UP else 'down'


def ensure_can_up(channel, bitrate=CAN_BITRATE, sys_net=SYS_NET, run=subprocess.run):
    """channel 이 켜져 있게 한다. 켜져 있으면 그대로 두고, 꺼져 있으면 켠다.

    반환: 무엇을 했는지 알리는 문장.  켤 수 없으면 CanSetupError.
    """
    cmd = bringup_command(channel, bitrate)
    manual = 'sudo ' + ' '.join(cmd)

    state = link_state(channel, sys_net)
    if state == 'up':
        return '%s 이 이미 켜져 있습니다.' % channel
    if state == 'missing':
        raise CanSetupError(
            '%s 이 없습니다. CAN 장치가 PC 에 연결되어 있는지 확인하세요.  (확인: ip link show %s)' % (channel, channel))

    def call(args):
        return run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=10)

    try:
        # 비밀번호 없이 실행할 수 있는지 먼저 확인한다 (-l: 실행하지 않고 허용 여부만 본다).
        if call(['sudo', '-n', '-l'] + cmd).returncode != 0:
            raise CanSetupError(
                '%s 이 꺼져 있는데, 비밀번호 없이 켤 수 있게 설정되어 있지 않습니다.\n'
                '  한 번만 설정 (이후 자동) : %s\n'
                '  지금만 직접 켜기         : %s\n'
                '둘 중 하나를 한 뒤 브릿지를 다시 켜세요.' % (channel, SETUP_HINT, manual))
        result = call(['sudo', '-n'] + cmd)
    except (OSError, subprocess.TimeoutExpired) as e:
        raise CanSetupError('%s 을 켜는 명령을 실행하지 못했습니다: %s\n  직접 켜기 : %s' % (channel, e, manual))

    if result.returncode != 0:
        raise CanSetupError('%s 을 켜지 못했습니다: %s\n  직접 켜기 : %s'
                            % (channel, (result.stderr or '').strip() or '알 수 없는 오류', manual))
    if link_state(channel, sys_net) != 'up':
        raise CanSetupError('%s 을 켜는 명령은 끝났지만 여전히 꺼져 있습니다.  (확인: ip link show %s)' % (channel, channel))
    return '%s 을 켰습니다 (%d bps).' % (channel, bitrate)
