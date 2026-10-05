import os

import pytest

from erp42pro_interface import can_setup as C


class FakeRun:
    """sudo 를 흉내 낸다. allowed: 비밀번호 없이 허용되는가, rc: 켜는 명령의 결과, sysfs: 성공하면 켜진 것으로 바꿀 폴더."""

    def __init__(self, allowed=True, rc=0, stderr='', sysfs=None, channel='can0'):
        self.allowed, self.rc, self.stderr, self.sysfs, self.channel = allowed, rc, stderr, sysfs, channel
        self.calls = []

    def __call__(self, args, **kwargs):
        self.calls.append(list(args))

        class R:
            pass
        r = R()
        r.stderr = ''
        if args[:3] == ['sudo', '-n', '-l']:
            r.returncode = 0 if self.allowed else 1
            return r
        r.returncode, r.stderr = self.rc, self.stderr
        if self.rc == 0 and self.sysfs:
            set_flags(self.sysfs, self.channel, '0x10c1')
        return r


def set_flags(sysfs, channel, flags):
    os.makedirs(os.path.join(str(sysfs), channel), exist_ok=True)
    with open(os.path.join(str(sysfs), channel, 'flags'), 'w') as f:
        f.write(flags + '\n')


def test_command_matches_can_bringup_script():
    # withus_control/can_bringup.sh 와 같은 명령이어야 한다
    assert ' '.join(C.bringup_command('can0')) == 'ip link set can0 up type can bitrate 500000'


def test_link_state(tmp_path):
    assert C.link_state('can0', str(tmp_path)) == 'missing'
    set_flags(tmp_path, 'can0', '0x10c0')      # 꺼짐 (첫 비트 0)
    assert C.link_state('can0', str(tmp_path)) == 'down'
    set_flags(tmp_path, 'can0', '0x10c1')      # 켜짐
    assert C.link_state('can0', str(tmp_path)) == 'up'


def test_already_up_does_nothing(tmp_path):
    set_flags(tmp_path, 'can0', '0x10c1')
    run = FakeRun()
    assert '이미' in C.ensure_can_up('can0', sys_net=str(tmp_path), run=run)
    assert run.calls == []                      # 켜져 있으면 아무 명령도 실행하지 않는다


def test_down_is_brought_up(tmp_path):
    set_flags(tmp_path, 'can0', '0x10c0')
    run = FakeRun(sysfs=tmp_path)
    assert '켰습니다' in C.ensure_can_up('can0', sys_net=str(tmp_path), run=run)
    assert run.calls[-1] == ['sudo', '-n', 'ip', 'link', 'set', 'can0', 'up', 'type', 'can', 'bitrate', '500000']
    assert C.link_state('can0', str(tmp_path)) == 'up'


def test_missing_device(tmp_path):
    run = FakeRun()
    with pytest.raises(C.CanSetupError) as e:
        C.ensure_can_up('can0', sys_net=str(tmp_path), run=run)
    assert '없습니다' in str(e.value)
    assert run.calls == []


def test_no_permission_tells_how_to_fix(tmp_path):
    set_flags(tmp_path, 'can0', '0x10c0')
    run = FakeRun(allowed=False)
    with pytest.raises(C.CanSetupError) as e:
        C.ensure_can_up('can0', sys_net=str(tmp_path), run=run)
    msg = str(e.value)
    assert 'setup_can_sudo.sh' in msg and 'sudo ip link set can0 up type can bitrate 500000' in msg
    assert all(c[:3] == ['sudo', '-n', '-l'] for c in run.calls)     # 허용되지 않으면 켜는 명령은 실행하지 않는다


def test_command_failure_is_reported(tmp_path):
    set_flags(tmp_path, 'can0', '0x10c0')
    run = FakeRun(rc=2, stderr='RTNETLINK answers: Operation not supported')
    with pytest.raises(C.CanSetupError) as e:
        C.ensure_can_up('can0', sys_net=str(tmp_path), run=run)
    assert 'Operation not supported' in str(e.value)


def test_command_ok_but_still_down(tmp_path):
    set_flags(tmp_path, 'can0', '0x10c0')
    with pytest.raises(C.CanSetupError):
        C.ensure_can_up('can0', sys_net=str(tmp_path), run=FakeRun())     # sysfs 를 바꾸지 않는 가짜
