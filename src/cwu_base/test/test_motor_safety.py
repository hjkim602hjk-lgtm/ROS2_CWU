"""Hardware independent protocol and watchdog checks."""
import math
import pytest
from cwu_base.motor_safety import Safety, parse_frame, wheel_ticks


def test_protocol_and_ratio():
    assert parse_frame(b'ENC,42,-2,3,9') == ('ENC', 42, -2, 3, 9)
    for line in (b'log 1 2', b'ENC,1,2', b'ENC,-1,2,3,4',
                 b'ENC,1,2147483648,2,3', b'ACK,0', b'READY,2'):
        with pytest.raises(ValueError):
            parse_frame(line)
    l, r = wheel_ticks(.2, 1., .2, .1, 100., 20.)
    assert r == pytest.approx(20.) and l/r == pytest.approx(1/3)
    with pytest.raises(ValueError):
        wheel_ticks(math.nan, 0., .2, .1, 100., 20.)


def test_fault_discards_command_and_reboot_sample():
    s = Safety(.2, .3, .3)
    s.acknowledged = True
    assert s.encoder(10, 1.)
    s.scan = 1.
    assert s.command(1., 2., 1., True)
    assert s.output(1.1, True) == (1., 2.)
    assert s.output(1.1, False) is None
    assert s.output(1.11, True) is None
    assert s.command(1., 2., 1.12, True)
    assert not s.encoder(10, 1.13)  # replay must not refresh watchdog
    assert not s.encoder(9, 1.14)
    assert s.output(1.4, True) is None
    s.reset()
    assert not s.command(1., 2., 2., True)
    assert s.output(2., True) is None


def test_encoder_wrap_and_individual_watchdogs():
    for fault in ('scan', 'enc', 'cmd'):
        s = Safety(.2, .3, .3)
        s.acknowledged = True
        s.scan = 2.
        assert s.encoder(0xfffffffe, 2.)
        assert s.encoder(5, 2.01)
        assert s.command(2., 3., 2.01, True)
        setattr(s, fault, 0.)
        assert s.output(2.02, True) is None
        setattr(s, fault, 2.02)
        assert s.output(2.03, True) is None


def test_unmeasured_config_cannot_arm():
    import pathlib
    import yaml
    from cwu_base.motor_safety import validate_config
    cfg = yaml.safe_load((pathlib.Path(__file__).parents[1] / 'config/motor.yaml').read_text())['motor_bridge']['ros__parameters']
    with pytest.raises(ValueError):
        validate_config(cfg)
