# 시리얼 줄에서 엔코더 틱을 뽑는 파싱을 하드웨어 없이 검사하는 pytest 파일입니다.
# 밀려 들어온 옛 줄을 버리고 최신 줄만 쓰는지, 깨진 줄을 건너뛰는지 확인합니다.

"""Guard the line parser: newest sample only, malformed lines skipped."""
from cwu_base.encoder_odom import latest_sample


def test_takes_the_newest_line_not_the_backlog():
    # 한 번의 read에 밀린 줄이 여러 개 들어와도 최신 값 하나만 씁니다.
    # 옛 줄까지 처리하면 줄 사이 dt가 0에 가까워져 twist가 발산합니다.
    lines = [b'10,20', b'11,21', b'12,22']
    assert latest_sample(lines, 0, 1) == (12, 22)


def test_skips_lines_without_enough_fields():
    # 잘린 첫 줄과 펌웨어 로그 줄은 건너뛰고 그 앞의 온전한 줄을 씁니다.
    lines = [b'5,6', b'BOOT ok', b'3']
    assert latest_sample(lines, 0, 1) == (5, 6)


def test_negative_ticks_and_field_order():
    assert latest_sample([b'L=-7 R=8'], 0, 1) == (-7, 8)
    assert latest_sample([b'L=-7 R=8'], 1, 0) == (8, -7)


def test_nothing_usable_returns_none():
    assert latest_sample([], 0, 1) is None
    assert latest_sample([b'', b'BOOT'], 0, 1) is None
