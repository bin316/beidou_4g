# -*- coding: utf-8 -*-
"""Simulate Solution::start_locate + start_lbs_if_needed for locate_switch modes."""

MASK = 0x07
GNSS, AGNSS, LBS = 0x01, 0x02, 0x04


def normalize(sw: int) -> int:
    return sw & MASK


def start_locate(
    sw: int,
    *,
    agnss_done: bool = False,
    air_ok: bool = True,
) -> list[str]:
    """Mirror Solution.cpp start_locate（仅 GNSS/AGNSS）。"""
    sw = normalize(sw)
    want_gnss = bool(sw & GNSS)
    want_agnss = bool(sw & AGNSS)
    steps: list[str] = []

    if not want_gnss:
        steps.append("SKIP_GNSS")
        return steps

    steps.append("GNSS_ACTIVATE")
    steps.append("GNSS_SETTLE_500ms")
    if want_agnss and air_ok and not agnss_done:
        steps.append("AGNSS_FETCH_INJECT")
    elif want_agnss and agnss_done:
        steps.append("AGNSS_SKIP_already_done")
    elif want_agnss and not air_ok:
        steps.append("AGNSS_SKIP_no_air")
    return steps


def start_lbs_if_needed(
    sw: int,
    *,
    gnss_fixed: bool = False,
) -> list[str]:
    """Mirror Solution.cpp start_lbs_if_needed（仅震动路径调用）。"""
    sw = normalize(sw)
    if not bool(sw & LBS):
        return ["LBS_SKIP_switch_off"]
    if gnss_fixed:
        return ["LBS_SKIP_gnss_fixed"]
    return ["LBS_SESSION_START"]


def locate_switch_is_valid(sw: int) -> bool:
    sw = normalize(sw)
    if bool(sw & AGNSS) and not bool(sw & GNSS):
        return False
    return True


NAMES = {
    0x00: "全关",
    0x01: "仅GNSS",
    0x02: "仅AGNSS",
    0x03: "GNSS+AGNSS",
    0x04: "仅LBS",
    0x05: "GNSS+LBS",
    0x06: "AGNSS+LBS",
    0x07: "GNSS+AGNSS+LBS",
}


def main() -> None:
    print("配置校验 (AGNSS 依赖 GNSS):")
    for sw in range(8):
        valid = locate_switch_is_valid(sw)
        print(f"  0x{sw:02X} {NAMES[sw]:<16} {'ACCEPT' if valid else 'REJECT ok=0 不落盘'}")

    print()
    print("=" * 96)
    print(f"{'sw':<6}{'模式':<16}{'路径':<12}{'执行步骤'}")
    print("=" * 96)

    for sw in range(8):
        for vib, path in [(False, "RTC/开机"), (True, "震动(含引脚)")]:
            if not locate_switch_is_valid(sw):
                print(f"0x{sw:02X}  {NAMES[sw]:<16}{path:<14}{'N/A (config rejected)'}")
                continue
            steps = start_locate(sw)
            if vib:
                steps = steps + start_lbs_if_needed(sw)
            note = ""
            if sw == 0x04 and not vib:
                note = "  # LBS仅震动唤醒"
            print(f"0x{sw:02X}  {NAMES[sw]:<16}{path:<14}{' -> '.join(steps)}{note}")

    print()
    print("=" * 96)
    print("可行性结论")
    print("=" * 96)
    for sw in range(8):
        if not locate_switch_is_valid(sw):
            print(f"  0x{sw:02X} {NAMES[sw]:<16}配置拒绝，不会应用")
            continue
        boot = start_locate(sw)
        vib = start_locate(sw) + start_lbs_if_needed(sw)
        if sw == 0x00:
            v = "正常: GNSS跳过；震动也不启LBS(开关关)"
        elif sw == 0x04:
            v = "正常: RTC无GNSS；震动可启LBS"
        else:
            v = "正常"
        print(f"  0x{sw:02X} {NAMES[sw]:<16}{v}")
        print(f"       RTC/开机: {' -> '.join(boot)}")
        print(f"       震动:     {' -> '.join(vib)}")


if __name__ == "__main__":
    main()
