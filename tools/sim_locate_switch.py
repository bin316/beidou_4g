# -*- coding: utf-8 -*-
"""Simulate Solution::start_locate for all locate_switch modes."""

MASK = 0x07
GNSS, AGNSS, LBS = 0x01, 0x02, 0x04


def normalize(sw: int) -> int:
    return sw & MASK


def start_locate(
    sw: int,
    allow_lbs: bool,
    *,
    gnss_fixed: bool = False,
    agnss_done: bool = False,
    agnss_rx_active: bool = False,
    air_ok: bool = True,
) -> list[str]:
    """Mirror Solution.cpp start_locate branching."""
    sw = normalize(sw)
    want_gnss = bool(sw & GNSS)
    want_agnss = bool(sw & AGNSS)
    want_lbs = allow_lbs and bool(sw & LBS)
    steps: list[str] = []

    if not want_gnss and not want_lbs:
        steps.append("SKIP")
        return steps

    if want_gnss:
        steps.append("GNSS_ACTIVATE")
        steps.append("GNSS_SETTLE_500ms")
        if want_agnss and air_ok and not agnss_done:
            steps.append("AGNSS_FETCH_INJECT")
        elif want_agnss and agnss_done:
            steps.append("AGNSS_SKIP_already_done")
        elif want_agnss and not air_ok:
            steps.append("AGNSS_SKIP_no_air")

    if want_lbs and not agnss_rx_active and (not want_gnss or not gnss_fixed):
        if air_ok:
            steps.append("LBS_QUERY")
            steps.append("LBS_APPLY_GEO_IF_OK")
        else:
            steps.append("LBS_SKIP_no_air")
    elif want_lbs and agnss_rx_active:
        steps.append("LBS_SKIP_agnss_rx")
    elif want_lbs and want_gnss and gnss_fixed:
        steps.append("LBS_SKIP_gnss_fixed")

    return steps


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
    print(f"{'sw':<6}{'模式':<16}{'路径':<10}{'执行步骤'}")
    print("=" * 96)

    issues: list[str] = []

    for sw in range(8):
        for allow, path in [(False, "开机/定时"), (True, "震动")]:
            # 非法配置不会写入；模拟运行时按合法开关
            if not locate_switch_is_valid(sw):
                print(f"0x{sw:02X}  {NAMES[sw]:<16}{path:<10}{'N/A (config rejected)':<55}")
                continue
            steps = start_locate(sw, allow)
            note = ""
            if sw == 0x04 and not allow and steps == ["SKIP"]:
                note = "设计: 定时路径不允许LBS"
            print(f"0x{sw:02X}  {NAMES[sw]:<16}{path:<10}{' -> '.join(steps):<55}{note}")

    print()
    print("=" * 96)
    print("可行性结论")
    print("=" * 96)
    for sw in range(8):
        if not locate_switch_is_valid(sw):
            print(f"  0x{sw:02X} {NAMES[sw]:<16}配置拒绝，不会应用")
            continue
        boot = start_locate(sw, False)
        vib = start_locate(sw, True)
        if sw == 0x00:
            v = "正常: 明确跳过"
        elif sw == 0x04:
            v = "正常: 仅震动做LBS；开机跳过"
        else:
            v = "正常"
        print(f"  0x{sw:02X} {NAMES[sw]:<16}{v}")
        print(f"       开机: {' -> '.join(boot)}")
        print(f"       震动: {' -> '.join(vib)}")


if __name__ == "__main__":
    main()
