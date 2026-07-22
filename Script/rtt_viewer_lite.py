# python3.11
# -*- coding: utf-8 -*-

"""
脚本名称: rtt_viewer_lite.py
功能描述: 基于 OpenOCD 的 RTT 日志预览（beidou_4g / STM32L432）。
前提: 已将 CubeIDE OpenOCD 的 .../tools/bin 加入 PATH（where openocd 可用）。
用法:  cd Script && python rtt_viewer_lite.py
"""

import glob
import os
import shutil
import signal
import socket
import subprocess
import sys
import time

# =============================================================================
# --- 配置区域 ---
# =============================================================================

OOCD_HOST = "127.0.0.1"
OOCD_TELNET_PORT = 4444
RTT_DATA_PORT = 12345
RTT_CTRL_ADDR = "0x20000000"  # RAM 扫描起点（勿用固定 _SEGGER_RTT 地址）
RTT_BUFF_SIZE = "0x40000"      # 扫描范围；勿再二次 rtt setup，否则会覆盖本范围
RTT_CHANNEL = 0

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_SCRIPT_DIR, ".."))
OOCD_CONFIG_FILE = os.path.join(_REPO_ROOT, "daplink-openocd.cfg")

# =============================================================================

class Color:
    YELLOW = '\033[1;33m'
    GREEN = '\033[0;32m'
    RED = '\033[0;31m'
    BLUE = '\033[0;34m'
    NC = '\033[0m'


def log_info(msg):
    print(f"{Color.GREEN}[Info] {msg}{Color.NC}")


def log_warn(msg):
    print(f"{Color.YELLOW}[Warn] {msg}{Color.NC}")


def log_error(msg):
    print(f"{Color.RED}[Error] {msg}{Color.NC}")


def log_debug(msg):
    print(f"{Color.BLUE}[Debug] {msg}{Color.NC}")


def check_port_open(host, port, timeout=1):
    """检查 TCP 端口是否开放"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((host, port))
        if result == 0:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        sock.close()
        return result == 0
    except OSError:
        return False


def find_openocd_exe():
    """从 PATH 解析 openocd"""
    return shutil.which("openocd") or shutil.which("openocd.exe")


def find_st_scripts(exe):
    """
    从 CubeIDE openocd.exe 推断 st_scripts：
    .../plugins/<externaltools.openocd.*>/tools/bin/openocd.exe
      → .../plugins/<debug.openocd.*>/resources/openocd/st_scripts
    """
    plugins_dir = None
    p = os.path.abspath(exe)
    for _ in range(6):
        p = os.path.dirname(p)
        if not p or p == os.path.dirname(p):
            break
        if os.path.basename(p).lower() == "plugins":
            plugins_dir = p
            break
    if not plugins_dir:
        return None
    matches = glob.glob(os.path.join(
        plugins_dir,
        "com.st.stm32cube.ide.mcu.debug.openocd_*",
        "resources", "openocd", "st_scripts",
    ))
    matches = [m for m in matches if os.path.isdir(m)]
    if not matches:
        return None
    return max(matches, key=os.path.getmtime)


def start_openocd():
    """启动 OpenOCD（必须 -s st_scripts；STM 版用 tcl_port 下划线语法）"""
    log_info("正在启动 OpenOCD...")

    if not os.path.isfile(OOCD_CONFIG_FILE):
        log_error(f"找不到配置文件: {OOCD_CONFIG_FILE}")
        return None

    exe = find_openocd_exe()
    if not exe:
        log_error("PATH 中找不到 openocd，请先把 CubeIDE .../tools/bin 加入 PATH")
        return None

    scripts = find_st_scripts(exe)
    if not scripts:
        log_error("找不到 st_scripts（需 CubeIDE debug.openocd 插件）")
        return None

    cmd = [
        exe,
        "-s", scripts,
        "-f", OOCD_CONFIG_FILE,
        "-c", "tcl_port disabled",
        "-c", "gdb_port disabled",
        "-c", f"telnet_port {OOCD_TELNET_PORT}",
    ]
    log_debug(f"启动命令: {' '.join(cmd)}")

    try:
        log_path = os.path.join(_SCRIPT_DIR, "openocd_rtt.log")
        log_fp = open(log_path, "w", encoding="utf-8", errors="replace")
        kwargs = {
            "stdout": log_fp,
            "stderr": subprocess.STDOUT,
        }
        if os.name == "nt":
            kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            kwargs["start_new_session"] = True
        process = subprocess.Popen(cmd, **kwargs)
        process._rtt_log_fp = log_fp  # noqa: SLF001
        process._rtt_log_path = log_path
        log_info(f"OpenOCD 进程已启动 (PID: {process.pid})，日志: {log_path}")
        return process
    except OSError as e:
        log_error(f"启动 OpenOCD 失败: {e}")
        return None


def send_telnet_command(host, port, commands):
    """通过 Telnet 端口发送 OpenOCD 命令"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3)
        sock.connect((host, port))
        time.sleep(0.1)

        for cmd in commands:
            sock.sendall((cmd + "\n").encode("utf-8"))
            time.sleep(2.0 if cmd == "reset run" else 0.2)

        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        sock.close()
        return True
    except OSError as e:
        log_error(f"Telnet 连接失败: {e}")
        return False


def stream_rtt_data(host, port, openocd_process=None):
    """连接 RTT 数据端口并流式输出"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)
        sock.connect((host, port))
        log_info("RTT 数据连接成功")
        print(f"{Color.GREEN}----------------------------------------{Color.NC}")
        print(f"{Color.GREEN}  RTT Log Viewer (Ctrl+C Exit.){Color.NC}")
        print(f"{Color.GREEN}----------------------------------------{Color.NC}")

        while True:
            try:
                data = sock.recv(4096)
                if not data:
                    log_warn("连接已断开")
                    break
                sys.stdout.write(data.decode("utf-8", errors="replace"))
                sys.stdout.flush()
            except socket.timeout:
                continue
            except KeyboardInterrupt:
                break

        sock.close()

        if openocd_process:
            log_info("正在停止 OpenOCD...")
            openocd_process.terminate()
            try:
                openocd_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                openocd_process.kill()
            fp = getattr(openocd_process, "_rtt_log_fp", None)
            if fp:
                try:
                    fp.close()
                except OSError:
                    pass
            log_info("OpenOCD 已停止")

    except OSError as e:
        log_error(f"RTT 数据连接失败: {e}")
        return False
    return True


def main():
    openocd_process = None

    def signal_handler(sig, frame):
        print(f"\n{Color.YELLOW}[退出] 断开连接{Color.NC}")
        if openocd_process:
            openocd_process.terminate()
            try:
                openocd_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                openocd_process.kill()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print(f"{Color.YELLOW}[0/3] 检测 OpenOCD 服务状态...{Color.NC}")

    if check_port_open(OOCD_HOST, OOCD_TELNET_PORT):
        log_info("检测到 OpenOCD 已运行（复用 4444）")
    else:
        openocd_process = start_openocd()
        if not openocd_process:
            sys.exit(1)

        log_info(f"等待 OpenOCD 启动 (监听 {OOCD_TELNET_PORT})...")
        ready = False
        for _ in range(30):
            if openocd_process.poll() is not None:
                log_error(f"OpenOCD 已退出 (code={openocd_process.returncode})")
                log_warn(f"详见 {getattr(openocd_process, '_rtt_log_path', 'openocd_rtt.log')}")
                sys.exit(1)
            if check_port_open(OOCD_HOST, OOCD_TELNET_PORT):
                ready = True
                log_info("OpenOCD 已就绪")
                break
            time.sleep(1)
        if not ready:
            log_error("OpenOCD 启动超时 (30 秒)")
            log_warn("请检查 ST-Link、是否被 IDE 占用、板子供电；并看 openocd_rtt.log")
            openocd_process.terminate()
            sys.exit(1)

    print(f"{Color.YELLOW}[1/3] 配置 OpenOCD RTT 服务...{Color.NC}")

    # 与 Slope_Weather_Monitor 一致：只 setup 一次。再次 rtt setup 会覆盖搜索区
    # （曾误加 SRAM2 扫描，导致只搜 0x10000000，找不到主 RAM 的 _SEGGER_RTT）
    commands = [
        "reset run",
        f"rtt setup {RTT_CTRL_ADDR} {RTT_BUFF_SIZE} {{SEGGER RTT}}",
        "rtt start",
        "rtt start",
        f"rtt server start {RTT_DATA_PORT} {RTT_CHANNEL}",
    ]
    if not send_telnet_command(OOCD_HOST, OOCD_TELNET_PORT, commands):
        log_error(f"无法连接 Telnet 端口 {OOCD_TELNET_PORT}")
        sys.exit(1)

    log_info("RTT 配置命令已发送")

    print(f"{Color.YELLOW}[2/3] 正在连接 RTT 数据流...{Color.NC}")

    if not stream_rtt_data(OOCD_HOST, RTT_DATA_PORT, openocd_process):
        log_error(f"无法连接 RTT 数据端口 {RTT_DATA_PORT}")
        sys.exit(1)


if __name__ == "__main__":
    main()
