#!/usr/bin/env python3
"""RuntimeProbe host-side soak tester.

Runs repeated cycles of:
1) GPIO waveform exercise
2) UART flood exercise
3) I2C scan exercise
4) Mode switching back to off
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Tuple

try:
    import serial
except ImportError as exc:  # pragma: no cover - environment dependent
    print("pyserial is required. Install with: pip install pyserial", file=sys.stderr)
    raise SystemExit(2) from exc


class RuntimeProbeSoak:
    def __init__(self, ser: serial.Serial):
        self.ser = ser

    def _read_available_lines(self) -> list[str]:
        lines: list[str] = []
        while True:
            raw = self.ser.readline()
            if not raw:
                break
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            print(f"< {line}")
            lines.append(line)
        return lines

    def pump(self, duration_s: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_s)
        while time.monotonic() < deadline:
            self._read_available_lines()
            time.sleep(0.01)

    def send_command(
        self,
        command: str,
        timeout_s: float = 2.0,
        allow_error: bool = False,
    ) -> Tuple[bool, str]:
        self._read_available_lines()
        print(f"> {command}")
        payload = (command + "\n").encode("ascii", errors="strict")
        self.ser.write(payload)
        self.ser.flush()

        deadline = time.monotonic() + max(0.1, timeout_s)
        last_error = ""
        while time.monotonic() < deadline:
            for line in self._read_available_lines():
                if line == "OK":
                    return True, ""
                if line.startswith("ERR:"):
                    last_error = line
                    if allow_error:
                        return False, line
                    return False, line
            time.sleep(0.01)

        if last_error:
            return False, last_error
        return False, "timeout waiting for OK/ERR"

    def wait_for_text(self, text: str, timeout_s: float) -> bool:
        deadline = time.monotonic() + max(0.1, timeout_s)
        while time.monotonic() < deadline:
            for line in self._read_available_lines():
                if text in line:
                    return True
            time.sleep(0.01)
        return False

    def run_cycle(self, cycle: int, args: argparse.Namespace) -> int:
        failures = 0

        def run(
            command: str,
            timeout_s: float = 2.0,
            required: bool = True,
            allow_error: bool = False,
        ) -> bool:
            nonlocal failures
            ok, reason = self.send_command(command, timeout_s=timeout_s, allow_error=allow_error)
            if not ok and required:
                failures += 1
                print(f"! cycle {cycle}: command failed: {command} ({reason})")
            return ok

        print(f"\n=== Cycle {cycle}/{args.cycles} ===")
        run("mode gpio")
        run(f"gpio set {args.gpio_pin}")
        run(f"gpio toggle {args.gpio_toggle_hz}")
        self.pump(args.gpio_toggle_seconds)
        run(f"gpio pulse {args.gpio_pulse_width_us} {args.gpio_pulse_period_us}")
        self.pump(args.gpio_pulse_seconds)
        run("gpio stop")

        run("mode uart")
        if args.uart_num == 0:
            run("uart force 1")

        if args.uart_tx_pin is not None and args.uart_rx_pin is not None:
            uart_start = f"uart start {args.uart_num} {args.uart_tx_pin} {args.uart_rx_pin} {args.uart_baud}"
        else:
            uart_start = f"uart start {args.uart_num} default {args.uart_baud}"

        uart_started = run(
            uart_start,
            required=not args.allow_missing_defaults,
        )
        if uart_started:
            run("uart pattern ramp")
            run(f"uart txrate {args.uart_tx_rate_bps}")
            run("uart echo 0")
            self.pump(args.uart_flood_seconds)
            run("uart stop")
        else:
            run("uart stop", required=False, allow_error=True)

        run("mode i2c")
        if args.i2c_sda_pin is not None and args.i2c_scl_pin is not None:
            i2c_start = (
                f"i2c start {args.i2c_sda_pin} {args.i2c_scl_pin} "
                f"{args.i2c_freq_hz} {args.i2c_timeout_ms}"
            )
        else:
            i2c_start = f"i2c start default {args.i2c_freq_hz} {args.i2c_timeout_ms}"

        i2c_started = run(
            i2c_start,
            required=not args.allow_missing_defaults,
        )
        if i2c_started:
            run(f"i2c scan 0x{args.i2c_scan_start:02X} 0x{args.i2c_scan_end:02X}")
            if not self.wait_for_text("I2C scan done", args.i2c_scan_timeout_s):
                failures += 1
                print(f"! cycle {cycle}: i2c scan completion timeout")
            run("i2c stop")
        else:
            run("i2c stop", required=False, allow_error=True)

        run("mode off")
        run("status", required=False)
        self.pump(args.cycle_delay_s)
        return failures


def positive_int(value: str) -> int:
    parsed = int(value, 0)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be > 0")
    return parsed


def nonnegative_int(value: str) -> int:
    parsed = int(value, 0)
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be >= 0")
    return parsed


def byte_value(value: str) -> int:
    parsed = int(value, 0)
    if parsed < 0 or parsed > 0x7F:
        raise argparse.ArgumentTypeError("must be in range 0x00..0x7F")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run RuntimeProbe soak cycles over serial.")
    parser.add_argument("--port", required=True, help="Serial port (for example COM7)")
    parser.add_argument("--baud", type=positive_int, default=2_000_000, help="Host serial baud")
    parser.add_argument("--cycles", type=positive_int, default=50, help="Number of soak cycles")
    parser.add_argument("--strict", action="store_true", help="Exit non-zero if any cycle has failures")
    parser.add_argument(
        "--allow-missing-defaults",
        action="store_true",
        help="Do not fail cycle when uart/i2c default-pin startup is unsupported",
    )
    parser.add_argument("--startup-delay-s", type=float, default=1.5, help="Delay after opening port")
    parser.add_argument("--cycle-delay-s", type=float, default=0.15, help="Idle delay between cycles")

    parser.add_argument("--gpio-pin", type=nonnegative_int, default=2, help="GPIO pin for waveform tests")
    parser.add_argument("--gpio-toggle-hz", type=positive_int, default=5000, help="GPIO toggle frequency")
    parser.add_argument("--gpio-toggle-seconds", type=float, default=0.35, help="GPIO toggle dwell")
    parser.add_argument("--gpio-pulse-width-us", type=positive_int, default=10, help="GPIO pulse width")
    parser.add_argument("--gpio-pulse-period-us", type=positive_int, default=100, help="GPIO pulse period")
    parser.add_argument("--gpio-pulse-seconds", type=float, default=0.35, help="GPIO pulse dwell")

    parser.add_argument("--uart-num", type=nonnegative_int, default=1, help="UART number to start")
    parser.add_argument("--uart-tx-pin", type=nonnegative_int, default=None, help="Optional explicit UART TX pin")
    parser.add_argument("--uart-rx-pin", type=nonnegative_int, default=None, help="Optional explicit UART RX pin")
    parser.add_argument("--uart-baud", type=positive_int, default=921600, help="UART line baud")
    parser.add_argument("--uart-tx-rate-bps", type=positive_int, default=200000, help="RuntimeProbe txrate")
    parser.add_argument("--uart-flood-seconds", type=float, default=1.0, help="UART flood dwell")

    parser.add_argument("--i2c-sda-pin", type=nonnegative_int, default=None, help="Optional explicit I2C SDA pin")
    parser.add_argument("--i2c-scl-pin", type=nonnegative_int, default=None, help="Optional explicit I2C SCL pin")
    parser.add_argument("--i2c-freq-hz", type=positive_int, default=400000, help="I2C frequency")
    parser.add_argument("--i2c-timeout-ms", type=positive_int, default=20, help="I2C timeout")
    parser.add_argument("--i2c-scan-start", type=byte_value, default=0x03, help="I2C scan start address")
    parser.add_argument("--i2c-scan-end", type=byte_value, default=0x77, help="I2C scan end address")
    parser.add_argument("--i2c-scan-timeout-s", type=float, default=10.0, help="Timeout for scan completion")

    args = parser.parse_args(argv)

    if args.gpio_pulse_width_us >= args.gpio_pulse_period_us:
        parser.error("--gpio-pulse-width-us must be < --gpio-pulse-period-us")
    if args.i2c_scan_start > args.i2c_scan_end:
        parser.error("--i2c-scan-start must be <= --i2c-scan-end")
    if (args.uart_tx_pin is None) != (args.uart_rx_pin is None):
        parser.error("--uart-tx-pin and --uart-rx-pin must be provided together")
    if (args.i2c_sda_pin is None) != (args.i2c_scl_pin is None):
        parser.error("--i2c-sda-pin and --i2c-scl-pin must be provided together")
    return args


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    print(f"Opening {args.port} @ {args.baud}...")
    with serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=0.5) as ser:
        ser.dtr = False
        ser.rts = False
        time.sleep(max(0.0, args.startup_delay_s))

        soak = RuntimeProbeSoak(ser)
        soak.pump(0.5)
        total_failures = 0

        for cycle in range(1, args.cycles + 1):
            total_failures += soak.run_cycle(cycle, args)

        soak.send_command("mode off", timeout_s=1.5, allow_error=True)
        soak.pump(0.2)
        print(f"\nCycles completed: {args.cycles}")
        print(f"Failures: {total_failures}")
        if total_failures and args.strict:
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
