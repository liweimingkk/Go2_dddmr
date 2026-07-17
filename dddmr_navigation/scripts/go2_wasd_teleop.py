#!/usr/bin/env python3
"""Supervised terminal teleoperation for a Unitree Go2 Sport service.

The default mode is a no-motion preview.  Live mode is deliberately gated and
publishes only Sport Move (1008) and StopMove (1003) requests.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import select
import signal
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Mapping, Optional, Sequence, TextIO, Tuple


MOVE_API_ID = 1008
STOP_MOVE_API_ID = 1003
SPORT_REQUEST_TOPIC = "/api/sport/request"
LIVE_CONFIRM_ENV = "GO2_WASD_LIVE_CONFIRM"
LIVE_CONFIRM_PHRASE = "I_AM_SUPERVISING_GO2_WASD"

DEFAULT_LINEAR_SPEED = 0.10
DEFAULT_ANGULAR_SPEED = 0.25
DEFAULT_COMMAND_TIMEOUT = 0.35
DEFAULT_PUBLISH_RATE = 20.0
DEFAULT_STOP_KEEPALIVE_RATE = 2.0

MAX_LINEAR_SPEED = 0.30
MAX_ANGULAR_SPEED = 0.35
MIN_COMMAND_TIMEOUT = 0.10
MAX_COMMAND_TIMEOUT = 1.00
MIN_PUBLISH_RATE = 5.0
MAX_PUBLISH_RATE = 50.0


@dataclass(frozen=True)
class MotionCommand:
    x: float
    y: float
    yaw: float

    def parameter(self) -> str:
        return json.dumps(
            {"x": self.x, "y": self.y, "z": self.yaw},
            separators=(",", ":"),
        )


@dataclass(frozen=True)
class KeyAction:
    kind: str
    label: str
    command: Optional[MotionCommand] = None


class ActiveMotion:
    """A command that expires unless keyboard repeat refreshes it."""

    def __init__(self) -> None:
        self.command: Optional[MotionCommand] = None
        self.deadline: Optional[float] = None

    def activate(self, command: MotionCommand, now: float, timeout: float) -> None:
        self.command = command
        self.deadline = now + timeout

    def stop(self) -> None:
        self.command = None
        self.deadline = None

    def current(self, now: float) -> Optional[MotionCommand]:
        if self.command is None or self.deadline is None:
            return None
        if now >= self.deadline:
            self.stop()
            return None
        return self.command


def action_for_key(
    key: str, linear_speed: float, angular_speed: float
) -> Optional[KeyAction]:
    normalized = key.lower()
    if normalized == "w":
        return KeyAction("move", "forward", MotionCommand(linear_speed, 0.0, 0.0))
    if normalized == "s":
        return KeyAction("move", "backward", MotionCommand(-linear_speed, 0.0, 0.0))
    if normalized == "a":
        return KeyAction("move", "turn left", MotionCommand(0.0, 0.0, angular_speed))
    if normalized == "d":
        return KeyAction("move", "turn right", MotionCommand(0.0, 0.0, -angular_speed))
    if key == " " or normalized == "x":
        return KeyAction("stop", "stop")
    if normalized == "q" or key in ("\x03", "\x1b"):
        return KeyAction("quit", "stop and quit")
    return None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "WASD teleoperation for Go2. Default: preview only; --live can move "
            "the physical robot."
        )
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help=(
            "publish to /api/sport/request; also requires "
            f"{LIVE_CONFIRM_ENV}={LIVE_CONFIRM_PHRASE}"
        ),
    )
    parser.add_argument(
        "--linear-speed",
        type=float,
        default=DEFAULT_LINEAR_SPEED,
        help=f"W/S speed in m/s (default: {DEFAULT_LINEAR_SPEED:.2f})",
    )
    parser.add_argument(
        "--angular-speed",
        type=float,
        default=DEFAULT_ANGULAR_SPEED,
        help=f"A/D yaw speed in rad/s (default: {DEFAULT_ANGULAR_SPEED:.2f})",
    )
    parser.add_argument(
        "--command-timeout",
        type=float,
        default=DEFAULT_COMMAND_TIMEOUT,
        help=(
            "seconds before an unrefreshed direction key triggers StopMove "
            f"(default: {DEFAULT_COMMAND_TIMEOUT:.2f})"
        ),
    )
    parser.add_argument(
        "--publish-rate",
        type=float,
        default=DEFAULT_PUBLISH_RATE,
        help=f"active Move publish rate in Hz (default: {DEFAULT_PUBLISH_RATE:.1f})",
    )
    parser.add_argument(
        "--stop-keepalive-rate",
        type=float,
        default=DEFAULT_STOP_KEEPALIVE_RATE,
        help=(
            "idle StopMove keepalive rate in Hz; 0 disables keepalive "
            f"(default: {DEFAULT_STOP_KEEPALIVE_RATE:.1f})"
        ),
    )
    parser.add_argument(
        "--discovery-timeout",
        type=float,
        default=5.0,
        help="seconds to wait for the Go2 Sport request subscriber in live mode",
    )
    return parser


def validate_options(
    args: argparse.Namespace, environ: Mapping[str, str]
) -> None:
    finite_values = {
        "linear speed": args.linear_speed,
        "angular speed": args.angular_speed,
        "command timeout": args.command_timeout,
        "publish rate": args.publish_rate,
        "stop keepalive rate": args.stop_keepalive_rate,
        "discovery timeout": args.discovery_timeout,
    }
    for label, value in finite_values.items():
        if not math.isfinite(value):
            raise ValueError(f"{label} must be finite")

    if not 0.0 < args.linear_speed <= MAX_LINEAR_SPEED:
        raise ValueError(
            f"linear speed must be in (0, {MAX_LINEAR_SPEED:.2f}] m/s"
        )
    if not 0.0 < args.angular_speed <= MAX_ANGULAR_SPEED:
        raise ValueError(
            f"angular speed must be in (0, {MAX_ANGULAR_SPEED:.2f}] rad/s"
        )
    if not MIN_COMMAND_TIMEOUT <= args.command_timeout <= MAX_COMMAND_TIMEOUT:
        raise ValueError(
            "command timeout must be in "
            f"[{MIN_COMMAND_TIMEOUT:.2f}, {MAX_COMMAND_TIMEOUT:.2f}] seconds"
        )
    if not MIN_PUBLISH_RATE <= args.publish_rate <= MAX_PUBLISH_RATE:
        raise ValueError(
            f"publish rate must be in [{MIN_PUBLISH_RATE:.1f}, "
            f"{MAX_PUBLISH_RATE:.1f}] Hz"
        )
    if not 0.0 <= args.stop_keepalive_rate <= 10.0:
        raise ValueError("stop keepalive rate must be in [0, 10] Hz")
    if not 0.1 <= args.discovery_timeout <= 30.0:
        raise ValueError("discovery timeout must be in [0.1, 30] seconds")

    if not args.live:
        return
    if environ.get(LIVE_CONFIRM_ENV) != LIVE_CONFIRM_PHRASE:
        raise ValueError(
            "live mode requires "
            f"{LIVE_CONFIRM_ENV}={LIVE_CONFIRM_PHRASE}"
        )
    if environ.get("RMW_IMPLEMENTATION") != "rmw_cyclonedds_cpp":
        raise ValueError(
            "live mode requires RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
        )
    if environ.get("ROS_DOMAIN_ID", "0") != "0":
        raise ValueError("live mode requires ROS_DOMAIN_ID=0 (or unset)")


class PreviewSink:
    live = False

    def wait_ready(self, timeout: float) -> None:
        del timeout

    def move(self, command: MotionCommand) -> None:
        del command

    def stop(self) -> None:
        pass

    def close(self) -> None:
        pass


class RosSportSink:
    live = True

    def __init__(self) -> None:
        try:
            import rclpy
            from unitree_api.msg import Request
        except ImportError as exc:
            raise RuntimeError(
                "live mode needs rclpy and unitree_api; source ROS 2 and the "
                "Unitree message workspace first"
            ) from exc

        self.rclpy = rclpy
        self.request_type = Request
        self.owns_rclpy = not rclpy.ok()
        if self.owns_rclpy:
            rclpy.init(args=[])
        self.node = rclpy.create_node("go2_wasd_teleop")
        self.publisher = self.node.create_publisher(
            Request, SPORT_REQUEST_TOPIC, 10
        )
        self.request_id = time.monotonic_ns()

    def wait_ready(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while (
            self.publisher.get_subscription_count() < 1
            and time.monotonic() < deadline
        ):
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        if self.publisher.get_subscription_count() < 1:
            raise RuntimeError(
                f"no Go2 Sport subscriber found on {SPORT_REQUEST_TOPIC}"
            )

    def _publish(self, api_id: int, parameter: str) -> None:
        request = self.request_type()
        self.request_id += 1
        request.header.identity.id = self.request_id
        request.header.identity.api_id = api_id
        request.parameter = parameter
        self.publisher.publish(request)

    def move(self, command: MotionCommand) -> None:
        self._publish(MOVE_API_ID, command.parameter())

    def stop(self) -> None:
        self._publish(STOP_MOVE_API_ID, "")

    def close(self) -> None:
        self.node.destroy_node()
        if self.owns_rclpy and self.rclpy.ok():
            self.rclpy.shutdown()


class RawTerminal:
    def __init__(self, stream: TextIO) -> None:
        self.stream = stream
        self.fd = stream.fileno()
        self.saved_attributes = None

    def __enter__(self) -> "RawTerminal":
        if not self.stream.isatty():
            raise RuntimeError("keyboard input must come from an interactive terminal")
        self.saved_attributes = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        del exc_type, exc_value, traceback
        if self.saved_attributes is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved_attributes)

    def read_key(self, timeout: float) -> Optional[str]:
        readable, _, _ = select.select([self.fd], [], [], max(timeout, 0.0))
        if not readable:
            return None
        data = os.read(self.fd, 1)
        if not data:
            raise EOFError("terminal input closed")
        return data.decode("utf-8", errors="ignore")


def describe_command(command: MotionCommand) -> str:
    return f"x={command.x:+.3f} y={command.y:+.3f} yaw={command.yaw:+.3f}"


def print_controls(args: argparse.Namespace, output: TextIO) -> None:
    mode = "LIVE" if args.live else "PREVIEW (no ROS publisher)"
    print(f"Go2 WASD teleop: {mode}", file=output)
    print("  W/S: forward/backward", file=output)
    print("  A/D: turn left/right", file=output)
    print("  Space or X: immediate StopMove", file=output)
    print("  Q, Esc, or Ctrl-C: StopMove and quit", file=output)
    print(
        "  Direction commands auto-stop after "
        f"{args.command_timeout:.2f}s without keyboard repeat.",
        file=output,
    )
    print(
        f"  Limits in use: linear={args.linear_speed:.2f}m/s "
        f"yaw={args.angular_speed:.2f}rad/s",
        file=output,
        flush=True,
    )


def publish_stop_burst(sink, count: int = 3, delay: float = 0.05) -> None:
    for index in range(count):
        sink.stop()
        if index + 1 < count:
            time.sleep(delay)


def run_keyboard_loop(
    args: argparse.Namespace,
    sink,
    input_stream: TextIO = sys.stdin,
    output: TextIO = sys.stdout,
) -> None:
    active = ActiveMotion()
    publish_period = 1.0 / args.publish_rate
    stop_keepalive_period = (
        1.0 / args.stop_keepalive_rate
        if args.stop_keepalive_rate > 0.0
        else None
    )
    next_move_publish = time.monotonic()
    last_stop_publish = time.monotonic()
    stopped = True

    with RawTerminal(input_stream) as terminal:
        print_controls(args, output)
        while True:
            now = time.monotonic()
            command = active.current(now)
            if command is not None:
                next_event = next_move_publish
            elif stop_keepalive_period is not None:
                next_event = last_stop_publish + stop_keepalive_period
            else:
                next_event = now + 0.10
            wait_timeout = min(0.05, max(0.0, next_event - now))

            key = terminal.read_key(wait_timeout)
            now = time.monotonic()
            if key is not None:
                action = action_for_key(
                    key, args.linear_speed, args.angular_speed
                )
                if action is not None and action.kind == "move":
                    assert action.command is not None
                    active.activate(action.command, now, args.command_timeout)
                    sink.move(action.command)
                    next_move_publish = now + publish_period
                    stopped = False
                    prefix = "LIVE" if sink.live else "PREVIEW"
                    print(
                        f"{prefix} {action.label}: "
                        f"Move({describe_command(action.command)})",
                        file=output,
                        flush=True,
                    )
                elif action is not None and action.kind == "stop":
                    active.stop()
                    sink.stop()
                    stopped = True
                    last_stop_publish = now
                    prefix = "LIVE" if sink.live else "PREVIEW"
                    print(f"{prefix} StopMove", file=output, flush=True)
                elif action is not None and action.kind == "quit":
                    active.stop()
                    sink.stop()
                    print("StopMove; quitting.", file=output, flush=True)
                    return

            now = time.monotonic()
            command = active.current(now)
            if command is not None and now >= next_move_publish:
                sink.move(command)
                next_move_publish = now + publish_period
                stopped = False
            elif command is None and not stopped:
                sink.stop()
                stopped = True
                last_stop_publish = now
                print("Command timeout: StopMove", file=output, flush=True)
            elif (
                command is None
                and stopped
                and stop_keepalive_period is not None
                and now - last_stop_publish >= stop_keepalive_period
            ):
                sink.stop()
                last_stop_publish = now


def install_exit_signal_handlers() -> None:
    def stop_on_signal(signum, frame) -> None:
        del frame
        raise KeyboardInterrupt(f"signal {signum}")

    for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGQUIT, signal.SIGTSTP):
        signal.signal(signum, stop_on_signal)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        validate_options(args, os.environ)
    except ValueError as exc:
        parser.error(str(exc))

    sink = None
    try:
        sink = RosSportSink() if args.live else PreviewSink()
        sink.wait_ready(args.discovery_timeout)
        install_exit_signal_handlers()
        if args.live:
            publish_stop_burst(sink)
            print(
                "LIVE Sport subscriber ready. Initial StopMove sent.",
                file=sys.stdout,
                flush=True,
            )
        run_keyboard_loop(args, sink)
        return 0
    except (EOFError, KeyboardInterrupt) as exc:
        print(f"Stopping teleop: {exc}", file=sys.stderr)
        return 130
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if sink is not None:
            try:
                publish_stop_burst(sink)
            except Exception as exc:  # pragma: no cover - best-effort shutdown path
                print(f"WARNING: final StopMove failed: {exc}", file=sys.stderr)
            finally:
                sink.close()


if __name__ == "__main__":
    raise SystemExit(main())
