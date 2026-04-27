from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

from .control_bus import read_commands


def parse_bool(value: str) -> bool:
    lowered = value.strip().lower()
    if lowered in {"1", "true", "yes", "y", "on"}:
        return True
    if lowered in {"0", "false", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid bool value: {value}")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish minimal offline mock topics for behavior_tree decision tests.")
    parser.add_argument("--team", choices=("red", "blue"), default="red")
    parser.add_argument("--target-source", choices=("none", "predictor", "buff", "outpost"), default="none")
    parser.add_argument("--target-status", type=parse_bool, default=True)
    parser.add_argument("--target-yaw", type=float, default=0.0)
    parser.add_argument("--target-pitch", type=float, default=0.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--posture", type=int, default=1)
    parser.add_argument("--time-left", type=int, default=420)
    parser.add_argument("--ammo-left", type=int, default=200)
    parser.add_argument("--self-health", type=int, default=400)
    parser.add_argument("--enemy-health", type=int, default=400)
    parser.add_argument("--simulate-match", type=parse_bool, default=False)
    parser.add_argument("--start-running", type=parse_bool, default=False)
    parser.add_argument("--match-duration-sec", type=int, default=420)
    parser.add_argument("--control-file", default="")
    args = parser.parse_args(argv)
    if args.hz <= 0:
        parser.error("--hz must be > 0")
    if args.match_duration_sec <= 0:
        parser.error("--match-duration-sec must be > 0")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        import rclpy
        from auto_aim_common.msg import Target
        from gimbal_driver.msg import BuffData, GameData, GimbalAngles, Health
        from rclpy.node import Node
        from std_msgs.msg import Bool, UInt8, UInt16, UInt32
    except ImportError as exc:
        print(
            "ROS Python deps are missing. Source ROS/workspace first, e.g. "
            "`source /opt/ros/humble/setup.bash && source install/setup.bash`",
            flush=True,
        )
        print(str(exc), flush=True)
        return 2

    class MockDecisionInputs(Node):
        def __init__(self) -> None:
            super().__init__("decision_viz_mock_inputs")
            self.team_red = args.team == "red"
            self.match_duration_sec = max(1, min(65535, int(args.match_duration_sec)))
            self.time_left_float = float(max(0, min(self.match_duration_sec, int(args.time_left))))
            self.time_left = int(math.ceil(self.time_left_float))
            self.ammo_left = max(0, min(65535, int(args.ammo_left)))
            self.self_health = max(0, min(65535, int(args.self_health)))
            self.enemy_health = max(0, min(65535, int(args.enemy_health)))
            self.posture = max(0, min(255, int(args.posture)))
            self.simulate_match = bool(args.simulate_match)
            self.match_started = not self.simulate_match
            self.match_running = not self.simulate_match
            if self.simulate_match and bool(args.start_running):
                self.match_started = True
                self.match_running = True
            self.last_wall_time = time.monotonic()
            self.control_path: Path | None = None
            self.control_offset = 0
            if str(args.control_file).strip():
                self.control_path = Path(args.control_file).expanduser().resolve()
                self.control_path.parent.mkdir(parents=True, exist_ok=True)
                if self.control_path.exists():
                    try:
                        self.control_offset = int(self.control_path.stat().st_size)
                    except OSError:
                        self.control_offset = 0

            self.pub_gimbal_angles = self.create_publisher(GimbalAngles, "/ly/gimbal/angles", 10)
            self.pub_gimbal_posture = self.create_publisher(UInt8, "/ly/gimbal/posture", 10)
            self.pub_team = self.create_publisher(Bool, "/ly/me/is_team_red", 10)
            self.pub_game_start = self.create_publisher(Bool, "/ly/game/is_start", 10)
            self.pub_time_left = self.create_publisher(UInt16, "/ly/game/time_left", 10)
            self.pub_ammo_left = self.create_publisher(UInt16, "/ly/me/ammo_left", 10)
            self.pub_game_all = self.create_publisher(GameData, "/ly/game/all", 10)
            self.pub_me_hp = self.create_publisher(Health, "/ly/me/hp", 10)
            self.pub_enemy_hp = self.create_publisher(Health, "/ly/enemy/hp", 10)
            self.pub_team_buff = self.create_publisher(BuffData, "/ly/team/buff", 10)
            self.pub_rfid = self.create_publisher(UInt32, "/ly/me/rfid", 10)

            self.target_source = args.target_source
            self.pub_target = None
            if self.target_source == "predictor":
                self.pub_target = self.create_publisher(Target, "/ly/predictor/target", 10)
            elif self.target_source == "buff":
                self.pub_target = self.create_publisher(Target, "/ly/buff/target", 10)
            elif self.target_source == "outpost":
                self.pub_target = self.create_publisher(Target, "/ly/outpost/target", 10)

            period = 1.0 / float(args.hz)
            self.timer = self.create_timer(period, self._publish_all)
            self.get_logger().info(
                "mock inputs started: "
                f"team={args.team} target_source={args.target_source} hz={args.hz:.1f} "
                f"time_left={self.time_left} ammo_left={self.ammo_left} "
                f"simulate_match={str(self.simulate_match).lower()} "
                f"running={str(self.match_running).lower()} "
                f"control_file={(self.control_path.as_posix() if self.control_path else '-')}"
            )

        def _clamp_time_left(self) -> None:
            self.time_left_float = max(0.0, min(float(self.match_duration_sec), float(self.time_left_float)))
            self.time_left = max(0, min(self.match_duration_sec, int(math.ceil(self.time_left_float))))

        def _apply_command(self, payload: dict) -> None:
            command = str(payload.get("command", "")).strip().lower()
            if not command:
                return
            if command == "start":
                if self.time_left_float <= 0.0:
                    self.time_left_float = float(self.match_duration_sec)
                    self._clamp_time_left()
                self.match_started = True
                self.match_running = True
                return
            if command == "pause":
                self.match_running = False
                return
            if command == "reset":
                self.time_left_float = float(self.match_duration_sec)
                self._clamp_time_left()
                self.match_started = False
                self.match_running = False
                return
            if command == "rewind":
                try:
                    delta = float(payload.get("seconds", 0.0))
                except (TypeError, ValueError):
                    return
                if not math.isfinite(delta):
                    return
                self.time_left_float += max(0.0, delta)
                self._clamp_time_left()
                return
            if command == "forward":
                try:
                    delta = float(payload.get("seconds", 0.0))
                except (TypeError, ValueError):
                    return
                if not math.isfinite(delta):
                    return
                self.time_left_float -= max(0.0, delta)
                self._clamp_time_left()
                return
            if command == "set_time_left":
                try:
                    target = float(payload.get("seconds", self.time_left_float))
                except (TypeError, ValueError):
                    return
                if not math.isfinite(target):
                    return
                self.time_left_float = target
                self._clamp_time_left()

        def _poll_commands(self) -> None:
            if self.control_path is None:
                return
            commands, new_offset = read_commands(self.control_path, self.control_offset)
            self.control_offset = new_offset
            for payload in commands:
                self._apply_command(payload)

        def _publish_all(self) -> None:
            self._poll_commands()
            now_wall = time.monotonic()
            dt = max(0.0, now_wall - self.last_wall_time)
            self.last_wall_time = now_wall
            if self.simulate_match and self.match_started and self.match_running and self.time_left_float > 0.0:
                self.time_left_float -= dt
                self._clamp_time_left()
                if self.time_left_float <= 0.0:
                    self.match_running = False
            now = self.get_clock().now().to_msg()

            gimbal = GimbalAngles()
            gimbal.yaw = float(args.yaw)
            gimbal.pitch = float(args.pitch)
            gimbal.header.stamp = now
            self.pub_gimbal_angles.publish(gimbal)

            posture = UInt8()
            posture.data = self.posture
            self.pub_gimbal_posture.publish(posture)

            team = Bool()
            team.data = self.team_red
            self.pub_team.publish(team)

            game_start = Bool()
            game_start.data = bool(self.match_started)
            self.pub_game_start.publish(game_start)

            time_left = UInt16()
            time_left.data = self.time_left
            self.pub_time_left.publish(time_left)

            ammo = UInt16()
            ammo.data = self.ammo_left
            self.pub_ammo_left.publish(ammo)

            game_all = GameData()
            game_all.gamecode = 0
            game_all.ammoleft = self.ammo_left
            game_all.timeleft = self.time_left
            game_all.selfhealth = self.self_health
            game_all.exteventdata = 0
            self.pub_game_all.publish(game_all)

            me_hp = Health()
            me_hp.hero = self.self_health
            me_hp.engineer = self.self_health
            me_hp.infantry1 = self.self_health
            me_hp.infantry2 = self.self_health
            me_hp.reserve = self.self_health
            me_hp.sentry = self.self_health
            self.pub_me_hp.publish(me_hp)

            enemy_hp = Health()
            enemy_hp.hero = self.enemy_health
            enemy_hp.engineer = self.enemy_health
            enemy_hp.infantry1 = self.enemy_health
            enemy_hp.infantry2 = self.enemy_health
            enemy_hp.reserve = self.enemy_health
            enemy_hp.sentry = self.enemy_health
            self.pub_enemy_hp.publish(enemy_hp)

            team_buff = BuffData()
            team_buff.recoverybuff = 0
            team_buff.coolingbuff = 0
            team_buff.defencebuff = 0
            team_buff.vulnerabilitybuff = 0
            team_buff.attackbuff = 0
            team_buff.remainingenergy = 0
            self.pub_team_buff.publish(team_buff)

            rfid = UInt32()
            rfid.data = 0
            self.pub_rfid.publish(rfid)

            if self.pub_target is not None:
                target = Target()
                target.header.stamp = now
                target.status = bool(args.target_status)
                target.buff_follow = self.target_source == "buff"
                target.yaw = float(args.target_yaw)
                target.pitch = float(args.target_pitch)
                self.pub_target.publish(target)

    rclpy.init(args=None)
    node = MockDecisionInputs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
