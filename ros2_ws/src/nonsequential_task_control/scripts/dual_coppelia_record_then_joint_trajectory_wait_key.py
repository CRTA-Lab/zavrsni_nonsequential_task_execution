#!/usr/bin/env python3

import argparse
import importlib.util
import sys
import termios
import tty
import time
from pathlib import Path

import rclpy


BASE_SCRIPT = Path(__file__).resolve().with_name(
    "dual_coppelia_record_then_joint_trajectory.py"
)


def import_original_module():
    if not BASE_SCRIPT.is_file():
        raise RuntimeError(
            "Nedostaje interna JointTrajectory implementacija:\n"
            f"{BASE_SCRIPT}\n"
            "Ponovno izgradi i instaliraj nonsequential_task_control paket."
        )

    spec = importlib.util.spec_from_file_location(
        "nonsequential_task_joint_trajectory_base",
        str(BASE_SCRIPT),
    )

    if spec is None or spec.loader is None:
        raise RuntimeError(f"Ne mogu ucitati Python modul: {BASE_SCRIPT}")

    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)

    return module


def read_single_key():
    """
    Cita jednu tipku bez ENTER.
    Ako terminal ne podrzava raw mode, koristi input fallback.
    """
    try:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        return key

    except Exception:
        text = input()
        if text == "":
            return "\n"
        return text[0]


def make_wait_class(original):
    class DualWaitKey(original.DualCoppeliaRecordThenTrajectory):
        def send_goals_together(self):
            goals = {
                robot: self.build_goal(robot)
                for robot in original.ROBOTS
            }

            print("")
            print("============================================================")
            print("SPREMNE SU DVIJE JOINTTRAJECTORY PUTANJE")
            print("============================================================")

            for robot in original.ROBOTS:
                total_time = (
                    goals[robot].trajectory.points[-1].time_from_start.sec
                    + goals[robot].trajectory.points[-1].time_from_start.nanosec * 1e-9
                )

                print("")
                print(f"{robot}:")
                print(f"  broj tocaka: {len(goals[robot].trajectory.points)}")
                print(f"  trajanje:    {total_time:.2f} s")

            print("")
            print("Robotima jos NIJE poslana putanja.")
            print("")
            print("Pritisni:")
            print("  s  = START, posalji obje putanje robotima")
            print("  q  = CANCEL, izadi bez slanja")
            print("")
            print("Drzi ruku na STOP tipki.")

            while True:
                key = read_single_key()

                if key.lower() == "s":
                    print("")
                    print("START pritisnut. Saljem obje putanje za 2 sekunde...")
                    time.sleep(2.0)
                    break

                if key.lower() == "q":
                    print("")
                    print("CANCEL pritisnut. Izlazim bez slanja robotima.")
                    return

                print("")
                print("Nepoznata tipka. Pritisni 's' za START ili 'q' za CANCEL.")

            send_futures = {}

            for robot in original.ROBOTS:
                send_futures[robot] = self.action_clients[robot].send_goal_async(goals[robot])

            goal_handles = {}

            for robot in original.ROBOTS:
                rclpy.spin_until_future_complete(self, send_futures[robot])
                goal_handle = send_futures[robot].result()

                if goal_handle is None:
                    raise RuntimeError(f"{robot}: nisam dobio goal_handle.")

                if not goal_handle.accepted:
                    raise RuntimeError(f"{robot}: goal je odbijen.")

                goal_handles[robot] = goal_handle
                print(f"{robot}: GOAL PRIHVACEN")

            result_futures = {}

            for robot in original.ROBOTS:
                result_futures[robot] = goal_handles[robot].get_result_async()

            print("")
            print("Oba robota izvode koordinirane JointTrajectory putanje.")
            print("Cekam rezultate...")

            for robot in original.ROBOTS:
                rclpy.spin_until_future_complete(self, result_futures[robot])
                result_response = result_futures[robot].result()

                if result_response is None:
                    raise RuntimeError(f"{robot}: nisam dobio action result.")

                result = result_response.result

                print("")
                print(f"{robot} ACTION RESULT:")
                print(f"  error_code: {result.error_code}")
                print(f"  error_string: {result.error_string}")

    return DualWaitKey


def main():
    original = import_original_module()
    DualWaitKey = make_wait_class(original)

    parser = argparse.ArgumentParser()

    parser.add_argument("--record-seconds", type=float, default=30.0)
    parser.add_argument("--time-scale", type=float, default=0.5)
    parser.add_argument("--initial-hold", type=float, default=1.0)
    parser.add_argument("--final-hold", type=float, default=1.0)
    parser.add_argument("--pre-start-hold", type=float, default=0.25)

    parser.add_argument("--downsample", type=int, default=5)
    parser.add_argument("--filter-alpha", type=float, default=0.30)

    parser.add_argument("--max-start-error", type=float, default=0.25)
    parser.add_argument("--max-step", type=float, default=0.35)

    parser.add_argument("--max-raw-velocity", type=float, default=2.0)
    parser.add_argument("--max-raw-acceleration", type=float, default=6.0)

    parser.add_argument("--max-velocity", type=float, default=5.0)
    parser.add_argument("--max-acceleration", type=float, default=5.0)

    args = parser.parse_args()

    if args.record_seconds <= 0.0:
        print("GRESKA: --record-seconds mora biti veci od 0.")
        sys.exit(1)

    if args.time_scale <= 0.0:
        print("GRESKA: --time-scale mora biti veci od 0.")
        sys.exit(1)

    if args.downsample < 1:
        print("GRESKA: --downsample mora biti barem 1.")
        sys.exit(1)

    if args.filter_alpha <= 0.0 or args.filter_alpha > 1.0:
        print("GRESKA: --filter-alpha mora biti u intervalu (0, 1].")
        sys.exit(1)

    rclpy.init()

    node = DualWaitKey(args)

    try:
        node.run()

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("")
        print("Prekinuto s CTRL+C.")
        sys.exit(1)
    except Exception as exc:
        print("")
        print("GRESKA:")
        print(exc)
        sys.exit(1)
