#!/usr/bin/env python3

import argparse
import csv
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration as DurationMsg


ROBOTS = ["robot1", "robot2"]

BASE_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


@dataclass
class Sample:
    t: float
    q: List[float]
    qd: List[float]
    qdd: List[float]


def controller_joint_names(robot: str):
    return [
        f"{robot}_shoulder_pan_joint",
        f"{robot}_shoulder_lift_joint",
        f"{robot}_elbow_joint",
        f"{robot}_wrist_1_joint",
        f"{robot}_wrist_2_joint",
        f"{robot}_wrist_3_joint",
    ]


def normalize_joint_name(name: str) -> str:
    name = name.split("/")[-1]

    if name.startswith("robot1_"):
        return name[len("robot1_"):]

    if name.startswith("robot2_"):
        return name[len("robot2_"):]

    return name


def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi

    while a < -math.pi:
        a += 2.0 * math.pi

    return a


def duration_from_seconds(seconds: float) -> DurationMsg:
    if seconds < 0.0:
        seconds = 0.0

    sec = int(math.floor(seconds))
    nanosec = int(round((seconds - sec) * 1e9))

    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000

    return DurationMsg(sec=sec, nanosec=nanosec)


def extract_joint_vector(msg: JointState, field_name: str) -> Optional[List[float]]:
    values = getattr(msg, field_name)

    if values is None or len(values) == 0:
        return None

    if len(msg.name) == 0:
        if len(values) >= 6:
            out = [float(values[i]) for i in range(6)]

            if all(math.isfinite(x) for x in out):
                return out

        return None

    if len(values) < len(msg.name):
        return None

    lookup = {}

    for i, joint_name in enumerate(msg.name):
        lookup[normalize_joint_name(joint_name)] = i

    out = []

    for joint_name in BASE_JOINT_NAMES:
        if joint_name not in lookup:
            return None

        index = lookup[joint_name]

        if index >= len(values):
            return None

        value = float(values[index])

        if not math.isfinite(value):
            return None

        out.append(value)

    return out


def clamp(x: float, limit: float) -> float:
    if x > limit:
        return limit

    if x < -limit:
        return -limit

    return x


def max_abs(values: List[float]) -> float:
    return max(abs(x) for x in values)


class DualCoppeliaRecordThenTrajectory(Node):
    def __init__(self, args):
        super().__init__("dual_coppelia_record_then_joint_trajectory")

        self.args = args

        self.actual_q = {robot: None for robot in ROBOTS}
        self.actual_qd = {robot: [0.0] * 6 for robot in ROBOTS}

        self.latest_q = {robot: None for robot in ROBOTS}
        self.latest_qd = {robot: None for robot in ROBOTS}
        self.latest_qdd = {robot: None for robot in ROBOTS}

        self.last_raw_q = {robot: None for robot in ROBOTS}
        self.last_unwrapped_q = {robot: None for robot in ROBOTS}

        self.filtered_qd = {robot: None for robot in ROBOTS}
        self.filtered_qdd = {robot: None for robot in ROBOTS}

        self.samples = {robot: [] for robot in ROBOTS}
        self.target_count = {robot: 0 for robot in ROBOTS}

        self.recording = False

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=300,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.ros_subscriptions = []

        for robot in ROBOTS:
            self.ros_subscriptions.append(
                self.create_subscription(
                    JointState,
                    f"/{robot}/coppelia_joint_target",
                    lambda msg, robot=robot: self.target_callback(robot, msg),
                    qos,
                )
            )

            self.ros_subscriptions.append(
                self.create_subscription(
                    JointState,
                    f"/{robot}/joint_states",
                    lambda msg, robot=robot: self.actual_callback(robot, msg),
                    qos,
                )
            )

        self.action_clients = {
            robot: ActionClient(
                self,
                FollowJointTrajectory,
                f"/{robot}/joint_trajectory_controller/follow_joint_trajectory",
            )
            for robot in ROBOTS
        }

        self.get_logger().info("")
        self.get_logger().info("Dual Coppelia record -> joint_trajectory_controller")
        self.get_logger().info("")
        self.get_logger().info("Robot2 se snima od approach faze.")
        self.get_logger().info("Robot1 pocinje snimati tek kad Coppelia posalje njegov target nakon robot2 ready signala.")
        self.get_logger().info("Pri izvedbi obje trajektorije imaju isti globalni vremenski pocetak.")

    def now_s(self):
        return time.time()

    def actual_callback(self, robot: str, msg: JointState):
        q = extract_joint_vector(msg, "position")
        qd = extract_joint_vector(msg, "velocity")

        if q is not None:
            self.actual_q[robot] = q

        if qd is not None:
            self.actual_qd[robot] = qd

    def unwrap_q(self, robot: str, q_raw: List[float]) -> List[float]:
        if self.last_raw_q[robot] is None or self.last_unwrapped_q[robot] is None:
            if self.actual_q[robot] is not None:
                out = []

                for i in range(6):
                    out.append(self.actual_q[robot][i] + wrap_to_pi(q_raw[i] - self.actual_q[robot][i]))

                return out

            return list(q_raw)

        out = []

        for i in range(6):
            dq = wrap_to_pi(q_raw[i] - self.last_raw_q[robot][i])
            out.append(self.last_unwrapped_q[robot][i] + dq)

        return out

    def target_callback(self, robot: str, msg: JointState):
        t = self.now_s()

        q_raw = extract_joint_vector(msg, "position")

        if q_raw is None:
            return

        q = self.unwrap_q(robot, q_raw)

        qd = extract_joint_vector(msg, "velocity")
        qdd = extract_joint_vector(msg, "effort")

        if qd is None:
            qd = [0.0] * 6

        if qdd is None:
            qdd = [0.0] * 6

        alpha = self.args.filter_alpha

        if self.filtered_qd[robot] is None:
            self.filtered_qd[robot] = list(qd)
        else:
            self.filtered_qd[robot] = [
                alpha * qd[i] + (1.0 - alpha) * self.filtered_qd[robot][i]
                for i in range(6)
            ]

        if self.filtered_qdd[robot] is None:
            self.filtered_qdd[robot] = list(qdd)
        else:
            self.filtered_qdd[robot] = [
                alpha * qdd[i] + (1.0 - alpha) * self.filtered_qdd[robot][i]
                for i in range(6)
            ]

        qd = [
            clamp(self.filtered_qd[robot][i], self.args.max_raw_velocity)
            for i in range(6)
        ]

        qdd = [
            clamp(self.filtered_qdd[robot][i], self.args.max_raw_acceleration)
            for i in range(6)
        ]

        self.latest_q[robot] = list(q)
        self.latest_qd[robot] = list(qd)
        self.latest_qdd[robot] = list(qdd)

        self.last_raw_q[robot] = list(q_raw)
        self.last_unwrapped_q[robot] = list(q)

        self.target_count[robot] += 1

        if self.recording:
            self.samples[robot].append(
                Sample(
                    t=t,
                    q=list(q),
                    qd=list(qd),
                    qdd=list(qdd),
                )
            )

    def wait_for_action_servers(self):
        print("")
        print("Cekam action servere...")

        for robot in ROBOTS:
            ok = self.action_clients[robot].wait_for_server(timeout_sec=10.0)

            if not ok:
                raise RuntimeError(
                    f"Action server nije dostupan za {robot}: "
                    f"/{robot}/joint_trajectory_controller/follow_joint_trajectory"
                )

            print(f"{robot}: action server dostupan.")

    def wait_for_robot_feedback(self):
        print("")
        print("Cekam /joint_states za oba robota...")

        start = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if all(self.actual_q[robot] is not None for robot in ROBOTS):
                print("Feedback primljen za oba robota.")
                return

            if time.time() - start > 10.0:
                raise RuntimeError("Nisam dobio /joint_states za oba robota.")

    def wait_for_robot2_target(self):
        print("")
        print("Cekam /robot2/coppelia_joint_target.")
        print("Pokreni CoppeliaSim simulation.")
        print("Robot2 ce prvo syncati i ici u approach.")

        start = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.latest_q["robot2"] is not None:
                print("Robot2 Coppelia target primljen. Krecem snimati oba robota.")
                return

            if time.time() - start > 60.0:
                raise RuntimeError("Nisam dobio /robot2/coppelia_joint_target.")

    def record_samples(self):
        print("")
        print("Snimam Coppelia q/q_dot/q_ddot za oba robota.")
        print(f"Trajanje snimanja: {self.args.record_seconds:.2f} s")
        print("")
        print("Napomena:")
        print("  robot2 target se snima odmah od approach faze")
        print("  robot1 target ce se pojaviti tek nakon robot2 READY signala")

        self.samples = {robot: [] for robot in ROBOTS}
        self.recording = True

        start_wall = time.time()
        last_print = time.time()
        last_counts = {robot: self.target_count[robot] for robot in ROBOTS}

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

            elapsed = time.time() - start_wall

            if time.time() - last_print >= 1.0:
                dt = time.time() - last_print

                hz1 = (self.target_count["robot1"] - last_counts["robot1"]) / dt
                hz2 = (self.target_count["robot2"] - last_counts["robot2"]) / dt

                last_counts = {robot: self.target_count[robot] for robot in ROBOTS}
                last_print = time.time()

                print(
                    f"  recording {elapsed:.1f}/{self.args.record_seconds:.1f} s, "
                    f"robot1_samples={len(self.samples['robot1'])}, "
                    f"robot2_samples={len(self.samples['robot2'])}, "
                    f"hz1={hz1:.1f}, hz2={hz2:.1f}"
                )

            if elapsed >= self.args.record_seconds:
                break

        self.recording = False

        for robot in ROBOTS:
            if len(self.samples[robot]) < 10:
                raise RuntimeError(
                    f"Premalo sampleova za {robot}: {len(self.samples[robot])}. "
                    "Produzi --record-seconds ili provjeri Coppelia skriptu."
                )

        print("")
        print("Snimanje gotovo.")
        print(f"robot1 sampleova: {len(self.samples['robot1'])}")
        print(f"robot2 sampleova: {len(self.samples['robot2'])}")

    def downsample_samples(self):
        if self.args.downsample <= 1:
            return

        for robot in ROBOTS:
            old_count = len(self.samples[robot])

            new_samples = self.samples[robot][::self.args.downsample]

            if new_samples[-1] is not self.samples[robot][-1]:
                new_samples.append(self.samples[robot][-1])

            self.samples[robot] = new_samples

            print(f"{robot} downsample: {old_count} -> {len(self.samples[robot])}")

    def validate_samples(self):
        print("")
        print("Provjeravam sigurnost trajektorija...")

        for robot in ROBOTS:
            if self.actual_q[robot] is None:
                raise RuntimeError(f"Nema actual_q za {robot}.")

            first_q = self.samples[robot][0].q

            start_errors = [
                wrap_to_pi(first_q[i] - self.actual_q[robot][i])
                for i in range(6)
            ]

            start_error = max_abs(start_errors)

            print("")
            print(f"{robot}:")
            print(f"  start error = {start_error:.4f} rad")
            print(f"  limit       = {self.args.max_start_error:.4f} rad")

            if start_error > self.args.max_start_error:
                raise RuntimeError(
                    f"{robot}: start error je prevelik. "
                    "Robot i Coppelia nisu dovoljno sinkronizirani."
                )

            max_step = 0.0
            max_v = 0.0
            max_a = 0.0

            for i in range(1, len(self.samples[robot])):
                step_values = [
                    self.samples[robot][i].q[j] - self.samples[robot][i - 1].q[j]
                    for j in range(6)
                ]

                step = max_abs(step_values)
                max_step = max(max_step, step)

                if step > self.args.max_step:
                    raise RuntimeError(
                        f"{robot}: prevelik skok izmedu Coppelia tocaka. "
                        f"Index={i}, step={step:.4f}, limit={self.args.max_step:.4f}"
                    )

            for sample in self.samples[robot]:
                for j in range(6):
                    v = abs(sample.qd[j] / self.args.time_scale)
                    a = abs(sample.qdd[j] / (self.args.time_scale * self.args.time_scale))

                    max_v = max(max_v, v)
                    max_a = max(max_a, a)

                    if v > self.args.max_velocity:
                        raise RuntimeError(
                            f"{robot}: previsoka brzina nakon time_scale. "
                            f"Zglob={BASE_JOINT_NAMES[j]}, v={v:.4f}, limit={self.args.max_velocity:.4f}"
                        )

                    if a > self.args.max_acceleration:
                        raise RuntimeError(
                            f"{robot}: previsoka akceleracija nakon time_scale. "
                            f"Zglob={BASE_JOINT_NAMES[j]}, a={a:.4f}, limit={self.args.max_acceleration:.4f}"
                        )

            print(f"  max step = {max_step:.4f} rad")
            print(f"  max v    = {max_v:.4f} rad/s")
            print(f"  max a    = {max_a:.4f} rad/s^2")

        print("")
        print("Obje trajektorije prosle su sigurnosnu provjeru.")

    def save_csv(self):
        for robot in ROBOTS:
            csv_path = os.path.expanduser(f"~/logs/{robot}_dual_joint_trajectory_record.csv")
            csv_dir = os.path.dirname(csv_path)

            if csv_dir:
                os.makedirs(csv_dir, exist_ok=True)

            with open(csv_path, "w", newline="") as f:
                writer = csv.writer(f)

                header = ["t_global_s"]

                for name in BASE_JOINT_NAMES:
                    header.append(f"{name}_q_rad")

                for name in BASE_JOINT_NAMES:
                    header.append(f"{name}_q_dot_rad_s")

                for name in BASE_JOINT_NAMES:
                    header.append(f"{name}_q_ddot_rad_s2")

                writer.writerow(header)

                t0 = self.global_t0()

                for sample in self.samples[robot]:
                    row = [sample.t - t0]
                    row.extend(sample.q)
                    row.extend(sample.qd)
                    row.extend(sample.qdd)
                    writer.writerow(row)

            print(f"CSV spremljen: {csv_path}")

    def global_t0(self):
        first_times = [
            self.samples[robot][0].t
            for robot in ROBOTS
            if len(self.samples[robot]) > 0
        ]

        return min(first_times)

    def build_goal(self, robot: str):
        t0 = self.global_t0()

        trajectory = JointTrajectory()
        trajectory.joint_names = controller_joint_names(robot)

        points = []

        p0 = JointTrajectoryPoint()
        p0.positions = [float(x) for x in self.actual_q[robot]]
        p0.velocities = [0.0] * 6
        p0.accelerations = [0.0] * 6
        p0.time_from_start = duration_from_seconds(0.5)
        points.append(p0)

        first_point_time = self.args.initial_hold + (self.samples[robot][0].t - t0) * self.args.time_scale

        hold_time = first_point_time - self.args.pre_start_hold

        if hold_time > 0.55:
            p_hold = JointTrajectoryPoint()
            p_hold.positions = [float(x) for x in self.actual_q[robot]]
            p_hold.velocities = [0.0] * 6
            p_hold.accelerations = [0.0] * 6
            p_hold.time_from_start = duration_from_seconds(hold_time)
            points.append(p_hold)

        last_time = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9

        for sample in self.samples[robot]:
            relative_t = sample.t - t0
            point_time = self.args.initial_hold + relative_t * self.args.time_scale

            if point_time <= last_time:
                continue

            p = JointTrajectoryPoint()
            p.positions = [float(x) for x in sample.q]
            p.velocities = [float(x / self.args.time_scale) for x in sample.qd]
            p.accelerations = [
                float(x / (self.args.time_scale * self.args.time_scale))
                for x in sample.qdd
            ]
            p.time_from_start = duration_from_seconds(point_time)

            points.append(p)
            last_time = point_time

        last_q = self.samples[robot][-1].q

        p_end = JointTrajectoryPoint()
        p_end.positions = [float(x) for x in last_q]
        p_end.velocities = [0.0] * 6
        p_end.accelerations = [0.0] * 6
        p_end.time_from_start = duration_from_seconds(last_time + self.args.final_hold)
        points.append(p_end)

        trajectory.points = points

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = duration_from_seconds(3.0)

        return goal

    def send_goals_together(self):
        goals = {
            robot: self.build_goal(robot)
            for robot in ROBOTS
        }

        print("")
        print("Spremne su dvije JointTrajectory putanje.")
        for robot in ROBOTS:
            total_time = (
                goals[robot].trajectory.points[-1].time_from_start.sec
                + goals[robot].trajectory.points[-1].time_from_start.nanosec * 1e-9
            )

            print(f"{robot}:")
            print(f"  broj tocaka: {len(goals[robot].trajectory.points)}")
            print(f"  trajanje:    {total_time:.2f} s")

        print("")
        print("Slanje obje putanje za 5 sekundi.")
        print("Drzi ruku na STOP tipki.")

        for i in range(5, 0, -1):
            print(f"{i}...")
            time.sleep(1.0)

        send_futures = {}

        for robot in ROBOTS:
            send_futures[robot] = self.action_clients[robot].send_goal_async(goals[robot])

        goal_handles = {}

        for robot in ROBOTS:
            rclpy.spin_until_future_complete(self, send_futures[robot])
            goal_handle = send_futures[robot].result()

            if goal_handle is None:
                raise RuntimeError(f"{robot}: nisam dobio goal_handle.")

            if not goal_handle.accepted:
                raise RuntimeError(f"{robot}: goal je odbijen.")

            goal_handles[robot] = goal_handle
            print(f"{robot}: GOAL PRIHVACEN")

        result_futures = {}

        for robot in ROBOTS:
            result_futures[robot] = goal_handles[robot].get_result_async()

        print("")
        print("Oba robota izvode koordinirane JointTrajectory putanje.")
        print("Cekam rezultate...")

        for robot in ROBOTS:
            rclpy.spin_until_future_complete(self, result_futures[robot])
            result_response = result_futures[robot].result()

            if result_response is None:
                raise RuntimeError(f"{robot}: nisam dobio action result.")

            result = result_response.result

            print("")
            print(f"{robot} ACTION RESULT:")
            print(f"  error_code: {result.error_code}")
            print(f"  error_string: {result.error_string}")

    def run(self):
        self.wait_for_action_servers()
        self.wait_for_robot_feedback()
        self.wait_for_robot2_target()
        self.record_samples()
        self.downsample_samples()
        self.validate_samples()
        self.save_csv()
        self.send_goals_together()


def main():
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

    node = DualCoppeliaRecordThenTrajectory(args)

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
