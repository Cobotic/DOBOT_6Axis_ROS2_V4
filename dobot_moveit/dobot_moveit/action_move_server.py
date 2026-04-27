#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
@author FTX
@date 2025 / 03 / 03
'''

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.task import Future
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dobot_msgs_v4.srv import *   # 自定义的服务接口
import os


_ACTION_MOVE_SERVER_LOG_DIR = "/data/roslogs/action_move_server"


def _duration_to_s(d) -> float:
    return float(getattr(d, "sec", 0)) + float(getattr(d, "nanosec", 0)) * 1e-9


def _env_flag(name: str) -> bool:
    v = (os.getenv(name) or "").strip().lower()
    return v in {"1", "true", "yes", "y", "on"}


def _maybe_enable_sched_fifo() -> None:
    if not _env_flag("ANYROB_ACTION_MOVE_SERVER_SCHED_FIFO"):
        return

    prio_raw = (os.getenv("ANYROB_ACTION_MOVE_SERVER_SCHED_FIFO_PRIORITY") or "50").strip()
    try:
        prio = int(prio_raw)
    except ValueError:
        prio = 50

    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(prio))
    except PermissionError as exc:
        # Must run as root or have CAP_SYS_NICE (e.g., docker compose cap_add: SYS_NICE)
        print(f"[action_move_server] Failed to enable SCHED_FIFO (prio={prio}): {exc}")
    except Exception as exc:
        print(f"[action_move_server] Failed to enable SCHED_FIFO (prio={prio}): {exc}")

class FollowJointTrajectoryServer(Node):

    def __init__(self):
        super().__init__('dobot_group_controller')
        name = (os.getenv("DOBOT_TYPE") or "").strip() or "cr5"
        # 创建FollowJointTrajectory动作服务器
        # Use a *relative* name so the action server respects the node namespace.
        # When this node runs under /robot1, the action becomes /robot1/<type>_group_controller/follow_joint_trajectory.
        self._action_server = ActionServer(self,FollowJointTrajectory,f'{name}_group_controller/follow_joint_trajectory',self.execute_callback)
        self.get_logger().info("FollowJointTrajectory Action Server is ready...")
        self.EnableRobot_l = self.create_client(EnableRobot, '/dobot_bringup_ros2/srv/EnableRobot')
        self.ServoJ_l = self.create_client(ServoJ, '/dobot_bringup_ros2/srv/ServoJ')
        while not self.EnableRobot_l.wait_for_service(timeout_sec=1.0):  # 循环等待服务器端成功启动
            self.get_logger().info('service not available, waiting again...')

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Received a new trajectory goal!")
        # 获取目标轨迹
        trajectory = goal_handle.request.trajectory
        try:
            ok = self.execution_trajectory(trajectory)
        except Exception as exc:
            self.get_logger().error(f"Trajectory execution failed: {exc}")
            ok = False

        result = FollowJointTrajectory.Result()
        if ok:
            goal_handle.succeed()
            result.error_code = 0
        else:
            goal_handle.abort()
            # Nonzero indicates failure. (We don't have a precise mapping here.)
            result.error_code = 1

        return result

    def _spin_until_future(self, future: Future, timeout_s: float) -> bool:
        end = time.time() + timeout_s
        while rclpy.ok() and not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()

    def _spin_sleep(self, seconds: float) -> None:
        end = time.monotonic() + max(0.0, seconds)
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.05, end - time.monotonic()))

    def execution_trajectory(self, trajectory: JointTrajectory) -> bool:
        self.get_logger().info("Joint Names: {}".format(trajectory.joint_names))
        Positions = []
        Times = []
        for i, point in enumerate(trajectory.points):
            joint= []
            for ii in point.positions:
                joint.append(180 * ii / 3.14159)
            Positions.append(joint)
            Times.append(_duration_to_s(point.time_from_start))
            self.get_logger().info(
               f"Point {i}: Positions: {joint}, Velocities: {point.velocities}, Accelerations: {point.accelerations}, TimeFromStart (ms): {point.time_from_start.sec * 1000 + point.time_from_start.nanosec / 1e6}"
            )
        ok = True
        # Trajectory points are assumed to be spaced at a fixed dt.
        # Strategy:
        # - Send point i early (before reaching point i-1) at time (i-1)*dt - ddt.
        # - Set ServoJ t to the remaining time until the planned reach time i*dt.
        # This reduces "buffer underrun" stops while keeping planned timing.
        if True:
            t0 = time.perf_counter()
            ddt = 0.03
            min_servoj_t = 0.004
            the_log = [f"THIS LOG IS CALLED 1 ddt={ddt} min_servoj_t={min_servoj_t}"]

            for i in range(1, len(Positions)):
                planned_prev = Times[i - 1]
                planned_reach = Times[i]
                dt_i = planned_reach - planned_prev
                if dt_i < min_servoj_t:
                    dt_i = min_servoj_t

                # Send the next point slightly before the previous point's planned time.
                send_at = planned_prev - ddt
                if send_at < 0.0:
                    send_at = 0.0

                elapsed = time.perf_counter() - t0
                
                sleep_s = send_at - elapsed
                the_log.append((f"Point {i} planned_send={send_at:.3f}s elapsed={elapsed:.3f}s sleep_s={sleep_s:.3f}s dt_i={dt_i:.3f}s"))
                if sleep_s > 0.0:
                    self._spin_sleep(sleep_s)

                elapsed = time.perf_counter() - t0
                the_log.append((f"Point {i} planned_reach={planned_reach:.3f}s elapsed={elapsed:.3f}s"))
                t_cmd = dt_i

                jj = Positions[i]
                the_log.append((f"Point {i} t_cmd={t_cmd:.3f}s"))
                tsend = time.perf_counter()
                if not self.ServoJ_C(jj[0], jj[1], jj[2], jj[3], jj[4], jj[5], dt=t_cmd):
                    ok = False
                    break
                the_log.append((f"T_SEND {time.perf_counter() - tsend}"))
                the_log.append("")
        else:
            # Simpler strategy: send each point with a fixed dt. This is less smooth but more robust to timing issues.
            dt = 0.2
            # send_duration_s = 0.0
            for i in range(1, len(Positions)):
                t0 = time.perf_counter()
                jj = Positions[i]
                if not self.ServoJ_C(jj[0], jj[1], jj[2], jj[3], jj[4], jj[5], dt=dt):
                    ok = False
                    break
                elapsed = time.perf_counter() - t0
                # curr_send_duration_s = elapsed/2
                already_waited_s = elapsed - curr_send_duration_s
                sleep_s = dt - already_waited_s
                if sleep_s > 0.0:
                    self._spin_sleep(sleep_s)

        for line in the_log:
            self.get_logger().info(line)
        
        return ok

    def ServoJ_C(self, j1, j2, j3, j4, j5, j6, *, dt) -> bool:  # 运动指令
        P1 = ServoJ.Request()
        P1.a = float(j1)
        P1.b = float(j2)
        P1.c = float(j3)
        P1.d = float(j4)
        P1.e = float(j5)
        P1.f = float(j6)
        P1.param_value = [f"t={dt}"]
        future = self.ServoJ_l.call_async(P1)
        # return True
        if not self._spin_until_future(future, timeout_s=2.0):
            self.get_logger().error("ServoJ call timed out")
            return False
        resp = future.result()
        if resp is None:
            self.get_logger().error("ServoJ returned no response")
            return False
        if getattr(resp, "res", 0) != 0:
            self.get_logger().error(f"ServoJ failed, res={resp.res}")
            return False
        return True

def main(args=None):
    # Hardcode a dedicated ROS2 log directory so logs are easy to find in a container.
    # IMPORTANT: must be set before rclpy.init() so the ROS logging backend picks it up.
    os.environ["ROS_LOG_DIR"] = _ACTION_MOVE_SERVER_LOG_DIR
    try:
        os.makedirs(_ACTION_MOVE_SERVER_LOG_DIR, exist_ok=True)
    except Exception:
        # If we can't create it (permissions/readonly FS), ROS will fall back to its default.
        pass

    # Enable SCHED_FIFO early so the ROS spin loop runs under the requested policy.
    _maybe_enable_sched_fifo()

    rclpy.init(args=args)
    follow_joint_trajectory_server = FollowJointTrajectoryServer()
    rclpy.spin(follow_joint_trajectory_server)
    follow_joint_trajectory_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
