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

    def execution_trajectory(self, trajectory: JointTrajectory) -> bool:
        self.get_logger().info("Joint Names: {}".format(trajectory.joint_names))
        Positions = []
        for i, point in enumerate(trajectory.points):
            joint= []
            for ii in point.positions:
                joint.append(180 * ii / 3.14159)
            Positions.append(joint)
            self.get_logger().info(
                "Point {}: Positions: {}, Velocities: {}, Accelerations: {}, TimeFromStart: {}".format(
                    i, joint, point.velocities, point.accelerations, point.time_from_start.sec
                )
            )
        ok = True
        for ii in Positions:
            if not self.ServoJ_C(ii[0],ii[1],ii[2],ii[3],ii[4],ii[5]):
                ok = False
                break
            time.sleep(0.18)
        return ok

    def ServoJ_C(self, j1, j2, j3, j4, j5, j6) -> bool:  # 运动指令
        P1 = ServoJ.Request()
        P1.a = float(j1)
        P1.b = float(j2)
        P1.c = float(j3)
        P1.d = float(j4)
        P1.e = float(j5)
        P1.f = float(j6)
        P1.param_value = ["t=0.2"]
        future = self.ServoJ_l.call_async(P1)
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
    rclpy.init(args=args)
    follow_joint_trajectory_server = FollowJointTrajectoryServer()
    rclpy.spin(follow_joint_trajectory_server)
    follow_joint_trajectory_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()