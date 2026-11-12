import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from arm05_search_interfaces.action import FindCubes
from arm05_search_interfaces.msg import CubeArray
from arm05_search_interfaces.srv import FinalizeCubes


POINTS = [
    (-0.895, 0.715),
    (-0.045, 2.865),
    (-4.595, 3.115),
    (-4.645, -0.235),
    (-4.695, -3.035),
    (-3.595, -5.035),
    (-0.745, -4.985),
    (0.505, -3.035),
    (0.705, -1.085),
    (2.905, -0.185),
    (3.105, 2.915),
    (5.005, 4.765),
    (8.655, 4.965),
    (10.805, 3.315),
    (12.855, 3.315),
    (11.905, 0.115),
    (14.355, 0.315),
    (16.655, 2.015),
    (17.055, -0.785),
    (15.905, -2.485),
    (12.005, -2.285),
    (8.255, -2.535),
    (6.605, -4.885),
    (9.855, -4.635),
    (12.655, -5.335),
    (5.105, -2.485),
    (3.455, -4.985),
]

EXPECTED_IDS = {0, 1, 2, 3, 4}
SCAN_STEPS = 8
DWELL_SEC = 0.5


class SearchManager(Node):
    def __init__(self):
        super().__init__('search_manager')

        self.found_ids = []
        self.busy = False
        self.motion = None

        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.spin = ActionClient(self, Spin, '/spin')
        self.reset = self.create_client(Trigger, '/reset_cubes')
        self.finalize = self.create_client(FinalizeCubes, '/finalize_cubes')
        self.create_subscription(CubeArray, '/detected_cubes', self.update_ids, 10)

        self.action = ActionServer(
            self, FindCubes, '/find_cubes', self.search,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.accept,
            cancel_callback=self.cancel,
        )
        self.get_logger().info(f'Loaded {len(POINTS)} points.')

    def update_ids(self, msg):
        self.found_ids = [cube.id for cube in msg.cubes]

    def accept(self, _request):
        if self.busy:
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    def cancel(self, _goal):
        if self.motion is not None:
            self.motion.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def move(self, goal, client, command):
        handle = await client.send_goal_async(command)
        if not handle.accepted:
            return False

        self.motion = handle
        if goal.is_cancel_requested:
            handle.cancel_goal_async()

        result = await handle.get_result_async()
        self.motion = None
        return result.status == GoalStatus.STATUS_SUCCEEDED

    async def scan(self, goal):
        turn = Spin.Goal(
            target_yaw=math.tau,
            time_allowance=Duration(sec=20),
        )
        return await self.move(goal, self.spin, turn)

    async def search(self, goal):
        result = FindCubes.Result(total_points=len(POINTS))
        self.found_ids = []
        await self.reset.call_async(Trigger.Request())

        for index, (x, y) in enumerate(POINTS, start=1):
            if goal.is_cancel_requested:
                break

            self.get_logger().info(f'Punkt {index}/{len(POINTS)} ({x:.2f}, {y:.2f}) m.')
            target = NavigateToPose.Goal()
            target.pose.header.frame_id = 'map'
            target.pose.header.stamp = self.get_clock().now().to_msg()
            target.pose.pose.position.x = x
            target.pose.pose.position.y = y
            target.pose.pose.orientation.w = 1.0

            if not await self.move(goal, self.nav, target):
                continue

            scanned = await self.scan(goal)
            if scanned:
                result.scanned_points += 1

            goal.publish_feedback(FindCubes.Feedback(
                waypoint_index=index,
                total_points=len(POINTS),
                found_ids=self.found_ids,
                phase='scanned' if scanned else 'scan_incomplete',
            ))

        response = await self.finalize.call_async(FinalizeCubes.Request())
        result.result = response.result
        missing = sorted(EXPECTED_IDS - {cube.id for cube in result.result.cubes})

        if goal.is_cancel_requested:
            result.message = 'Canceled.'
            goal.canceled()
        elif missing:
            result.message = f'Finished. Missing: {missing}.'
            goal.abort()
        else:
            result.message = 'All cubes found!!!'
            goal.succeed()

        self.busy = False
        return result


def main():
    rclpy.init()
    node = SearchManager()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()