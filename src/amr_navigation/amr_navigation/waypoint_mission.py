#!/usr/bin/env python3

import math

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


WAYPOINTS = [
    ('Pickup', 1.526, 4.774, 0.0),
    ('Delivery', 1.493, -3.055, 0.0),
    ('Home', 7.283, 0.797, 0.0),
]


def create_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)

    return pose


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()

    try:
        navigator.get_logger().info('Waiting for Nav2 to become active...')
        navigator.waitUntilNav2Active()

        navigator.get_logger().info('Nav2 is active')
        navigator.get_logger().info(
            'Mission: Pickup -> Delivery -> Home'
        )

        navigator.clearAllCostmaps()

        goal_poses = [
            create_pose(navigator, x, y, yaw)
            for _, x, y, yaw in WAYPOINTS
        ]

        navigator.followWaypoints(goal_poses)

        last_waypoint = -1

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()

            if feedback is not None:
                current = feedback.current_waypoint

                if current != last_waypoint and current < len(WAYPOINTS):
                    name = WAYPOINTS[current][0]
                    navigator.get_logger().info(
                        f'Navigating to waypoint '
                        f'{current + 1}/{len(WAYPOINTS)}: {name}'
                    )
                    last_waypoint = current

        result = navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info(
                'Mission completed successfully'
            )
        elif result == TaskResult.CANCELED:
            navigator.get_logger().warn('Mission was canceled')
        elif result == TaskResult.FAILED:
            navigator.get_logger().error('Mission failed')
        else:
            navigator.get_logger().error(
                f'Unknown mission result: {result}'
            )

    except KeyboardInterrupt:
        navigator.get_logger().warn(
            'Mission canceled by keyboard interrupt'
        )
        navigator.cancelTask()

    except Exception as error:
        navigator.get_logger().error(
            f'Mission error: {error}'
        )
        navigator.cancelTask()

    finally:
        navigator.destroyNode()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
