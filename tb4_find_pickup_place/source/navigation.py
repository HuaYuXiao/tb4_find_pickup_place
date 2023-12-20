#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose


class SimpleGoalClient(Node):
    def __init__(self):
        super().__init__('simple_goal')

        # 创建导航客户端
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 等待导航服务器连接
        self._action_client.wait_for_server()
        self.get_logger().info('Connected to navigate_to_pose server')

        # 设置目标点
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 2.0
        goal_msg.pose.pose.position.y = 1.0
        goal_msg.pose.pose.orientation.w = 1.0

        # 发送目标点
        self.get_logger().info('Sending goal')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        # 设置定时器检查结果
        self.create_timer(0.1, self._check_goal_status)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Received feedback: {0}'.format(feedback_msg))

    def _check_goal_status(self):
        # 检查目标是否完成
        if self._send_goal_future.done():
            goal_handle = self._send_goal_future.result()
            if goal_handle.accepted:
                self.get_logger().info('Goal accepted')
            else:
                self.get_logger().info('Goal rejected')
            if goal_handle.status == goal_handle.Status.SUCCEEDED:
                self.get_logger().info('Goal succeeded!')
            else:
                self.get_logger().info('Goal failed!')


def main(args=None):
    rclpy.init(args=args)

    try:
        simple_goal_client = SimpleGoalClient()
        rclpy.spin(simple_goal_client)
    except KeyboardInterrupt:
        pass
    finally:
        simple_goal_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
