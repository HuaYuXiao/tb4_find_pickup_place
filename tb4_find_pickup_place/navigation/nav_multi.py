from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration


def nav_single(navigator, goal_pose, point):
    goal_pose.pose.position.x = point[0]
    goal_pose.pose.position.y = point[1]
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    navigator.goToPose(goal_pose)
    print('Reaching for S')

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

    # Do something depending on the return code
    resultS = navigator.getResult()
    print(resultS)
    if resultS == TaskResult.SUCCEEDED:
        print('S reached')
    elif resultS == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif resultS == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')


if __name__ == '__main__':
    rclpy.init()

    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()


    nav_single(navigator, goal_pose, [0.27, -0.38])
    nav_single(navigator, goal_pose, [0.32, -3.0])
    # nav_single(navigator, goal_pose, [0.32, -2.0])
    nav_single(navigator, goal_pose, [2.7, -3.2])
    nav_single(navigator, goal_pose, [0.27, -0.38])



    # navigator.lifecycleShutdown()

    exit(0)
