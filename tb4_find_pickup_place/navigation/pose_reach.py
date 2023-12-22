from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration


def main():
    rclpy.init()

    navigatorS = BasicNavigator()
    navigatorS.waitUntilNav2Active()
    goal_poseS = PoseStamped()
    goal_poseS.header.frame_id = 'map'
    goal_poseS.header.stamp = navigatorS.get_clock().now().to_msg()

    goal_poseS.pose.position.x = 2.7
    goal_poseS.pose.position.y = -3.2
    goal_poseS.pose.orientation.z = 0.0
    goal_poseS.pose.orientation.w = 1.0

    navigatorS.goToPose(goal_poseS)
    print('Reaching for S')

    i = 0
    while not navigatorS.isTaskComplete():
        i = i + 1
        feedback = navigatorS.getFeedback()
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
                navigatorS.cancelTask()

    # Do something depending on the return code
    resultS = navigatorS.getResult()
    print(resultS)
    if resultS == TaskResult.SUCCEEDED:
        print('S reached')
    elif resultS == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif resultS == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigatorS.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
