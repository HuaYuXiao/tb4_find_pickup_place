from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

"""
Basic navigation demo to go to pose.
"""


def main():
    rclpy.init()

    navigatorS = BasicNavigator()
    navigatorS.waitUntilNav2Active()
    goal_poseS = PoseStamped()
    goal_poseS.header.frame_id = 'map'
    goal_poseS.header.stamp = navigatorS.get_clock().now().to_msg()

    goal_poseS.pose.position.x = 0.27
    goal_poseS.pose.position.y = -0.38
    goal_poseS.pose.orientation.z = 0.0
    goal_poseS.pose.orientation.w = 1.0

    navigatorS.goToPose(goal_poseS)
    print('Reaching for S')

    # Do something depending on the return code
    resultS = navigatorS.getResult()
    if resultS == TaskResult.SUCCEEDED:
        print('S reached')
    elif resultS == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif resultS == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigatorS.lifecycleShutdown()




    navigatorA = BasicNavigator()
    navigatorA.waitUntilNav2Active()
    goal_poseA = PoseStamped()
    goal_poseA.header.frame_id = 'map'
    goal_poseA.header.stamp = navigatorA.get_clock().now().to_msg()

    goal_poseA.pose.position.x = 0.32
    goal_poseA.pose.position.y = -0.3
    goal_poseA.pose.orientation.z = 0.0
    goal_poseA.pose.orientation.w = 1.0

    navigatorA.goToPose(goal_poseA)
    print('Reaching for A')

    resultA = navigatorA.getResult()
    if resultA == TaskResult.SUCCEEDED:
        print('A reached')
    elif resultA == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif resultA == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')




    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
