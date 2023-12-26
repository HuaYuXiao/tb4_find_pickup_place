from utils import *


def rclpySpin(controller):
    rclpy.spin(controller)


def main():
    rclpy.init(args=None)


    ### 初始状态 ###
    place = Place()
    place_thread = Thread(target=rclpySpin, args=(place,))
    place_thread.start()
    place_thread.join()


    # 创建导航器并等待其激活
    # navigator = BasicNavigator()
    # navigator.waitUntilNav2Active()


    ## 前往S点 ###
    # navigator.goToPose(PointS)
    # while not navigator.isTaskComplete():
    #     feedback = navigator.getFeedback()
    #     if feedback:
    #         # Some navigation timeout to demo cancellation
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
    #             navigator.cancelTask()
    # # Do something depending on the return code
    # result = navigator.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     print('Goal reached')
    # elif result == TaskResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == TaskResult.FAILED:
    #     print('Goal failed!')
    # else:
    #     print('Goal has an invalid return status!')


    ### 前往A点 ###
    # navigator.goToPose(PointA)
    # while not navigator.isTaskComplete():
    #     feedback = navigator.getFeedback()
    #     if feedback:
    #         # Some navigation timeout to demo cancellation
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
    #             navigator.cancelTask()
    # # Do something depending on the return code
    # result = navigator.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     print('Goal reached')
    # elif result == TaskResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == TaskResult.FAILED:
    #     print('Goal failed!')
    # else:
    #     print('Goal has an invalid return status!')


    spin = Spin()
    spin_thread = Thread(target=rclpySpin, args=(spin,))
    spin_thread.start()
    spin_thread.join()



    # move = Move()
    # move_thread = Thread(target=rclpySpin, args=(move,))
    # move_thread.start()
    # move_thread.join()


    # global start_time
    # start_time = datetime.datetime.now()
    # pickup = Pickup()
    # pickup_thread = Thread(target=rclpySpin, args=(pickup,))
    # pickup_thread.start()
    # pickup_thread.join()


    ### 前往B点 ###
    # navigator.goToPose(PointB)
    # while not navigator.isTaskComplete():
    #     feedback = navigator.getFeedback()
    #     if feedback:
    #         # Some navigation timeout to demo cancellation
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
    #             navigator.cancelTask()
    # # Do something depending on the return code
    # result = navigator.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     print('Goal reached')
    # elif result == TaskResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == TaskResult.FAILED:
    #     print('Goal failed!')
    # else:
    #     print('Goal has an invalid return status!')


    # place = Place()
    # place_thread = Thread(target=rclpySpin, args=(place,))
    # place_thread.start()
    # place_thread.join()


    ### 前往S点 ###
    # navigator.goToPose(PointS)
    # while not navigator.isTaskComplete():
    #     feedback = navigator.getFeedback()
    #     if feedback:
    #         # Some navigation timeout to demo cancellation
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
    #             navigator.cancelTask()
    # # Do something depending on the return code
    # result = navigator.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     print('Goal reached')
    # elif result == TaskResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == TaskResult.FAILED:
    #     print('Goal failed!')
    # else:
    #     print('Goal has an invalid return status!')


if __name__ == '__main__':
    main()
