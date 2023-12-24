from define import *


def main():
    PointS = PoseStamped(
        header=Header(frame_id='map'),
        pose=Pose(
            position=Point(x=0.25, y=-0.3, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
    )

    PointA = PoseStamped(
        header=Header(frame_id='map'),
        pose=Pose(
            position=Point(x=0.2, y=-3.2, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
    )

    PointB = PoseStamped(
        header=Header(frame_id='map'),
        pose=Pose(
            position=Point(x=2.85, y=-3.25, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
    )


    rclpy.init(args=None)
    # 创建导航器并等待其激活
    navigator = BasicNavigator()
    controller = ArmController()

    ### 前往S点 ###     
    navS = Thread(target=nav_thread, args=(navigator, PointS))
    navS.start()
    # 等待导航线程完成
    navS.join()


    ### 前往A点 ###
    navA = Thread(target=nav_thread, args=(navigator, PointA))
    navA.start()
    # 等待导航线程完成
    navA.join()


    ### 抓取物块 ###
    # pickup = Thread(target=arm_thread, args=(controller, True))
    # pickup.start()
    # # 等待导航线程完成
    # pickup.join()


    ### 前往B点 ###
    navB = Thread(target=nav_thread, args=(navigator, PointB))
    navB.start()
    # 等待导航线程完成
    navB.join()


    ### 放下物块 ###
    place = Thread(target=arm_thread, args=(controller, False))
    place.start()
    # 等待导航线程完成
    place.join()


    ### 前往S点 ###
    navS = Thread(target=nav_thread, args=(navigator, PointS))
    navS.start()
    # 等待导航线程完成
    navS.join()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
