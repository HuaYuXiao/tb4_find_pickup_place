from define import *


def main():
    qA = quaternion_from_euler(0.0, 0.0, np.deg2rad(-90.0))
    PointA = PoseStamped(
        header=Header(frame_id='map'),
        pose=Pose(
            position=Point(x=0.25, y=-3.3, z=0.0),
            orientation=Quaternion(x=qA[0], y=qA[1], z=qA[2], w=qA[3])
        )
    )

    qB = quaternion_from_euler(0.0, 0.0, np.deg2rad(0.0))
    PointB = PoseStamped(
        header=Header(frame_id='map'),
        pose=Pose(
            position=Point(x=2.85, y=-3.45, z=0.0),
            orientation=Quaternion(x=qB[0], y=qB[1], z=qB[2], w=qB[3])
        )
    )

    qS = quaternion_from_euler(0.0, 0.0, np.deg2rad(-90.0))  
    PointS = PoseStamped(
        header=Header(frame_id='map'),
        pose=Pose(
            position=Point(x=0.25, y=-0.3, z=0.0),
            orientation=Quaternion(x=qS[0], y=qS[1], z=qS[2], w=qS[3])
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
    pickup = Thread(target=arm_thread, args=(controller, True))
    pickup.start()
    # 等待导航线程完成
    pickup.join()


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
