#!/usr/bin/env python3
import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock()

    # Set initial pose
    # S point
    initial_pose = navigator.getPoseStamped([0.27, -0.38], TurtleBot4Directions.SOUTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = []
    # A point
    goal_pose.append(navigator.getPoseStamped([0.32, -3.0], TurtleBot4Directions.EAST))
    # B point
    goal_pose.append(navigator.getPoseStamped([2.7, -3.2], TurtleBot4Directions.NORTH_WEST))
    # S point
    goal_pose.append(navigator.getPoseStamped([0.27, -0.38], TurtleBot4Directions.NORTH_WEST))

    # Undock
    navigator.undock()

    # Navigate through poses
    navigator.startThroughPoses(goal_pose)

    # Finished navigating, dock
    navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()