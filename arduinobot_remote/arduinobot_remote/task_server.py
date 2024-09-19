#!/usr/bin/env python3

import rclpy                            # Import core ROS2 Python library
from rclpy.node import Node             # Import Node class for creating nodes
from rclpy.action import ActionServer    # Import ActionServer for handling actions
from rclpy.action import GoalResponse, CancelResponse  # Import responses for goals and cancellations
from moveit_commander import MoveGroupCommander  # Import MoveGroupCommander for controlling robot movements
from moveit_commander.robot_trajectory import RobotTrajectory
from threading import Thread            # Import Thread to handle asynchronous execution
from arduinobot_msgs.action import ArduinobotTask  # Import custom ArduinobotTask action

class TaskServer(Node):
    """
    A class to define the TaskServer node that will handle action requests 
    for controlling a robotic arm and gripper via MoveIt2.
    """
    def __init__(self):
        # Initialize the node with the name 'task_server'
        super().__init__('task_server')

        self.get_logger().info('Starting the Server')

        # Create an Action Server for ArduinobotTask
        self.action_server = ActionServer(
            self,
            ArduinobotTask,
            'task_server',
            self.goal_callback,
            self.cancel_callback,
            self.accepted_callback
        )

    def goal_callback(self, goal_handle):
        """
        Called when a new goal is received. It checks the validity of the goal.
        """
        self.get_logger().info(f"Received goal request with task number {goal_handle.request.task_number}")

        # Accept and execute the goal
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Called when a cancellation request is received. Stops the arm and gripper.
        """
        self.get_logger().info("Received request to cancel goal")

        # Stop the arm and gripper by invoking MoveGroupCommander
        arm_move_group = MoveGroupCommander("arm")
        gripper_move_group = MoveGroupCommander("gripper")
        arm_move_group.stop()
        gripper_move_group.stop()

        return CancelResponse.ACCEPT

    def accepted_callback(self, goal_handle):
        """
        Called when a goal is accepted. Spawns a new thread for executing the task to avoid blocking.
        """
        # Start a new thread to execute the task
        Thread(target=self.execute, args=(goal_handle,)).start()

    def execute(self, goal_handle):
        """
        Main function to execute the action goal. This handles arm and gripper movements
        based on the task number received in the goal.
        """
        self.get_logger().info('Executing goal')

        # Initialize MoveIt2 interface for controlling arm and gripper
        arm_move_group = MoveGroupCommander("arm")
        gripper_move_group = MoveGroupCommander("gripper")

        arm_joint_goal = []
        gripper_joint_goal = []

        # Set different arm and gripper goals based on the task number
        if goal_handle.request.task_number == 0:
            arm_joint_goal = [0.0, 0.0, 0.0]
            gripper_joint_goal = [-0.7, 0.7]
        elif goal_handle.request.task_number == 1:
            arm_joint_goal = [-1.14, -0.6, -0.07]
            gripper_joint_goal = [0.0, 0.0]
        elif goal_handle.request.task_number == 2:
            arm_joint_goal = [-1.57, 0.0, -0.9]
            gripper_joint_goal = [0.0, 0.0]
        else:
            # If task number is invalid, log an error and return
            self.get_logger().error('Invalid Task Number')
            return

        # Set target joint positions and check if they are within bounds
        arm_within_bounds = arm_move_group.set_joint_value_target(arm_joint_goal)
        gripper_within_bounds = gripper_move_group.set_joint_value_target(gripper_joint_goal)

        if not arm_within_bounds or not gripper_within_bounds:
            self.get_logger().warn("Target joint position(s) are outside of limits. Clamping to limits.")
            return

        # Plan motion for both the arm and gripper
        arm_plan = arm_move_group.plan()
        gripper_plan = gripper_move_group.plan()

        # Execute the planned motion if the planning is successful
        if arm_plan and gripper_plan:
            self.get_logger().info("Planner SUCCEEDED, moving the arm and gripper")
            arm_move_group.go(wait=True)
            gripper_move_group.go(wait=True)
        else:
            self.get_logger().error("One or more planners failed!")
            return

        # Mark the goal as successful
        goal_handle.succeed()
        result = ArduinobotTask.Result()
        result.success = True
        self.get_logger().info('Goal succeeded')

def main(args=None):
    """
    Main function to initialize the ROS2 node and spin it to keep it alive.
    """
    rclpy.init(args=args)
    task_server = TaskServer()
    rclpy.spin(task_server)  # Keep the node alive and listening for goals

    task_server.destroy_node()  # Cleanup
    rclpy.shutdown()  # Shutdown the ROS2 system


if __name__ == '__main__':
    main()
