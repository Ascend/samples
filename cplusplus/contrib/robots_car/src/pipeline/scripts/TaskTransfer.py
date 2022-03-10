#!/usr/bin/python
"""
Copyright (c) Deep Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Haoyi Han <hanhaoyi@deeprobotics.cn>, Feb, 2020
"""

import rospy
import dynamic_reconfigure.client
import actionlib
from tf.transformations import *
from pipeline.msg import MoveBaseAction, MoveBaseGoal
from TaskPoint import TaskPoint
from RobotCommander import RobotCommander


class TaskTransfer:
    def __init__(self):
        # self.client_teb = dynamic_reconfigure.client.Client(
        #     "/move_base/TebLocalPlannerROS"
        # )
        self.client_global_costmap_static = dynamic_reconfigure.client.Client(
            "/move_base/global_costmap/static_layer"
        )
        # self.client_global_costmap_obstacle = dynamic_reconfigure.client.Client(
        #     "/move_base/global_costmap/obstacle_layer"
        # )
        self.client_local_costmap_static = dynamic_reconfigure.client.Client(
            "/move_base/local_costmap/static_layer"
        )
        self.client_local_costmap_obstacle = dynamic_reconfigure.client.Client(
            "/move_base/local_costmap/obstacle_layer"
        )
        self.moveBaseClient = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.moveBaseClient.wait_for_server()
        rospy.loginfo("Action 'move_base' is up!")

    def plan_failed(self):
        return self.moveBaseClient.get_state() == actionlib.GoalStatus.ABORTED

    def is_action_succeed(self):
        return self.moveBaseClient.get_state() == actionlib.GoalStatus.SUCCEEDED

    # def set_low_vel(self):
    #     slow_vel_params = {
    #         "max_vel_x": 0.1,
    #         "max_vel_x_backwards": 0.02,
    #         "max_vel_y": 0.08,
    #         "max_vel_theta": 0.2,
    #         "acc_lim_x": 0.08,
    #         "acc_lim_y": 0.03,
    #         "acc_lim_theta": 0.1,
    #     }
    #     self.client_teb.update_configuration(slow_vel_params)

    # def reset_vel(self):
    #     normal_vel_params = {
    #         "max_vel_x": 0.5,
    #         "max_vel_x_backwards": 0.101,
    #         "max_vel_y": 0.2,
    #         "max_vel_theta": 0.5,
    #         "acc_lim_x": 0.2,
    #         "acc_lim_y": 0.11,
    #         "acc_lim_theta": 0.3,
    #     }
    #     self.client_teb.update_configuration(normal_vel_params)

    def disable_costmap(self):
        params = {"enabled": False}
        # self.client_global_costmap_static.update_configuration(params)
        # self.client_global_costmap_obstacle.update_configuration(params)
        # self.client_global_costmap_static.update_configuration(params)
        self.client_local_costmap_obstacle.update_configuration(params)

    def enable_costmap(self):
        params = {"enabled": True}
        # self.client_global_costmap_static.update_configuration(params)
        # self.client_global_costmap_obstacle.update_configuration(params)
        # self.client_global_costmap_static.update_configuration(params)
        self.client_local_costmap_obstacle.update_configuration(params)

    def task_transfer(self, src_point, des_point):
        """
        Main Decision Function
        """
        with RobotCommander() as robot_commander:
            robot_commander.sendCordinate(
                command_code=51,
                x=src_point.getPosX(),
                y=src_point.getPosY(),
                yaw=src_point.getYaw(),
            )
            print ("-----------task_transfer-----------")
        des_point.setPreTaskPoint(src_point)

        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.stamp = rospy.Time.now()
        goal_msg.target_pose.header.frame_id = "map"
        goal_msg.target_pose.pose.position.x = des_point.getPosX()
        goal_msg.target_pose.pose.position.y = des_point.getPosY()
        goal_msg.target_pose.pose.position.z = 0
        my_q = quaternion_from_euler(0, 0, des_point.getYaw())
        goal_msg.target_pose.pose.orientation.x = my_q[0]
        goal_msg.target_pose.pose.orientation.y = my_q[1]
        goal_msg.target_pose.pose.orientation.z = my_q[2]
        goal_msg.target_pose.pose.orientation.w = my_q[3]

        print des_point.record["option"]

        not_done = True

        while not_done and not rospy.is_shutdown():
            """
            Do something REPEATEDLY
            """
            self.moveBaseClient.send_goal(goal_msg)
            rospy.logwarn(
                "Transfer from [%s] to [%s]" % (src_point.name, des_point.name)
            )
            rospy.sleep(5)

            done = self.moveBaseClient.wait_for_result(timeout=rospy.Duration(5.0))
            not_done = (not done) or (
                self.moveBaseClient.get_state() != actionlib.GoalStatus.SUCCEEDED
            )

        """
        Do something to finish the action, only ONCE
        """
        if not self.plan_failed() and des_point.order_equal_to(2):
            print "START TO UP STAIRS..."
            with RobotCommander() as robot_commander:
                robot_commander.up_stair_trait()
                self.disable_costmap()
                rospy.sleep(0.5)
        if not self.plan_failed() and des_point.order_equal_to(3):
            print "Finish UP STAIRS."
            with RobotCommander() as robot_commander:
                robot_commander.finish_up_stair_trait()
                self.enable_costmap()
                rospy.sleep(0.5)
        if not self.plan_failed() and des_point.order_equal_to(5):
            print "START TO UP STAIRS..."
            with RobotCommander() as robot_commander:
                robot_commander.up_stair_trait()
                self.disable_costmap()
                rospy.sleep(0.5)
        if not self.plan_failed() and des_point.order_equal_to(6):
            print "Finish UP STAIRS."
            with RobotCommander() as robot_commander:
                robot_commander.finish_up_stair_trait()
                self.enable_costmap()
                rospy.sleep(0.5)

