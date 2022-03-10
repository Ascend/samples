#!/usr/bin/python
"""
Copyright (c) Deep Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Haoyi Han <hanhaoyi@deeprobotics.cn>, Feb, 2020
"""
import time
from tf import transformations
from RobotCommander import RobotCommander
import rospy

import tf
import copy


class TaskPoint:
    """Property and method about a task point.

    Attributes:
        pose: the pose of robot in this wappoint.
        name: waypoint name.
    """

    def __init__(self, record=None):
        if not record:
            record = {
                "order": 0,
                "robot_pose": {
                    "pos_x": 0.0,
                    "pos_y": 0.0,
                    "pos_z": 0.0,
                    "ori_x": 0.0,
                    "ori_y": 0.0,
                    "ori_z": 0.0,
                    "ori_w": 1.0,
                },
                "option": {
                    "up_stair": False,
                    "down_stair": False,
                    "speed_up": False,
                    "crawl": False,
                },
            }
        self.pre_task_point = None
        self.record = copy.deepcopy(record)
        self.tf_listener = tf.TransformListener()
        self.update()

    def setPreTaskPoint(self, src_point):
        self.pre_task_point = src_point

    def getPreTaskPoint(self):
        return self.pre_task_point.record["order"]

    def order_equal_to(self, num):
        return self.record["order"] == num

    def is_up_stair(self):
        return self.record["option"]["up_stair"]

    def is_down_stair(self):
        return self.record["option"]["down_stair"]

    def is_speed_up(self):
        return self.record["option"]["speed_up"]

    def is_crawl(self):
        return self.record["option"]["crawl"]

    def update(self):
        self.robot_pose = self.record["robot_pose"]
        pose = []
        pose.append(self.robot_pose["pos_x"])
        pose.append(self.robot_pose["pos_y"])
        pose.append(self.robot_pose["pos_z"])
        pose.append(self.robot_pose["ori_x"])
        pose.append(self.robot_pose["ori_y"])
        pose.append(self.robot_pose["ori_z"])
        pose.append(self.robot_pose["ori_w"])
        self.posX = pose[0]
        self.posY = pose[1]
        self.yaw = transformations.euler_from_quaternion(pose[3:])[2]
        self.name = "waypoint_" + str(self.record["order"])

    def setRobotPose(self, robot_pose):
        self.record["robot_pose"]["pos_x"] = robot_pose[0]
        self.record["robot_pose"]["pos_y"] = robot_pose[1]
        self.record["robot_pose"]["pos_z"] = robot_pose[2]
        self.record["robot_pose"]["ori_x"] = robot_pose[3]
        self.record["robot_pose"]["ori_y"] = robot_pose[4]
        self.record["robot_pose"]["ori_z"] = robot_pose[5]
        self.record["robot_pose"]["ori_w"] = robot_pose[6]
        self.update()

    def getPosX(self):
        return self.posX

    def getPosY(self):
        return self.posY

    def getYaw(self):
        return self.yaw

    def calDistance(self, other):
        return (
            (self.getPosX() - other.getPosX()) ** 2
            + (self.getPosY() - other.getPosY()) ** 2
        ) ** 0.5


def globalTaskPrepare():
    with RobotCommander() as robot_commander:
        robot_commander.stand_down_up()
        rospy.sleep(5.0)
        robot_commander.start_force_mode()
        rospy.sleep(2.0)
        robot_commander.motion_start_stop()
        rospy.sleep(2.0)


def globalTaskFinish():
    with RobotCommander() as robot_commander:
        robot_commander.motion_start_stop()
        rospy.sleep(2.0)
        robot_commander.stand_down_up()
        rospy.sleep(5.0)
