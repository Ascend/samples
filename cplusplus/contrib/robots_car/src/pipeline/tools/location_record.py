#!/usr/bin/python
# coding: utf-8
# author: jsnjhhy@126.com

import sys
import rospy
from PyQt5.QtWidgets import (
    QWidget,
    QApplication,
    QLabel,
    QHBoxLayout,
    QVBoxLayout,
    QPushButton,
    QLineEdit,
    QTextEdit,
    QCheckBox,
)

import json
import os
import tf


class LocationRecorder(QWidget):
    def __init__(self):
        super(LocationRecorder, self).__init__()
        self.init_ui()
        self.robot_record_pose = None
        self.option = None
        self.tf_listener = tf.TransformListener()
        self.show()

    def init_ui(self):
        self.layout = QVBoxLayout()

        self.option_layout = QHBoxLayout()
        self.up_stair_checkbox = QCheckBox("上台阶")
        self.down_stair_checkbox = QCheckBox("下台阶")
        self.crawl_checkbox = QCheckBox("匍匐")
        self.speed_up_checkbox = QCheckBox("加速")

        self.option_layout.addWidget(self.up_stair_checkbox)
        self.option_layout.addWidget(self.down_stair_checkbox)
        self.option_layout.addWidget(self.speed_up_checkbox)
        self.option_layout.addWidget(self.crawl_checkbox)

        self.order_layout = QHBoxLayout()
        self.order_layout.addWidget(QLabel("位点编号:"))
        self.order_edit = QLineEdit("")
        self.order_layout.addWidget(self.order_edit)

        self.text_content = QTextEdit()
        self.text_content.setEnabled(False)

        self.record_layout = QHBoxLayout()
        self.receive_button = QPushButton("获取位点")
        self.record_button = QPushButton("记录位点")
        self.record_layout.addWidget(self.receive_button)
        self.record_layout.addWidget(self.record_button)

        self.layout.addLayout(self.option_layout)
        self.layout.addLayout(self.order_layout)
        self.layout.addWidget(self.text_content)
        self.layout.addLayout(self.record_layout)
        self.setLayout(self.layout)

        self.record_button.clicked.connect(self.record)
        self.receive_button.clicked.connect(self.receive)

    def record(self):
        order = self.order_edit.text()
        try:
            order = int(order)
        except:
            return
        new_record = {
            "order": order,
            "robot_pose": self.robot_record_pose,
            "option": self.option,
        }
        data_dir = str(os.path.dirname(os.path.abspath(__file__))) + "/../data"
        os.system("mkdir -p " + data_dir)
        data_dir = data_dir + "/%d.json"
        with open(data_dir % order, "w+") as out:
            json.dump(new_record, out, indent=4)

    def listen_tf(self):
        try:
            (pos, ori) = self.tf_listener.lookupTransform(
                "map", "base_link", rospy.Duration(0.0)
            )
            msg_dict = {
                "pos_x": pos[0],
                "pos_y": pos[1],
                "pos_z": pos[2],
                "ori_x": ori[0],
                "ori_y": ori[1],
                "ori_z": ori[2],
                "ori_w": ori[3],
            }
            self.robot_record_pose = msg_dict
            return True
        except tf.Exception as e:
            print "listen to tf failed"
            return False

    def update_option(self):
        self.option = {}
        self.option["up_stair"] = self.up_stair_checkbox.isChecked()
        self.option["down_stair"] = self.down_stair_checkbox.isChecked()
        self.option["speed_up"] = self.speed_up_checkbox.isChecked()
        self.option["crawl"] = self.crawl_checkbox.isChecked()

    def receive(self):
        while not self.listen_tf():
            rospy.sleep(1.0)
        self.update_option()
        display_msg = "Robot:\n" + json.dumps(self.robot_record_pose, indent=4) + "\n"
        display_msg += "Option:\n" + json.dumps(self.option, indent=4) + "\n"
        self.text_content.setText(display_msg)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    rospy.init_node("location_recorder")
    lr = LocationRecorder()
    sys.exit(app.exec_())
