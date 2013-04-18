#!/usr/bin/env python
# -*- coding: utf -*-

import roslib
roslib.load_manifest('amr_ui')

import sys
import math
from PySide.QtGui import *
from PySide.QtCore import *

import rospy
import actionlib
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import PoseStamped


class Form(QDialog):

    def __init__(self, pub, parent=None):
        super(Form, self).__init__(parent)
        self.pub = pub
        # General layout
        new_group = QGroupBox('New goal')
        current_group = QGroupBox('Current pose')
        layout = QHBoxLayout()
        layout.addWidget(new_group)
        layout.addWidget(current_group)
        self.setLayout(layout)
        # Left side -- new goal group
        grid = QGridLayout()
        grid.addWidget(QLabel('X:'), 0, 0)
        self.x_spin = QDoubleSpinBox()
        self.x_spin.setRange(-10, 10)
        self.x_spin.setSingleStep(0.1)
        grid.addWidget(self.x_spin, 0, 1)
        grid.addWidget(QLabel('Y:'), 1, 0)
        self.y_spin = QDoubleSpinBox()
        self.y_spin.setRange(-10, 10)
        self.y_spin.setSingleStep(0.1)
        grid.addWidget(self.y_spin, 1, 1)
        grid.addWidget(QLabel('Yaw:'), 2, 0)
        self.yaw_spin = QSpinBox()
        self.yaw_spin.setRange(0, 360)
        grid.addWidget(self.yaw_spin, 2, 1)
        send_button = QPushButton('Send')
        send_button.clicked.connect(self.button_send_cb)
        grid.addWidget(send_button, 3, 0, 1, 2)
        new_group.setLayout(grid)
        # Right side -- current goal group
        grid = QGridLayout()
        grid.addWidget(QLabel('State:'), 0, 0)
        self.state_text = QLabel('<i>unknown</i>')
        grid.addWidget(self.state_text, 0, 1)
        grid.addWidget(QLabel('Target:'), 1, 0)
        self.target_text = QLabel()
        grid.addWidget(self.target_text, 1, 1)
        current_group.setLayout(grid)

    def update_state_text(self, target=None):
        state = ['pending', 'active', 'preempted', 'succeeded', 'aborted',
                 'rejected', 'preempting', 'recalling', 'recalled',
                 'lost'][self.client.get_state()]
        self.state_text.setText('<i>%s</i>' % state)
        if target is not None:
            self.target_text.setText('<i>%s</i>' % target)

    def button_send_cb(self):
        x = self.x_spin.value()
        y = self.y_spin.value()
        yaw = self.yaw_spin.value()
        goal = PoseStamped()
        q = quaternion_from_euler(0, 0, yaw * math.pi / 180)
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        goal.header.frame_id = "odom"
        self.pub.publish(goal)
#        goal_text = u'%.2f; %.2f; %i°' % (x, y, yaw)
#        self.update_state_text(goal_text)

if __name__ == '__main__':
    rospy.init_node('pub_goal_gui')
    pub = rospy.Publisher('goal', PoseStamped)
    app = QApplication(sys.argv)
    gui = Form(pub)
    gui.show()
    rospy.on_shutdown(lambda: app.exit())
    # Qt + Python hack: this timer will allow the interpreter to run each 500
    # ms and at some point in time receive the shutdown callback from ROS.
    timer = QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)
    # Start application execution
    sys.exit(app.exec_())
