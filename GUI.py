# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'GUI.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(720, 650)
        self.label = QtWidgets.QLabel(Form)
        self.label.setGeometry(QtCore.QRect(40, 10, 640, 480))
        self.label.setObjectName("label")
        self.layoutWidget = QtWidgets.QWidget(Form)
        self.layoutWidget.setGeometry(QtCore.QRect(20, 510, 681, 131))
        self.layoutWidget.setObjectName("layoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.layoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.groupBox_2 = QtWidgets.QGroupBox(self.layoutWidget)
        self.groupBox_2.setObjectName("groupBox_2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.groupBox_2)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.Pause = QtWidgets.QPushButton(self.groupBox_2)
        self.Pause.setObjectName("Pause")
        self.gridLayout_2.addWidget(self.Pause, 0, 1, 1, 1)
        self.Reset = QtWidgets.QPushButton(self.groupBox_2)
        self.Reset.setObjectName("Reset")
        self.gridLayout_2.addWidget(self.Reset, 0, 2, 1, 1)
        self.Start = QtWidgets.QPushButton(self.groupBox_2)
        self.Start.setObjectName("Start")
        self.gridLayout_2.addWidget(self.Start, 0, 0, 1, 1)
        self.Exit = QtWidgets.QPushButton(self.groupBox_2)
        self.Exit.setObjectName("Exit")
        self.gridLayout_2.addWidget(self.Exit, 0, 3, 1, 1)
        self.horizontalLayout.addWidget(self.groupBox_2)
        self.groupBox = QtWidgets.QGroupBox(self.layoutWidget)
        self.groupBox.setObjectName("groupBox")
        self.gridLayout = QtWidgets.QGridLayout(self.groupBox)
        self.gridLayout.setObjectName("gridLayout")
        self.label_4 = QtWidgets.QLabel(self.groupBox)
        self.label_4.setObjectName("label_4")
        self.gridLayout.addWidget(self.label_4, 0, 0, 1, 1)
        self.label_5 = QtWidgets.QLabel(self.groupBox)
        self.label_5.setObjectName("label_5")
        self.gridLayout.addWidget(self.label_5, 1, 0, 1, 1)
        self.display_angle = QtWidgets.QLabel(self.groupBox)
        self.display_angle.setObjectName("display_angle")
        self.gridLayout.addWidget(self.display_angle, 1, 1, 1, 1)
        self.display_distance = QtWidgets.QLabel(self.groupBox)
        self.display_distance.setObjectName("display_distance")
        self.gridLayout.addWidget(self.display_distance, 0, 1, 1, 1)
        self.horizontalLayout.addWidget(self.groupBox)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.label.setText(_translate("Form", "TextLabel"))
        self.groupBox_2.setTitle(_translate("Form", "Control"))
        self.Pause.setText(_translate("Form", "Pause"))
        self.Reset.setText(_translate("Form", "Reset"))
        self.Start.setText(_translate("Form", "Start"))
        self.Exit.setText(_translate("Form", "Exit"))
        self.groupBox.setTitle(_translate("Form", "Object"))
        self.label_4.setText(_translate("Form", "Distance:"))
        self.label_5.setText(_translate("Form", "Angle:"))
        self.display_angle.setText(_translate("Form", "TextLabel"))
        self.display_distance.setText(_translate("Form", "TextLabel"))




