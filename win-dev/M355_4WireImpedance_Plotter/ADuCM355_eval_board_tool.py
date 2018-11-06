# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ADuCM355_eval_board_tool.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import pyqtgraph as pg
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import (
        FigureCanvas, NavigationToolbar2QT as NavigationToolbar)
import random
from matplotlib import rcParams, pyplot as pp
from smithplot import SmithAxes
import smithplot as sp

class MyDialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(1480, 884)
        self.runContButton = QtWidgets.QPushButton(Dialog)
        self.runContButton.setGeometry(QtCore.QRect(1320, 770, 131, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.runContButton.setFont(font)
        self.runContButton.setCheckable(False)
        self.runContButton.setDefault(False)
        self.runContButton.setFlat(False)
        self.runContButton.setObjectName("runContButton")
        self.stopRunButton = QtWidgets.QPushButton(Dialog)
        self.stopRunButton.setGeometry(QtCore.QRect(1320, 810, 131, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.stopRunButton.setFont(font)
        self.stopRunButton.setObjectName("stopRunButton")
        self.startFreqEdit = QtWidgets.QTextEdit(Dialog)
        self.startFreqEdit.setGeometry(QtCore.QRect(920, 800, 111, 31))
        self.startFreqEdit.setObjectName("startFreqEdit")
        self.label = QtWidgets.QLabel(Dialog)
        self.label.setGeometry(QtCore.QRect(920, 780, 111, 20))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.line = QtWidgets.QFrame(Dialog)
        self.line.setGeometry(QtCore.QRect(0, 720, 1481, 21))
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.label_2 = QtWidgets.QLabel(Dialog)
        self.label_2.setGeometry(QtCore.QRect(1050, 780, 111, 16))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(Dialog)
        self.label_3.setGeometry(QtCore.QRect(1180, 780, 111, 16))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.stepFreqEdit = QtWidgets.QTextEdit(Dialog)
        self.stepFreqEdit.setGeometry(QtCore.QRect(1050, 800, 111, 31))
        self.stepFreqEdit.setObjectName("stepFreqEdit")
        self.stopFreqEdit = QtWidgets.QTextEdit(Dialog)
        self.stopFreqEdit.setGeometry(QtCore.QRect(1180, 800, 111, 31))
        self.stopFreqEdit.setObjectName("stopFreqEdit")
        self.checkBox = QtWidgets.QCheckBox(Dialog)
        self.checkBox.setGeometry(QtCore.QRect(1340, 840, 101, 41))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.checkBox.setFont(font)
        self.checkBox.setObjectName("checkBox")
        self.connectButton = QtWidgets.QPushButton(Dialog)
        self.connectButton.setGeometry(QtCore.QRect(30, 840, 131, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.connectButton.setFont(font)
        self.connectButton.setCheckable(False)
        self.connectButton.setDefault(True)
        self.connectButton.setFlat(False)
        self.connectButton.setObjectName("connectButton")
        self.refreshButton = QtWidgets.QPushButton(Dialog)
        self.refreshButton.setGeometry(QtCore.QRect(30, 800, 131, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.refreshButton.setFont(font)
        self.refreshButton.setCheckable(False)
        self.refreshButton.setFlat(False)
        self.refreshButton.setObjectName("refreshButton")
        self.label_4 = QtWidgets.QLabel(Dialog)
        self.label_4.setGeometry(QtCore.QRect(30, 770, 131, 20))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.comPortEdit = QtWidgets.QComboBox(Dialog) #QtWidgets.QTextEdit(Dialog)
        self.comPortEdit.setGeometry(QtCore.QRect(30, 760, 131, 31))
        self.comPortEdit.setObjectName("comPortEdit")
        self.plot_tabWidget = QtWidgets.QTabWidget(Dialog)
        self.plot_tabWidget.setGeometry(QtCore.QRect(40, 50, 1401, 641))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.plot_tabWidget.setFont(font)
        self.plot_tabWidget.setObjectName("plot_tabWidget")
        self.tab = QtWidgets.QWidget()
        self.tab.setObjectName("tab")
        self.plot_tabWidget.addTab(self.tab, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.tab_3 = QtWidgets.QWidget()
        self.tab_3.setObjectName("tab_3")
        self.tab_4 = QtWidgets.QWidget()
        self.tab_4.setObjectName("tab_4")
        self.freqIndexSpinbox = QtWidgets.QSpinBox(self.tab_2)
        self.freqIndexSpinbox.setGeometry(QtCore.QRect(640, 40, 141, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.freqIndexSpinbox.setFont(font)
        self.freqIndexSpinbox.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.freqIndexSpinbox.setObjectName("freqIndexSpinbox")
        self.ringBox = QtWidgets.QComboBox(self.tab_2)
        self.ringBox.setGeometry(QtCore.QRect(230, 40, 211, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.ringBox.setFont(font)
        self.ringBox.setObjectName("ringBox")
        self.avgCountSpinbox = QtWidgets.QSpinBox(self.tab_2)
        self.avgCountSpinbox.setGeometry(QtCore.QRect(470, 40, 141, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.avgCountSpinbox.setFont(font)
        self.avgCountSpinbox.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.avgCountSpinbox.setObjectName("avgCountSpinbox")
        self.label_12 = QtWidgets.QLabel(self.tab_2)
        self.label_12.setGeometry(QtCore.QRect(430, 10, 211, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label_12.setFont(font)
        self.label_12.setAlignment(QtCore.Qt.AlignCenter)
        self.label_12.setObjectName("label_12")
        #self.ringBox.addItem("")
        #self.ringBox.addItem("")
        #self.ringBox.addItem("")
        #self.ringBox.addItem("")
        #self.ringBox.addItem("")
        #self.ringBox.addItem("")
        #self.ringBox.addItem("")
        #self.ringBox.addItem("")
        #self.ringBox.addItem("")
        self.label_7 = QtWidgets.QLabel(self.tab_2)
        self.label_7.setGeometry(QtCore.QRect(230, 10, 211, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label_7.setFont(font)
        self.label_7.setAlignment(QtCore.Qt.AlignCenter)
        self.label_7.setObjectName("label_7")
        self.label_8 = QtWidgets.QLabel(self.tab_2)
        self.label_8.setGeometry(QtCore.QRect(600, 10, 211, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label_8.setFont(font)
        self.label_8.setAlignment(QtCore.Qt.AlignCenter)
        self.label_8.setObjectName("label_8")
        self.label_9 = QtWidgets.QLabel(self.tab_2)
        self.label_9.setGeometry(QtCore.QRect(790, 10, 221, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label_9.setFont(font)
        self.label_9.setAlignment(QtCore.Qt.AlignCenter)
        self.label_9.setObjectName("label_9")
        self.freqValueText = QtWidgets.QTextEdit(self.tab_2)
        self.freqValueText.setGeometry(QtCore.QRect(810, 40, 171, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.freqValueText.setFont(font)
        self.freqValueText.setReadOnly(True)
        self.freqValueText.setAcceptRichText(True)
        self.freqValueText.setObjectName("freqValueText")
        self.plot_tabWidget.addTab(self.tab_2, "")
        self.plot_tabWidget.addTab(self.tab_3, "")
        self.plot_tabWidget.addTab(self.tab_4, "")
        self.label_5 = QtWidgets.QLabel(Dialog)
        self.label_5.setGeometry(QtCore.QRect(440, 740, 211, 20))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.label_5.setFont(font)
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.label_6 = QtWidgets.QLabel(Dialog)
        self.label_6.setGeometry(QtCore.QRect(10, 10, 1441, 31))
        font = QtGui.QFont()
        font.setPointSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.label_6.setFont(font)
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_6.setObjectName("label_6")


        # plot widgets
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        self.plot1_tab1 = pg.PlotWidget(self.tab,title='Magnitude',labels={'left':'Real','bottom':'Frequency [Hz]'})
        self.plot1_tab1.setGeometry(QtCore.QRect(50, 75, 630, 516))
        self.plot1_tab1.setObjectName("plot1_tab1")
        self.plot2_tab1 = pg.PlotWidget(self.tab,title='Phase',labels={'left':'Imag','bottom':'Frequency [Hz]'})
        self.plot2_tab1.setGeometry(QtCore.QRect(715, 75, 630, 516))
        self.plot2_tab1.setObjectName("plot2_tab1")

        self.plot1_tab2 = pg.PlotWidget(self.tab_2,title='Impedance')
        self.plot1_tab2.setGeometry(QtCore.QRect(50, 107, 1300, 466))
        self.plot1_tab2.setObjectName("plot1_tab2")

        # Tab 3 stuff
        self.smithchart_tab3 = pp.figure(figsize=(5, 3))
        pp.ion()
        self.smithcanvas = FigureCanvas(self.smithchart_tab3)
        layout = QtGui.QGridLayout()#QHBoxLayout()
        layout.addWidget(self.smithcanvas,0,0,1,1)
        sp.SmithAxes.scDefaultParams['axes.impedance'] = 100
        self.smith_ax = self.smithchart_tab3.add_subplot(111, projection='smith')
        self.z0_Label=QLabel("Z0 [Ohms] =  ")
        self.z0_Label.setObjectName("z0_Label")
        self.z0_Edit = QtWidgets.QLineEdit("100")
        self.z0_Edit.setObjectName("z0_Edit")
        layout.addWidget(self.z0_Label,1,0,1,1,Qt.AlignRight)
        layout.addWidget(self.z0_Edit,1,1,1,1,Qt.AlignLeft)
        layout.setHorizontalSpacing(1)
        self.smithchart_tab3.tight_layout()
        layout.setColumnStretch(0,2)
        self.plot2_tab3 = pg.PlotWidget(title='S11',labels={'left':'S11 [dB]','bottom':'Frequency [Hz]'})
        self.plot2_tab3.setGeometry(QtCore.QRect(715, 75, 630, 516))
        self.plot2_tab3.setObjectName("plot2_tab3")
        layout.addWidget(self.plot2_tab3,0,1,1,1)
        self.smithPlotButton = QtWidgets.QPushButton("Update Smith Chart")
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        self.smithPlotButton.setCheckable(False)
        self.smithPlotButton.setFlat(False)
        self.smithPlotButton.setObjectName("smithPlotButton")
        layout.addWidget(self.smithPlotButton,1,0,1,1,Qt.AlignCenter)
        self.smithPlotWebButton = QtWidgets.QPushButton("Save Plot")
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.smithPlotWebButton.setFont(font)
        self.smithPlotWebButton.setCheckable(False)
        self.smithPlotWebButton.setFlat(False)
        self.smithPlotWebButton.setObjectName("smithPlotButton")
        layout.addWidget(self.smithPlotWebButton,1,0,1,1,Qt.AlignLeft)
        self.toggleS11PlotButton = QtWidgets.QPushButton("Toggle Z'' vs Z' On")
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.toggleS11PlotButton.setFont(font)
        self.toggleS11PlotButton.setCheckable(False)
        self.toggleS11PlotButton.setFlat(False)
        self.toggleS11PlotButton.setObjectName("toggleS11PlotButton")
        layout.addWidget(self.toggleS11PlotButton,1,1,1,1,Qt.AlignRight)
        self.tab_3.setLayout(layout)

        # Tab 4 stuff
        self.plot1_tab4 = pg.PlotWidget(title="Z'",labels={'left':"Z' [Ohm]",'bottom':'Time [sec]'})
        self.plot1_tab4.setGeometry(QtCore.QRect(715, 75, 630, 516))
        self.plot1_tab4.setObjectName("plot1_tab4")
        self.plot2_tab4 = pg.PlotWidget(title="Z'",labels={'left':"Z' [Ohm]",'bottom':'Time [sec]'})
        self.plot2_tab4.setGeometry(QtCore.QRect(715, 75, 630, 516))
        self.plot2_tab4.setObjectName("plot2_tab4")
        self.plot3_tab4 = pg.PlotWidget(title="Z'",labels={'left':"Z' [Ohm]",'bottom':'Time [sec]'})
        self.plot3_tab4.setGeometry(QtCore.QRect(715, 75, 630, 516))
        self.plot3_tab4.setObjectName("plot3_tab4")
        self.plot4_tab4 = pg.PlotWidget(title="Z'",labels={'left':"Z' [Ohm]",'bottom':'Time [sec]'})
        self.plot4_tab4.setGeometry(QtCore.QRect(715, 75, 630, 516))
        self.plot4_tab4.setObjectName("plot4_tab4")

        self.avgCountSpinbox_tab4 = QtWidgets.QSpinBox(self.tab_4)
        font = QtGui.QFont()
        font.setPointSize(16)
        self.avgCountSpinbox_tab4.setFont(font)
        self.avgCountSpinbox_tab4.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.avgCountSpinbox_tab4.setObjectName("avgCountSpinbox_tab4")
        self.label_20 = QtWidgets.QLabel("Avg. Count:")
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label_20.setFont(font)
        self.label_20.setAlignment(QtCore.Qt.AlignCenter)
        self.label_20.setObjectName("label_20")

        self.avgTimeSpinbox_tab4 = QtWidgets.QSpinBox(self.tab_4)
        self.avgTimeSpinbox_tab4.setObjectName("avgTimeSpinbox_tab4")
        self.avgTimeSpinbox_tab4.setFont(font)
        self.label_21 = QtWidgets.QLabel("Time [sec]:")
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label_21.setFont(font)
        self.label_21.setAlignment(QtCore.Qt.AlignCenter)
        self.label_21.setObjectName("label_21")

        self.passLabel = QtWidgets.QLabel("PASS")
        font = QtGui.QFont()
        font.setPointSize(16)
        self.passLabel.setStyleSheet("color: rgb(0,255,0)")
        self.passLabel.setFont(font)
        self.passLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.passLabel.setObjectName("passLabel")

        self.label_30 = QtWidgets.QLabel("--")
        self.label_31 = QtWidgets.QLabel("--")
        self.label_32 = QtWidgets.QLabel("--")
        self.label_33 = QtWidgets.QLabel("--")
        font = QtGui.QFont()
        font.setPointSize(16)

        font = QtGui.QFont()
        font.setPointSize(16)
        self.freq_tab4_plot1_Spinbox_tab4 = QtWidgets.QSpinBox(self.tab_4)
        self.freq_tab4_plot1_Spinbox_tab4.setObjectName("freq_tab4_plot1_Spinbox_tab4")
        self.freq_tab4_plot1_Spinbox_tab4.setFont(font)
        self.freq_tab4_plot2_Spinbox_tab4 = QtWidgets.QSpinBox(self.tab_4)
        self.freq_tab4_plot2_Spinbox_tab4.setObjectName("freq_tab4_plot2_Spinbox_tab4")
        self.freq_tab4_plot2_Spinbox_tab4.setFont(font)
        self.freq_tab4_plot3_Spinbox_tab4 = QtWidgets.QSpinBox(self.tab_4)
        self.freq_tab4_plot3_Spinbox_tab4.setObjectName("freq_tab4_plot3_Spinbox_tab4")
        self.freq_tab4_plot3_Spinbox_tab4.setFont(font)
        self.freq_tab4_plot4_Spinbox_tab4 = QtWidgets.QSpinBox(self.tab_4)
        self.freq_tab4_plot4_Spinbox_tab4.setObjectName("freq_tab4_plot4_Spinbox_tab4")
        self.freq_tab4_plot4_Spinbox_tab4.setFont(font)

        self.zRecordButton = QtWidgets.QPushButton("Turn On Record")
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        self.zRecordButton.setCheckable(False)
        self.zRecordButton.setFlat(False)
        self.zRecordButton.setObjectName("zRecordButton")
        

        layout = QtGui.QGridLayout()
        layout.addWidget(self.plot1_tab4,1,1,1,1)
        layout.addWidget(self.plot2_tab4,1,2,1,1)
        layout.addWidget(self.plot3_tab4,1,3,1,1)
        layout.addWidget(self.plot4_tab4,1,4,1,1)
        layout.addWidget(self.avgCountSpinbox_tab4,0,2,1,1,Qt.AlignRight)
        layout.addWidget(self.label_20,0,2,1,1,Qt.AlignCenter)
        layout.addWidget(self.avgTimeSpinbox_tab4,0,3,1,1,Qt.AlignRight)
        layout.addWidget(self.label_21,0,3,1,1,Qt.AlignCenter)
        layout.addWidget(self.passLabel,0,1,1,1,Qt.AlignCenter)
        layout.addWidget(self.freq_tab4_plot1_Spinbox_tab4,2,1,1,1,Qt.AlignCenter)
        layout.addWidget(self.freq_tab4_plot2_Spinbox_tab4,2,2,1,1,Qt.AlignCenter)
        layout.addWidget(self.freq_tab4_plot3_Spinbox_tab4,2,3,1,1,Qt.AlignCenter)
        layout.addWidget(self.freq_tab4_plot4_Spinbox_tab4,2,4,1,1,Qt.AlignCenter)
        layout.addWidget(self.label_30,2,1,1,1,Qt.AlignRight)
        layout.addWidget(self.label_31,2,2,1,1,Qt.AlignRight)
        layout.addWidget(self.label_32,2,3,1,1,Qt.AlignRight)
        layout.addWidget(self.label_33,2,4,1,1,Qt.AlignRight)
        layout.addWidget(self.zRecordButton,0,4,1,1,Qt.AlignRight)
        self.tab_4.setLayout(layout)


        self.togglePlotButton = QtWidgets.QPushButton(self.tab)
        self.togglePlotButton.setGeometry(QtCore.QRect(585, 10, 220, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.togglePlotButton.setFont(font)
        self.togglePlotButton.setCheckable(False)
        self.togglePlotButton.setDefault(True)
        self.togglePlotButton.setFlat(False)
        self.togglePlotButton.setObjectName("connectButton")



        self.retranslateUi(Dialog)
        self.plot_tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "ADuCM355 Eval Board Tool"))
        self.runContButton.setText(_translate("Dialog", "Run Continuous"))
        self.stopRunButton.setText(_translate("Dialog", "Stop"))
        self.startFreqEdit.setHtml(_translate("Dialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'MS Shell Dlg 2\'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:10pt;\">10</span></p></body></html>"))
        self.label.setText(_translate("Dialog", "Start Freq. [kHz]"))
        self.label_2.setText(_translate("Dialog", "Step Freq. [kHz]"))
        self.label_3.setText(_translate("Dialog", "Stop Freq. [kHz]"))
        self.stepFreqEdit.setHtml(_translate("Dialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'MS Shell Dlg 2\'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:10pt;\">10</span></p></body></html>"))
        self.stopFreqEdit.setHtml(_translate("Dialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'MS Shell Dlg 2\'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:10pt;\">200</span></p></body></html>"))
        self.checkBox.setText(_translate("Dialog", "Save CSV"))
        self.connectButton.setText(_translate("Dialog", "Connect"))
        self.refreshButton.setText(_translate("Dialog", "Refresh"))
        self.label_4.setText(_translate("Dialog", "Serial Port"))
        #self.comPortEdit.setHtml(_translate("Dialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
#"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
#"p, li { white-space: pre-wrap; }\n"
#"</style></head><body style=\" font-family:\'MS Shell Dlg 2\'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
#"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:10pt;\">COM6</span></p></body></html>"))
        self.plot_tabWidget.setTabText(self.plot_tabWidget.indexOf(self.tab), _translate("Dialog", "Raw Data"))
        #self.ringBox.setItemText(0, _translate("Dialog", "Impedance"))
        #self.ringBox.setItemText(2, _translate("Dialog", "Raw Real"))
        #self.ringBox.setItemText(3, _translate("Dialog", "Raw Imag"))
        #self.ringBox.setItemText(4, _translate("Dialog", "Magnitude"))
        #self.ringBox.setItemText(5, _translate("Dialog", "Avg. Real"))
        #self.ringBox.setItemText(6, _translate("Dialog", "Avg. Imag"))
        #self.ringBox.setItemText(4, _translate("Dialog", "Magnitude"))
        #self.ringBox.setItemText(1, _translate("Dialog", "Phase"))
        #self.ringBox.setItemText(7, _translate("Dialog", "Rp"))
        #self.ringBox.setItemText(8, _translate("Dialog", "Cp"))
        self.label_7.setText(_translate("Dialog", "Ring"))
        self.label_12.setText(_translate("Dialog", "Avg. Count"))
        self.label_8.setText(_translate("Dialog", "Freq Index"))
        self.label_9.setText(_translate("Dialog", "Freq Value [kHz]"))
        self.freqValueText.setHtml(_translate("Dialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'MS Shell Dlg 2\'; font-size:14pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">10</p></body></html>"))
        self.plot_tabWidget.setTabText(self.plot_tabWidget.indexOf(self.tab_2), _translate("Dialog", "Run Chart"))
        self.plot_tabWidget.setTabText(self.plot_tabWidget.indexOf(self.tab_3), _translate("Dialog", "Smith Chart"))
        self.plot_tabWidget.setTabText(self.plot_tabWidget.indexOf(self.tab_4), _translate("Dialog", "Z' Charts"))
        self.label_5.setText(_translate("Dialog", "Console Log"))
        self.label_6.setText(_translate("Dialog", "ADuCM355 Eval Board Plotting Tool"))
        self.togglePlotButton.setText(_translate("Dialog", "Toggle Mag/Ph Off"))

