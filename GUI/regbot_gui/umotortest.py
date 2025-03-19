#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2014-2022 by DTU
 #*   jcan@dtu.dk            
 #* 
 #* 
 #* The MIT License (MIT)  https://mit-license.org/
 #* 
 #* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 #* and associated documentation files (the “Software”), to deal in the Software without restriction, 
 #* including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 #* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 #* is furnished to do so, subject to the following conditions:
 #* 
 #* The above copyright notice and this permission notice shall be included in all copies 
 #* or substantial portions of the Software.
 #* 
 #* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 #* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 #* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 #* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 #* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 #* THE SOFTWARE. */

# import threading
# import time
import numpy as np
import pyqtgraph as pg

class UMotorTest(object):
  dataRead = False # for all servos
  inEdit = False
  inTimerUpdate = True
  hasFocus = False
  thisTab = -1
  # testdata
  mK = [0.1, 0.1]
  mR = [0.0, 0.0]
  mL = [0.001, 0.001]
  mFricDyn = [1e-6, 1e-6]
  mFricStat = [0.001, 0.001]
  mInertia = [0.0, 0.0]
  mValid = [False, False]
  # test response
  dataMax = 7000
  data = np.zeros((9,dataMax)) # test step response for
  dataIdx = 0
  dataUsed = 0 # filled elements on last get log
  # setup data
  motv = [3,3]
  motLow = 3
  motHigh = 6
  motCV = False
  motStateTime = 100
  # update flags
  dataReadMotv = True
  dataReadTest = True
  dataReadResults = True
  dataReadLog = True
  logTimerCnt = 0
  dataTestTimer = False
  dataVel = [0, 0]
  dataPos = [0, 0]
  # plot
  pwg = [] # handle for plot window
  cg  = 0 # plot
  #
  def __init__(self, parent):
    self.main = parent.main
    self.ui = parent.ui
    self.info = parent.robotInfo

  def setup(self):
    self.ui.pushButton_mt_motv_run.clicked.connect(self.startMotv)
    self.ui.pushButton_mt_motv_stop.clicked.connect(self.stopMotv)
    self.ui.pushButton_mt_run_test.clicked.connect(self.runTest)
    self.ui.pushButton_mt_get_log.clicked.connect(self.getLog)
    self.initGraph()
    pass

  def timerUpdate(self, timerCnt, justConnected):
    if self.dataReadLog:
      self.ui.spinBox_mt_log_samples.setValue(self.dataIdx)
      self.dataReadLog = False
      self.logTimerCnt = timerCnt
      self.dataTestTimer = True
    if self.dataReadTest:
      self.ui.doubleSpinBox_mt_v_low_2.setValue(self.motLow)
      self.ui.doubleSpinBox_mt_v_high_2.setValue(self.motHigh)
      self.ui.doubleSpinBox_mt_sample_time_2.setValue(self.motStateTime)
      self.ui.checkBox_mt_runClockWise_2.setChecked(self.motCV)
      self.dataReadTest = False
    if self.dataReadResults:
      self.ui.doubleSpinBox_mt_valid_left.setValue(self.mValid[0])
      self.ui.doubleSpinBox_mt_valid_right.setValue(self.mValid[1])
      self.ui.doubleSpinBox_mt_k_left.setValue(self.mK[0])
      self.ui.doubleSpinBox_mt_k_right.setValue(self.mK[1])
      self.ui.doubleSpinBox_mt_r_left.setValue(self.mR[0])
      self.ui.doubleSpinBox_mt_r_right.setValue(self.mR[1])
      # debug
      # print("# resistance use mR[0]=" + str(self.mR[0]) + " mR[1]=" + str(self.mR[1]) + " Ohm")
      self.ui.doubleSpinBox_mt_l_left.setValue(self.mL[0]*1000)
      self.ui.doubleSpinBox_mt_l_right.setValue(self.mL[1]*1000)
      # but inductance is not valid
      self.ui.doubleSpinBox_mt_l_left.setVisible(False)
      self.ui.doubleSpinBox_mt_l_right.setVisible(False)
      #
      self.ui.doubleSpinBox_mt_fric_dyn_left.setValue(self.mFricDyn[0])
      self.ui.doubleSpinBox_mt_fric_dyn_right.setValue(self.mFricDyn[1])
      self.ui.doubleSpinBox_mt_fric_stat_left.setValue(self.mFricStat[0])
      self.ui.doubleSpinBox_mt_fric_stat_right.setValue(self.mFricStat[1])
      self.ui.doubleSpinBox_mt_inertia_left.setValue(self.mInertia[0])
      self.ui.doubleSpinBox_mt_inertia_right.setValue(self.mInertia[1])
      self.dataReadResults = False
    pass
    if self.dataTestTimer:
      if (timerCnt - self.logTimerCnt) > 30:
        self.updateGraph()
        print("#time to update graph, log index=" + str(self.dataUsed) + ", time=" + str(timerCnt - self.logTimerCnt))
        #self.dataTestTimer = False
      pass
    #
    thisTab = self.ui.tabPages.indexOf(self.ui.tab_motortest)
    if (self.hasFocus or justConnected) and self.ui.tabPages.currentIndex() != thisTab:
      # just switched away from this tab
      self.hasFocus = False
      # if we are talking to a bridge - then just un-subscribe
      if self.main.isBridge():
        # NB! not the right messages
        self.main.devWrite(":motest subscribe 0\n") # gyro
        self.main.devWrite(":motpar subscribe 0\n") # gyro offset
        self.main.devWrite(":logdata subscribe 0\n") # gyro offset
      else:
        # talking to Teensy directly, so un-subscribe here
        self.main.devWrite("sub motest 0\n") # acc subscribe
        self.main.devWrite("sub motpar 0\n") # gyro subscribe
        pass
      pass
    if (not self.hasFocus or justConnected) and self.ui.tabPages.currentIndex() == thisTab:
      # just entering this tab
      self.hasFocus = True
      # if we are talking to a bridge - then just subscribe
      if self.main.isBridge():
        self.main.devWrite(":motest subscribe -1\n")
        self.main.devWrite(":motpar subscribe -1\n")
        self.main.devWrite(":logdata subscribe -1\n")
      else:
        # talking to Teensy directly, so subscribe here
        self.main.devWrite("sub motest 110\n") # imu board pose
        self.main.devWrite("sub motpar 111\n") # gyro subscribe
      pass
    pass

  def updateGraph(self):
    if self.dataUsed < self.dataMax and self.dataUsed > 0:
      # set the rest of the buffer to 0
      for i in range(self.dataUsed, self.dataMax):
        self.data[0,i] = self.data[0, self.dataUsed - 1]
        self.data[3,i] = 0
        self.data[4,i] = 0
        self.data[5,i] = 0
        self.data[6,i] = 0
        self.data[7,i] = 0
        self.data[8,i] = 0
    # update graph data
    self.cg[0].setData(x=self.data[0], y=self.data[3])
    self.cg[1].setData(x=self.data[0], y=self.data[4])
    self.cg[2].setData(x=self.data[0], y=self.data[5])
    self.cg[3].setData(x=self.data[0], y=self.data[6])
    self.cg[4].setData(x=self.data[0], y=self.data[7])
    self.cg[5].setData(x=self.data[0], y=self.data[8])
    self.dataTestTimer = False
  #
  def decode(self, gg):
    # Reading test results and current parameters
    dataUsed = True
    if gg[0] == "motest":
      #snprintf(s, MSL, "motest %d %f %g %g %g %g %g %g\r\n",
      #          0, voltageCCV, mKonstant[0], mResistance[0], 
      #          mInductance[0], mFricDyn[0], mFricStat[0], 
      #          mInertia[0]);      
      if len(gg) > 8:
        motor = int(gg[1])
        ccv_volt = float(gg[2])
        self.motCV = ccv_volt > 0
        self.mK[motor] = float(gg[3])
        self.mR[motor] = float(gg[4])
        self.mL[motor] = float(gg[5])
        self.mFricDyn[motor] = float(gg[6])
        self.mFricStat[motor] = float(gg[7])
        self.mInertia[motor] = float(gg[8])
        self.mValid[motor] = int(gg[9])
        self.dataReadResults = True
        if self.main.deviceID > 0 and self.mValid[motor] == 1:
          # save robot parameters to list of robot parameters
          # to be saved with file->save configuration to ini
          robot = self.info.getRobot(self.main.deviceID)
          #print("# saving motor parameters for " + str(self.main.deviceID) + " valid = " + str(self.mValid[motor]))
          robot.motor_K[motor] = self.mK[motor]
          robot.motor_R[motor] = self.mR[motor]
          robot.motor_L[motor] = self.mL[motor]
          robot.motor_B[motor] = self.mFricDyn[motor]
          robot.motor_S[motor] = self.mFricStat[motor]
          robot.motor_J[motor] = self.mInertia[motor]
        # debug
        # print("# resistance decode " + str(self.mR[motor]) + " from " + gg[4] + " motor " + str(motor))
    elif gg[0] == "motpar":
      #   snprintf(s, MSL, "motpar %f %f %d\r\n",
      #   voltageLow * voltageCCV, voltageHigh * voltageCCV, stateLength);
      if len(gg) > 3:
        self.motLow = float(gg[1])
        self.motHigh = float(gg[2])
        self.motStateTime = int(gg[3])/1000.0
        self.dataReadTest = True
    else:
      dataUsed = False
      
    if self.hasFocus:
      # test also for logfile entry
      # to add to graph data      
      # but hide that it is used
      if (gg[0] == 'logend'):
        self.updateGraph()
        print("# motortest, log end received")
      elif (gg[0][0] == '%'):
        # start of log-data - ignore,
        # but reset index
        #self.dataIdx = 0
        #self.dataPos = [0,0]
        #print("# got log header " + gg[0] + " " + gg[1])
        pass
      elif ((gg[0][0] >= '0' and (gg[0][0] <= '9') or gg[0][0] == '.') and len(gg) == 9):
        # loglist direct from Teensy - data line
        #   usb.send("% Motor test log\n");
        #if self.dataIdx < 10:
        #  print("# got log data " + str(len(gg)) + " params, time " + gg[0])
        #usb.send("% 1 time stamp (sec)\n");
        #usb.send("% 2 Motor CCV factor (1 = CV, -1 = CCV)\n");
        #usb.send("% 3 sample number\n");
        #usb.send("% 4 Left motor voltage\n");
        #usb.send("% 5 Left motor current\n");
        #usb.send("% 6 Left motor velocity\n");
        #usb.send("% 7 Right motor voltage\n");
        #usb.send("% 8 Right motor current\n");
        #usb.send("% 9 Right motor velocity\n");
        #print("# motortest log len(gg) is " + str(len(gg)))
        try:
          self.data[0, self.dataIdx] = float(gg[0]) # time
          self.data[1, self.dataIdx] = float(gg[1]) # (CV=1 or CCV=-1)
          self.data[2, self.dataIdx] = float(gg[2]) # sample number
          self.data[3, self.dataIdx] = float(gg[3])/10 # volt left
          self.data[4, self.dataIdx] = float(gg[4])    # current
          pos = float(gg[5])
          self.dataVel[0] = self.dataVel[0] * 0.5 + (pos - self.dataPos[0]) * 0.5
          self.dataPos[0] = pos
          self.data[5, self.dataIdx] = self.dataVel[0]
          self.data[6, self.dataIdx] = float(gg[6])/10
          self.data[7, self.dataIdx] = float(gg[7])
          pos = float(gg[8])
          self.dataVel[1] = self.dataVel[1] * 0.8 + (pos - self.dataPos[1]) * 0.2
          self.dataPos[1] = pos
          self.data[8, self.dataIdx] = self.dataVel[1]
          #
          self.dataUsed = self.dataIdx
          if self.dataIdx < self.dataMax - 1:
            self.dataIdx += 1
          self.dataReadLog = True
        except:
          print("# motortest read log failed (index=" + str(self.dataIdx) + ")")
    return dataUsed
  
  def startMotv(self):
    # send motv to robot
    print("# starting motor voltage")
    self.main.devWrite("motv {:f} {:f}\n".
                         format(self.ui.doubleSpinBox_mt_motv_left.value(), 
                                self.ui.doubleSpinBox_mt_motv_right.value()), True
                         )
    pass
    
  def stopMotv(self):
    print("# stop motor voltage")
    self.main.devWrite("stop\n")
    # send stop motors
    pass
    
  def runTest(self):
    print("# starting test")
    self.main.devWrite("motset {:f} {:f} {:d}\n".format(
                       self.ui.doubleSpinBox_mt_v_low.value(), 
                       self.ui.doubleSpinBox_mt_v_high.value(),
                       int(self.ui.doubleSpinBox_mt_sample_time.value() * 1000)))
    estCCV = self.ui.checkBox_mt_runClockWise.isChecked() == False
    self.main.devWrite("mottest {:d}\n".format(estCCV), True)
    # ask for test to run with current parameters
    pass
    
  def getLog(self):
    print("# motortest get log")
    self.dataIdx = 0
    self.dataPos = [0,0]
    self.main.devWrite("log\n")
    # just ask for normal log
    pass
  
  def saveToIniFile(self, config):
    # settings
    config.add_section('motor')
    config.set('motor', 'motv_left', str(self.ui.doubleSpinBox_mt_motv_left.value()))
    config.set('motor', 'motv_right', str(self.ui.doubleSpinBox_mt_motv_right.value()))
    config.set('motor', 'state_time', str(self.ui.doubleSpinBox_mt_sample_time.value()))

  def loadFromIniFile(self, config):
    try:
      print("# Loading to motortest " + str(config.get('motor', 'state_time')))
      self.ui.doubleSpinBox_mt_motv_left.setValue(config.get('motor', 'motv_left'))
      self.ui.doubleSpinBox_mt_motv_right.setValue(config.get('motor', 'motv_right'))
      self.ui.doubleSpinBox_mt_sample_time.setValue(config.get('motor', 'state_time'))
      print("# Loading to motortest 2 " + str(config.get('motor', 'state_time')))
    except:
      self.main.message("# failed to load motor test section from ini-file")
    pass
    if self.main.deviceID > 0:
      # save robot parameters to list of robot parameters
      # to be saved with file->save configuration to ini
      #print("# saving motor parameters for " + str(self.main.deviceID) + " valid = " + str(self.mValid[motor]))
      robot = self.info.getRobot(self.main.deviceID)
      for motor in range(0,2):
        self.mK[motor] = robot.motor_K[motor]
        self.mR[motor] = robot.motor_R[motor]
        self.mL[motor] = robot.motor_L[motor]
        self.mB[motor] = robot.motor_B[motor]
        self.mS[motor] = robot.motor_S[motor]
        self.mJ[motor] = robot.motor_J[motor]

  def initGraph(self):
    # "initialize graph plot of step data"
    # giving the plots names allows us to link their axes together
    self.pwg = pg.PlotWidget(name='motres',title='Test responce')  
    self.pwg.setLabel('bottom','time','s')
    self.pwg.setLabel('left','value','')
    self.pwg.addLegend()    
    #self.pwg.setLimits([0, 8.0, -5.0, 5.0])
    self.pwg.setWindowTitle('Test responce')
    self.ui.robot_graph_layout_mt.addWidget(self.pwg)
    pm = pg.mkPen(color=(200, 200, 255))
    self.cg = [self.pwg.plot(pen='r',name='left V/10'), 
               self.pwg.plot(pen='y',name='left A'), 
               self.pwg.plot(pen='g',name='left rad/sec'),
               self.pwg.plot(pen=pm,name='right V/10'), 
               self.pwg.plot(pen=pg.mkPen(color=(255, 200, 200)),name='right A'), 
               self.pwg.plot(pen=pg.mkPen(color=(200, 255, 200)),name='right rad/sec')]
    self.cg[0].setData(x=self.data[0], y=self.data[3])
    self.cg[1].setData(x=self.data[0], y=self.data[4])
    self.cg[2].setData(x=self.data[0], y=self.data[5])
    self.cg[3].setData(x=self.data[0], y=self.data[6])
    self.cg[4].setData(x=self.data[0], y=self.data[7])
    self.cg[5].setData(x=self.data[0], y=self.data[8])
