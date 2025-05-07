import threading
import time
import lgpio as GPIO
import queue
import os
import re
import math

class Dancer:
	def __init__(self, robotId):
		self.robotId = robotId
		self.pose = [0.0, 0.0, 0.0]  # x, y, h coordinates and angle of robot
		self.heading = 0 # angle in degrees
		self.targetPos = [0.0, 0.0] # x, y coordinates of target
		self.pathArr = []
		self.step = 0
		self.cache = []
		

	def updatePose(self, data):
		pattern = r":(\d+)X(-?\d+\.\d+)Y(-?\d+\.\d+)H(-?\d+\.\d+)"
		matches = re.findall(pattern, data)
		
		for match in matches:
			matchId= int(match[0])
			if matchId == self.robotId:
				self.pose[0] = float(match[1])
				self.pose[1] = float(match[2])
				self.pose[2] = float(match[3])
		#formattedList = [ '%.2f' % elem for elem in self.pose ]
		self.cache.append([self.pose[0], self.pose[1]])
	
	def getCache(self):
		return self.cache

	def initPath(self, path):
		self.pathArr = path

	def nextTarget(self):
		self.step = self.step + 1
		if self.step < len(self.pathArr):
			scaledPoint = tuple(x * 0.01 for x in self.pathArr[self.step])
			self.targetPos = scaledPoint

	def __str__(self):
		return f"Robot {self.robotId}: Pose: {self.pose} Target: {self.targetPos} step: {self.step}"
	
	def poseToTarget(self):
		
		dx = self.targetPos[0] - self.pose[0]
		dy = self.targetPos[1] - self.pose[1]
		targetAngle = math.degrees(math.atan2(dy, dx))
		
		dist = math.hypot(dx, dy) - 0.10
		
		head = targetAngle - self.pose[2]
		
		if head < -90:
			dist = -dist
			head = head + 180
		elif head > 90:
			dist = -dist
			head = head - 180
		
		move = f":{self.robotId}D{dist:.2f}H{head:.2f}"
		
		return move
	
	def goToFirst(self):
		if len(self.pathArr) >= 1:
			scaledPoint = tuple(x * 0.01 for x in self.pathArr[0])
			self.targetPos = scaledPoint
		else:
			print("ID " + str(self.robotId) + "s path array is empty. Please try to generate a path before calling this function")
	
	def resetStep(self):
		self.step = 0
		
