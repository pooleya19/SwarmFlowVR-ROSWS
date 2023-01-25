#!/usr/bin/env python3

class PID:
	def __init__(self, Pgain, Igain, Dgain):
		self.P = Pgain
		self.I = Igain
		self.D = Dgain

		self.lastError = None
		self.lastTime = None
		self.integral = 0

	def step(self, error, time):
		# Handle P component
		compP = self.P * error

		# Handle I component
		self.integral += error
		compI = self.I * self.integral
		
		# Handle D component
		if self.lastError != None:
			compD = self.D * (error - self.lastError)/(time - self.lastTime)
		else:
			compD = 0
		
		self.lastError = error
		self.lastTime = time

		# Return component sum
		return compP + compI + compD