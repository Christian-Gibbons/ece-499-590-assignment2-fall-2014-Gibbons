import diff_drive
import pid_include as pid
import cv_tracking_include as cvti
import ach
import sys
import time
import numpy as np
from ctypes import *


dd = diff_drive
tim = dd.H_TIME()

ROBOT_TIME_CHAN  = 'robot-time'
CV_REF_CHAN = cvti.CV_REF_NAME
PID_REF_CHAN = pid.PID_REF_NAME

t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()
cvt = ach.Channel(CV_REF_CHAN)
cvt.flush()
p = ach.Channel(pid.PID_REF_NAME)
p.flush()

PIDout = pid.PID_REF()
cvPID = cvti.CV_REF()

lastTime = 0
Input = 0.0
lastInput = 0.0
Output = 0.0
setPoint = 0
errSum = 0

ku = .0038
Tu = 2.5
kp = (0.6*ku)
ki = (2.0*kp)/Tu
kd = (kp*Tu)/8

sampleRate = 1.0 #seconds

#PID control loop
while(1):
	while(tim.sim[0] == lastTime):
		[status, framesize] = t.get(tim, wait=False, last=True)
		if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
			pass
			#print 'Sim Time = ', tim.sim[0]
		else:
			raise ach.AchException( v.result_string(status) )
	[statusc, framesizec] = cvt.get(cvPID, wait=False, last=True)

	setPoint = cvPID.setX
	Input = cvPID.X
	dt = tim.sim[0] - lastTime #delta T, change in time
	error = setPoint - Input 
	errSum += (error*dt) #integral of error
	dInput = (Input - lastInput)/dt #derivative of input; use this to avoid spikes at new setpoints
	Output = (kp*error) + (ki*errSum) - (kd*dInput)
	print "Pre-Output: ", Output
	if(Output > 1.0):
		Output = 1.0
	elif(Output < -1.0):
		Output = -1.0
	print "Output: ", Output
	PIDout.wheel[0] = Output
	PIDout.wheel[1] = -Output
	p.put(PIDout)
	print "Wheel[0]: ", PIDout.wheel[0], "% Wheel[1]: ", PIDout.wheel[1], "%"
#	lastError = error
	lastInput = Input
	lastTime = tim.sim[0]
	
	[status, framesize] = t.get(tim, wait=False, last=True)
	if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
		pass
		#print 'Sim Time = ', tim.sim[0]
	else:
		raise ach.AchException( v.result_string(status) )
	sleepTime = (sampleRate - (tim.sim[0] - lastTime))
	print sleepTime
	time.sleep(sleepTime)
