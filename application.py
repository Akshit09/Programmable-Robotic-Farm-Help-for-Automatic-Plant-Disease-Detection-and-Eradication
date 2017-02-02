import sys
import trace
import threading

'''
class KThread(threading.Thread):
	def __init__(self, *args, **keywords):
		threading.Thread.__init__(self, *args, **keywords)
		self.killed = False
	def start(self):
		self.__run_backup = self.run
		self.run = self.__run
		threading.Thread.start(self)
	def __run(self):
		sys.settrace(self.globaltrace)
		sys.__run_backup()
		self.run = self.__run_backup
	def globaltrace(self, frame, why, arg):
		if why == 'call':
			return self.localtrace
		else:
			return None
	def localtrace(self, frame, why, arg):
		if self.killed:
			if why == 'line' :
				raise SystemExit()
		return self.localtrace
	def kill(self):
		self.killed = True
'''

class KThread(threading.Thread):
	def start(self):
		self.__run_backup = self.run
		self.run = self.__run
		threading.Thread.start(self)
	def kill(self):
		self.__stop = True
	def _bootstrap(self):
		if threading ._trace_hook is not None:
			raise ValueError('Cannot run thread with tracing')
		self.__stop = False
		sys.settrace(self.trace)
		super()._bootstrap()

	def __trace(self, frame, event, arg):
		if self._stop:
			raise StopThread()
		return self.__trace



import RPi.GPIO as GPIO
import time
import random

print "STARTING GPIO"
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.cleanup()

#FOR MOTOR 1
M1_A = 11
GPIO.setup(M1_A, GPIO.OUT)
GPIO.output(M1_A, 0)
M1_B = 33
GPIO.setup(M1_B, GPIO.OUT)
GPIO.output(M1_B, 0)

#FOR MOTOR 2
M2_A = 35
GPIO.setup(M2_A, GPIO.OUT)
GPIO.output(M2_A, 0)
M2_B = 37
GPIO.setup(M2_B, GPIO.OUT)
GPIO.output(M2_B, 0)


def f():
	GPIO.output(M1_A, 0)
	GPIO.output(M1_B, 1)
	GPIO.output(M2_A, 0)
	GPIO.output(M2_B, 1)

def b():
	GPIO.output(M1_A, 1)
	GPIO.output(M1_B, 0)
	GPIO.output(M2_A, 1)
	GPIO.output(M2_B, 0)

def s():
	GPIO.output(M1_A, 0)
	GPIO.output(M1_B, 0)
	GPIO.output(M2_A, 0)
	GPIO.output(M2_B, 0)

def r():
	GPIO.output(M1_A, 0)
	GPIO.output(M1_B, 1)
	GPIO.output(M2_A, 1)
	GPIO.output(M2_B, 0)

def l():
	GPIO.output(M1_A, 1)
	GPIO.output(M1_B, 0)
	GPIO.output(M2_A, 0)
	GPIO.output(M2_B, 1)



fThread = KThread(target = f)
forwardThread = False
bThread = KThread(target = b)
backwardThread = False
lThread = KThread(target = l)
rightThread = False
rThread = KThread(target = r)
leftThread = False
sThread = KThread(target = s)
stopThread = False



def clean():

	global forwardThread
	global backwardThread
	global rightThread
	global leftThread
	global stopThread
	global fThread
	global bThread
	global rThread
	global lThread
	global sThread

	if forwardThread:
		fThread.kill()
		forwardThread = False
	if backwardThread:
		bThread.kill()
		backwardThread = False
	if rightThread:
		rThread.kill()
		rightThread = False
	if leftThread:
		lThread.kill()
		leftThread = False
	if stopThread:
		sThread.kill()
		stopThread = False


#--------------------------
#SETTING UP GPIO STRUCTURE
#--------------------------

def forward():
	global forwardThread
	global fThread
	
	try:
		#thread.start_new_thread(f)
		fThread.start()
		forwardThread = True
	except:
		print "Error: unable to start thread"

def backward():
	global backwardThread
	global bThread
	try:
		#thread.start_new_thread(b)
		bThread.start()
		backThread = True
	except:
		print "Error: unable to start thread"

def stop():
	global stopThread
	global sThread
	try:
		#thread.start_new_thread(s)
		sThread.start()
		stopThread = True
	except:
		print "Error: unable to start thread"


def right():
	global rightThread
	global rThread
	try:
		#thread.start_new_thread(r)
		rThread.start()
		rightThread = True
	except:
		print "Error: unable to start thread"


def left():
	global leftThread
	global lThrea
	try:
		#thread.start_new_thread(l)
		lThread.start()
		leftThread = True
	except:
		print "Error: unable to start thread"

	




#------------------------------------------------------------
#DRIVER ENDS HERE!
#------------------------------------------------------------

import cv2
import numpy as np
import math

print "Capturing"
#Capturing the frame
cap = cv2.VideoCapture(0)

while(cap.isOpened()):
    print "inside while"
    ret, img = cap.read()
    cv2.rectangle(img,(300,300),(100,100),(0,255,0),0)
    crop_img = img[100:300, 100:300]
    grey = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
    value = (35, 35)
    blurred = cv2.GaussianBlur(grey, value, 0)
    _, thresh1 = cv2.threshold(blurred, 127, 255,
                               cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    cv2.imshow('Thresholded', thresh1)
    _, contours, hierarchy = cv2.findContours(thresh1.copy(),cv2.RETR_TREE, \
            cv2.CHAIN_APPROX_NONE)
    max_area = -1
    for i in range(len(contours)):
        cnt=contours[i]
        area = cv2.contourArea(cnt)
        if(area>max_area):
            max_area=area
            ci=i
    cnt=contours[ci]
    x,y,w,h = cv2.boundingRect(cnt)
    cv2.rectangle(crop_img,(x,y),(x+w,y+h),(0,0,255),0)
    hull = cv2.convexHull(cnt)
    drawing = np.zeros(crop_img.shape,np.uint8)
    cv2.drawContours(drawing,[cnt],0,(0,255,0),0)
    cv2.drawContours(drawing,[hull],0,(0,0,255),0)
    hull = cv2.convexHull(cnt,returnPoints = False)
    defects = cv2.convexityDefects(cnt,hull)
    count_defects = 0
    cv2.drawContours(thresh1, contours, -1, (0,255,0), 3)
    for i in range(defects.shape[0]):
        s,e,f,d = defects[i,0]
        start = tuple(cnt[s][0])
        end = tuple(cnt[e][0])
        far = tuple(cnt[f][0])
        a = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        b = math.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
        c = math.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)
        angle = math.acos((b**2 + c**2 - a**2)/(2*b*c)) * 57
        if angle <= 90:
            count_defects += 1
            cv2.circle(crop_img,far,1,[0,0,255],-1)
        #dist = cv2.pointPolygonTest(cnt,far,True)
        cv2.line(crop_img,start,end,[0,255,0],2)
        #cv2.circle(crop_img,far,5,[0,0,255],-1)

    print "going to calibrate"

    if count_defects == 1:
	print "FORWARD"
	#FORWARD
        cv2.putText(img,"DEFECT 1", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
	clean()
	forward()

    elif count_defects == 2:
	print "LEFT"
	#LEFT
        cv2.putText(img,"DEFECT 2", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
	clean()
	left()

    elif count_defects == 3:
	print "RIGHT"
	#RIGHT
        cv2.putText(img,"DEFECT 3", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
	clean()
	right()

    elif count_defects == 4:
	print "BACKWARD"
	#BACK
        cv2.putText(img,"DEFECT 4", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
	clean()
	backward()

    else:
	print "STOP"
	#STOP
        cv2.putText(img,"Acquiring target", (50,50),\
                    cv2.FONT_HERSHEY_SIMPLEX, 2, 2)
	clean()
	stop()

    #cv2.imshow('drawing', drawing)
    #cv2.imshow('end', crop_img)
    cv2.imshow('Gesture', img)
    all_img = np.hstack((drawing, crop_img))
    cv2.imshow('Contours', all_img)
    k = cv2.waitKey(10)
    if k == 27:
	clean()
	stop()
        break
