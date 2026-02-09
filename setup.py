#!/usr/bin/python3

# Low-level servo controller for 5-DOF arm (PCA9685). Use arm.Arm for the chainable API.
import time
import sys
import os
import json
import threading
import random
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import busio
from board import SCL, SDA
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x40) #default 0x40
pca.frequency = 50

curPath = os.path.realpath(__file__)
thisPath = '/' + os.path.dirname(curPath) + '/'

planJsonFileHere = open(thisPath + 'plan.json', 'r')
print(thisPath + 'plan.json')
contentPlanGose  = planJsonFileHere.read()
planGoseList = json.loads(contentPlanGose)

# Set the servo rotation angle.
def set_angle(ID, angle):
    servo_angle = servo.Servo(pca.channels[ID], min_pulse=500, max_pulse=2400,actuation_range=180)
    servo_angle.angle = angle

# Servo control.
class ServoCtrl(threading.Thread):
    def __init__(self, *args, **kwargs):
        super().__init__()
        self.__flag = threading.Event()
        self.__flag.clear()
        self.initAngle = [90,90,90,90, 90,90,90,90, 90,90,90,90, 90,90,90,90]  # 16 servo initial angle
        self.goalAngle = [90,90,90,90, 90,90,90,90, 90,90,90,90, 90,90,90,90]  # target angle
        self.nowAngle = [90,90,90,90, 90,90,90,90, 90,90,90,90, 90,90,90,90]  # current angle
        self.bufferAngle = [90.0,90.0,90.0,90.0, 90.0,90.0,90.0,90.0, 90.0,90.0,90.0,90.0, 90.0,90.0,90.0,90.0]  # buffer angle
        self.lastAngle = [90,90,90,90, 90,90,90,90, 90,90,90,90, 90,90,90,90]  # angle before change

        self.sc_direction = [1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1]  # 1: normal, -1: reverse
        self.scSpeed = [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]  # Servo rotation speed
        self.wiggleID = 0  # Servo ID
        self.wiggleDirection = 1  # 1: forward, -1: reverse
        self.maxAngle = 180
        self.minAngle = 0
        self.scMoveTime = 0.01
        self.goalUpdate = 0
        self.scMode = "auto"
        self.scSteps = 30
        self.scTime = 2.0
        # 5-DOF robotic arm
        self.nowPos = [90, 90, 90, 90, 90]
        self.planJsonFile = open(thisPath + 'plan.json', 'r')
        print(thisPath + 'plan.json')
        self.contentPlan  = self.planJsonFile.read()
        self.planSave = json.loads(self.contentPlan)

    def pause(self):  # Block thread
        self.__flag.clear()

    def resume(self):  # Resume thread
        self.__flag.set()

    # Update the servo angle value.
    def angleUpdate(self):
        self.goalUpdate = 1
        for i in range(0,16):
            self.lastAngle[i] = self.nowAngle[i]
        self.goalUpdate = 0

    # Get the servo angle.
    def servoAngle(self):
        return (self.nowAngle)

    # Initialize all servo angles.
    def moveInit(self):
        for i in range(0, 16):
            set_angle(i, self.initAngle[i])
            self.lastAngle[i] = self.initAngle[i]
            self.nowAngle[i] = self.initAngle[i]
            self.bufferAngle[i] = float(self.initAngle[i])
            self.goalAngle[i] = self.initAngle[i]
        self.pause()

    def initConfig(self, ID, initInput, moveTo):
        if initInput > self.minAngle and initInput < self.maxAngle:
            self.initAngle[ID] = initInput
            if moveTo:
                set_angle(ID, self.initAngle[ID])
            else:
                print("initAngle Value Error.")

    # The servo turns in a certain direction.
    def moveWiggle(self):
        self.bufferAngle[self.wiggleID] += self.wiggleDirection*self.sc_direction[self.wiggleID]*self.scSpeed[self.wiggleID]
        if self.bufferAngle[self.wiggleID] > self.maxAngle: self.bufferAngle[self.wiggleID] = self.maxAngle
        elif self.bufferAngle[self.wiggleID] < self.minAngle: self.bufferAngle[self.wiggleID] = self.minAngle
        newNow = int(round(self.bufferAngle[self.wiggleID],0))
        self.nowAngle[self.wiggleID] = newNow
        self.lastAngle[self.wiggleID] = newNow
        if self.bufferAngle[self.wiggleID] < self.maxAngle and self.bufferAngle[self.wiggleID] > self.minAngle:
            set_angle(self.wiggleID, self.nowAngle[self.wiggleID])
        else:
            self.stopWiggle()
        time.sleep(self.scMoveTime)

    # Set the angle to which a certain servo rotates.
    def moveAngle(self,ID, angleInput):
        self.nowAngle[ID] = int(self.initAngle[ID] + angleInput)
        if self.nowAngle[ID] > self.maxAngle: self.nowAngle[ID] = self.maxAngle
        elif self.nowAngle[ID] < self.minAngle: self.nowAngle[ID] = self.minAngle
        self.lastAngle[ID] = self.nowAngle[ID]
        set_angle(ID, self.nowAngle[ID])

    # Stop turning.
    def stopWiggle(self):
        self.pause()
        self.angleUpdate()

    # Set a single servo rotation.
    def singleServo(self, ID, directInput, speedSet):
        self.wiggleID = ID
        self.wiggleDirection = directInput
        self.scSpeed[ID] = speedSet
        self.scMode = "wiggle"
        self.angleUpdate()
        self.resume()

    # Move all servos to the specified position.
    def moveToPos(self, number, goalPos):
        if isinstance(goalPos, list):
            for i in range(0, len(goalPos)):
                self.goalAngle[i] = goalPos[i]
            for i in range(0, self.scSteps):
                for dc in range(0, number):
                    if not self.goalUpdate and self.goalAngle[dc] != self.nowAngle[dc]:
                        self.nowAngle[dc] = int(round((self.lastAngle[dc] + ((self.goalAngle[dc] - self.lastAngle[dc])/self.scSteps)*(i+1)), 0))
                        set_angle(dc, self.nowAngle[dc])
                        time.sleep(self.scMoveTime)
            self.angleUpdate()
            self.pause()
        else:
            print("goalPos not an array")

    # Save angle values to local file.
    def savePlanJson(self):
        content2write = json.dumps(planGoseList)
        file2write = open('plan.json', 'w')
        file2write.write(content2write)
        print(content2write)
        file2write.close()

    def createNewPlan(self):
        global planGoseList
        planGoseList = []
        print("planGoseList:",planGoseList)

    def newPlanAppend(self, nowPos):
        global planGoseList
        print(planGoseList)
        planGoseList.append(nowPos)
        print(planGoseList)

    def moveThreadingStop(self):
        self.scMode = 'stop'
        self.pause()

    def planThreadingStart(self):
        self.scMode = 'planMove'
        self.resume()

    def planGoes(self):
        self.scMode = 'planMove'
        if isinstance(planGoseList, list):
            for goalPos in planGoseList:
                if self.scMode == 'stop':
                    self.pause()
                    break
                print(goalPos)
                self.moveToPos(5, goalPos)
                time.sleep(1)
        else:
            print("planGoseList is not an array, and the content saved in the plan.json file is incorrect.")

    def scMove(self):
        if self.scMode == "init":
            self.moveInit()
        elif self.scMode == "wiggle":
            self.moveWiggle()
        elif self.scMode == 'planMove':
            self.planGoes()
        if self.scMode == 'stop':
            self.pause()

    def run(self):
        while True:
            self.__flag.wait()
            self.scMove()

if __name__ == "__main__":
    sc = ServoCtrl()
    sc.start()
    print("start!")
    goalPosList = [[90, 90], [20, 20], [160, 160]]
    try:
        while True:
            for goalPos in goalPosList:
                print(goalPos)
                sc.moveToPos(2, goalPos)
                time.sleep(2)
    except:
        print("stop!")
