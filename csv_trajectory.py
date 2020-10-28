#!/usr/bin/env python

from math import radians
from delta_robot import DeltaRobotController
import numpy as np
import time

class RobotCommander(object):
    def __init__(self):
        self.t, self.a, self.b, self.c, self.d = np.loadtxt("/home/pi/delta_robot/angles.csv", delimiter=",", unpack=True)
        self.dt = []
        self.i = 0
        self.getTimeDifference()
        self.publish()

    def getTimeDifference(self):
        for x in range(len(self.t)):
            if x == len(self.t)-1:
                break
            self.dt.append(self.t[x+1] - self.t[x])

    def publish(self):
        robot = DeltaRobotController()
        for t, i in zip(self.dt, range(len(self.dt))):
            now = time.time()
            m1 = -self.a[i]
            m2 = -self.b[i]
            m3 = -self.c[i]
            robot.writeServoPositions(m1, m2, m3)
            elapsed = time.time() - now
            time.sleep(t/1000000 - elapsed)

if __name__ == "__main__":
    c = RobotCommander()