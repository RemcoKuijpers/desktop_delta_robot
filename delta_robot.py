print("Initializing")
from lx16a import *
from kinematics import DeltaRobotKinematics, DeltaPositionError
from math import sin, cos, pi
from gripper import Gripper
import time
import numpy as np
from scipy import interpolate

class DeltaRobotController(object):
    def __init__(self, using_vision=False):
        if using_vision == True:
            from vision import Vision
            self.vision = Vision()
        self.kin = DeltaRobotKinematics()
        LX16A.initialize("/dev/ttyUSB0")
        self.servo1 = LX16A(1) #0 is 85
        self.servo2 = LX16A(2) #0 is 90
        self.servo3 = LX16A(3) #0 is 98
        self.offsets = [85, 90, 98]
        self.gripper = Gripper()
        self.gripper.close()
        print("Initializing done ...")

    def getServoInfo(self):
        print("The following servo ID's were found: %s, %s, %s" % (self.servo1.IDRead(), self.servo2.IDRead(), self.servo3.IDRead()))
        print("The servo temperatures are: %s, %s, %s in degrees celsius" % (self.servo1.tempRead(), self.servo2.tempRead(), self.servo3.tempRead()))
        print("The input voltages are: %s, %s, %s in millivolts" % (self.servo1.vInRead(), self.servo2.vInRead(), self.servo3.vInRead()))

    def writeServoPositions(self, m1, m2, m3):
        self.servo1.moveTimeWrite(self.offsets[0] + m1)
        self.servo2.moveTimeWrite(self.offsets[1] + m2)
        self.servo3.moveTimeWrite(self.offsets[2] + m3)

    def homeServos(self, t=1.5):
        self.sendInterpolatedJointMove(0, 0, 0, t)

    def scanPosition(self, t=1.5):
        self.sendInterpolatedJointMove(-60, 15, 15, t)

    def getServoPositions(self):
        p1 = self.servo1.getPhysicalPos() - self.offsets[0]
        p2 = self.servo2.getPhysicalPos() - self.offsets[1]
        p3 = self.servo3.getPhysicalPos() - self.offsets[2]
        return p1, p2, p3

    def getCartesianPosition(self):
        m1, m2, m3 = self.getServoPositions()
        x, y, z = self.kin.forward(m1, m2, m3)
        return x, y, z

    def sendCartesianMove(self, x, y, z):
        m1, m2, m3 = self.kin.reverse(x, y, z)
        self.writeServoPositions(m1, m2, m3)

    def sendInterpolatedCartesianMove(self, x, y, z, t, type="linear"):
        x_old, y_old, z_old = self.getCartesianPosition()
        time = np.array([0, t])
        x = np.array([x_old, x])
        y = np.array([y_old, y])
        z = np.array([z_old, z])
        fx = interpolate.interp1d(time, x, kind=type)
        fy = interpolate.interp1d(time, y, kind=type)
        fz = interpolate.interp1d(time, z, kind=type)
        time_new = np.arange(0, t, 0.01)
        x_new = fx(time_new)
        y_new = fy(time_new)
        z_new = fz(time_new)
        import time
        for a, b, c in zip(x_new, y_new, z_new):
            now = time.time()
            self.sendCartesianMove(a,b,c)
            elapsed = time.time() - now
            time.sleep(0.01 - elapsed)

    def sendInterpolatedJointMove(self, m1, m2, m3, t, type="linear"):
        m1_old, m2_old, m3_old = self.getServoPositions()
        time = np.array([0, t])
        m1 = np.array([m1_old, m1])
        m2 = np.array([m2_old, m2])
        m3 = np.array([m3_old, m3])
        fm1 = interpolate.interp1d(time, m1, kind=type)
        fm2 = interpolate.interp1d(time, m2, kind=type)
        fm3 = interpolate.interp1d(time, m3, kind=type)
        time_new = np.arange(0, t, 0.02)
        m1_new = fm1(time_new)
        m2_new = fm2(time_new)
        m3_new = fm3(time_new)
        import time
        for a, b, c in zip(m1_new, m2_new, m3_new):
            now = time.time()
            self.writeServoPositions(a, b, c)
            elapsed = time.time() - now
            time.sleep(0.02 - elapsed)

    def sendTrajectory(self, poses, t, type="linear"):
        x_old, y_old, z_old = self.getCartesianPosition()
        time = np.array([0])
        x = np.array([x_old])
        y = np.array([y_old])
        z = np.array([z_old])
        for p in poses:
            x = np.append(x, p[0])
            y = np.append(y, p[1])
            z = np.append(z, p[2])
        for i in range(len(poses)):
            time = np.append(time, ((i+1)/len(poses))*t)
        fx = interpolate.interp1d(time, x, kind=type)
        fy = interpolate.interp1d(time, y, kind=type)
        fz = interpolate.interp1d(time, z, kind=type)
        time_new = np.arange(0, t, 0.02)
        x_new = fx(time_new)
        y_new = fy(time_new)
        z_new = fz(time_new)
        import time
        for a, b, c in zip(x_new, y_new, z_new):
            now = time.time()
            self.sendCartesianMove(a,b,c)
            elapsed = time.time() - now
            time.sleep(0.02 - elapsed)

    def pick(self, x, y, z, h):
        self.gripper.open()
        z = z - 210
        p1 = []
        p2 = []
        p1.append([x, y, z+h])
        p1.append([x,y,z])
        p2.append([x, y, z+h])
        self.sendTrajectory(p1, 1)
        time.sleep(0.3)
        self.gripper.close()
        time.sleep(0.3)
        self.sendTrajectory(p2, 0.25)

    def place(self, x, y, z, h):
        z = z - 210
        p1 = []
        p2 = []
        p1.append([x, y, z+h])
        p1.append([x,y,z])
        p2.append([x, y, z+h])
        self.sendTrajectory(p1, 1)
        time.sleep(0.3)
        self.gripper.open()
        time.sleep(0.3)
        self.sendTrajectory(p2, 0.25)

    def sickCircleMovements(self):
        while True:
            x = 50*sin(time.time()*4)
            y = 0 #50*cos(time.time()*4)
            z = 30*sin(time.time()*2) - 180
            self.sendCartesianMove(x,y,z)

    def pointToPointMoves(self):
        self.sendInterpolatedCartesianMove(0, 50, -230, 0.5)
        self.sendInterpolatedCartesianMove(0, 0, -200, 0.4)
        self.sendInterpolatedCartesianMove(0, -50, -230, 0.5)
        self.sendInterpolatedCartesianMove(0, 0, -200, 0.4)
        self.sendInterpolatedCartesianMove(0, 0, -230, 0.3)
        self.sendInterpolatedCartesianMove(0, 0, -200, 0.3)
        self.homeServos(t=1)

    def trajectoryMovements(self):
        positions = []
        positions.append([0, 50, -210])
        positions.append([0, 50, -230])
        positions.append([0, 50, -210])
        positions.append([0, 0, -190])
        positions.append([0, -50, -210])
        positions.append([0, -50, -230])
        positions.append([0, -50, -210])
        positions.append([0, 0, -190])
        positions.append([50, 0, -210])
        positions.append([50, 0, -230])
        positions.append([50, 0, -210])
        positions.append([0, 0, -190])
        positions.append([-50, 0, -210])
        positions.append([-50, 0, -230])
        positions.append([-50, 0, -210])
        positions.append([0, 0, -190])
        self.homeServos()
        while True:
            elapsed = time.time()
            self.sendTrajectory(positions, 2, type="linear")
            print("Trjactory took: " + str(time.time() - elapsed) + " seconds")

    def trajectoryMovementsWithGripper(self, z_offset):
        positions1 = []
        positions2 = []
        positions3 = []
        positions4 = []
        positions1.append([0, 50, -180+z_offset])
        positions1.append([0, 50, -230+z_offset])
        positions2.append([0, 50, -180+z_offset])
        positions2.append([0, 0, -180+z_offset])
        positions2.append([0, -50, -180+z_offset])
        positions2.append([0, -50, -230+z_offset])
        positions3.append([0, -50, -180+z_offset])
        positions3.append([0, -50, -230+z_offset])
        positions4.append([0, -50, -180+z_offset])
        positions4.append([0, 0, -180+z_offset])
        positions4.append([0, 50, -180+z_offset])
        positions4.append([0, 50, -230+z_offset])

        self.homeServos()
        while True:
            self.sendTrajectory(positions1, 1, type="linear")
            time.sleep(0.25)
            self.gripper.close()
            time.sleep(0.25)
            self.sendTrajectory(positions2, 1, type="linear")
            time.sleep(0.25)
            self.gripper.open()
            time.sleep(0.25)
            self.sendTrajectory(positions3, 1, type="linear")
            time.sleep(0.25)
            self.gripper.close()
            time.sleep(0.25)
            self.sendTrajectory(positions4, 1, type="linear")
            time.sleep(0.25)
            self.gripper.open()
            time.sleep(0.25)

    def buildCylinderTower(self):
        self.scanPosition()
        h = 5
        while True:
                circles = self.vision.getCirclesFromCam(debug=True)
                if circles is not None:
                    for circle in circles:
                        self.pick(circle[0], circle[1], 5, 30)
                        self.place(0,0,h,30)
                        h += 16
                self.scanPosition()
                input("Press Enter to continue...")
                h = 5

if __name__ == "__main__":
    robot = DeltaRobotController(using_vision=True)
    robot.buildCylinderTower()