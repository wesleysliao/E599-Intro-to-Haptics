"""
KinArm.py

A visualisation script for learning robot kinematics on a KinArm-like RR arm device.

Eatai Roth, 2020
"""

import pygame
import numpy as np

BLACK = (0,0,0)
WHITE = (255, 255, 255)

GRAY75 = (63, 63, 63)
GRAY50 = (127, 127, 127)
GRAY25 = (191, 191, 191)

PALEBLUE = (200, 240, 255)
NAVY = (0, 0, 153)
DEEPBLUE = (0, 128, 255)
GOLD = (240, 195, 10)
CORAL = (255,125,80)
CRIMSON = (204, 0, 0)

METERS2PIX = 400

def screenCoords(p, screenSize):
    return np.array([0.5*screenSize[0]+p[0]*METERS2PIX, screenSize[1]-(p[1]+0.1)*METERS2PIX]).astype(int)

def inverseScreenCoords(sp, screenSize):
    return np.array([sp[0] - 0.5 * screenSize[0], (screenSize[1]-sp[1]-0.1*METERS2PIX)]).astype(float)/METERS2PIX

def interpPoints(p1, p2, numSteps):
    return np.outer(np.ones(numSteps), p1) + np.outer(np.linspace(0,1,numSteps), p2-p1)

class KinArm(object):

    def __init__(self,
                 win = None,
                 L = np.array([0.4,0.5]),
                 theta = np.radians(np.array([75,105])),
                 thetaLim = np.radians(np.array([9, 171])),
                 screenSize = (1000, 500)):

        self.name = 'KinArm'

        self.L = L

        self.thetaLim = thetaLim
        self.theta = theta

        self.xy = self.forwardKin(theta)

        # For problem 2
        self.workspacePointList = [] # will be a list of x-y coordinates (list of lists or list of tuples)
        self.workspaceEnvelopePointList = [] # will be a list of x-y coordinates (list of lists or list of tuples)
        self.outOfWorkspace = False

        # For problem 3
        self.stepNum = 0
        self.path = []

        pygame.init()  # It is harmless to call init() again if it has already been called before.

        self.screenSize = screenSize

        if win is None:
            pygame.display.init()
            self.win = pygame.display.set_mode(screenSize)
        else:
            self.win = win
            self.screenSize = pygame.display.get_surface().get_size()

        pygame.display.set_caption(self.name)

    @property
    def theta(self):
        return self.__theta

    @theta.setter
    def theta(self, newTheta):
        newTheta = self.thetaInRange(newTheta)
        self.__theta = newTheta

    def thetaInRange(self, newTheta):
        newNewTheta = np.minimum(np.maximum(newTheta, self.thetaLim[0]), self.thetaLim[1])
        if (newNewTheta[1] < newNewTheta[0]):
            newNewTheta[1] = newNewTheta[0]
        return newNewTheta

#=======================================================================================================================
#       START EDITING HERE
#=======================================================================================================================

# Problem 1
    def forwardKin(self, theta=None):
        if theta is None:
            theta = self.theta

        return np.array([np.sum(self.L * np.cos(theta)), np.sum(self.L * np.sin(theta))])


# Problem 2
    def workspace(self):

        resolution = 50

        for theta0 in np.linspace(0.0, np.pi, resolution):
            for theta1 in np.linspace(0.0, np.pi, resolution):
                joint_angles = np.array([theta0, theta1])
                if np.sum(self.thetaInRange(joint_angles) == joint_angles) == 2:
                    self.workspacePointList.append(self.forwardKin(joint_angles))



    def workspaceEnvelope(self):

        resolution = 100

        theta0_max, theta1_max = 0.0, 0.0
        theta0_min, theta1_min = np.pi, np.pi
        for theta0 in np.linspace(0.0, np.pi, resolution):
            for theta1 in np.linspace(0.0, np.pi, resolution):
                joint_angles = np.array([theta0, theta1])
                if (np.sum(self.thetaInRange(joint_angles) == joint_angles) == 2
                    and theta1 <= theta0):
                    if theta0 < theta0_min:
                        theta0_min = theta0
                    if theta0 > theta0_max:
                        theta0_max = theta0

                    if theta1 < theta1_min:
                        theta1_min = theta0
                    if theta1 > theta1_max:
                        theta1_max = theta1

        for theta0 in np.linspace(theta0_min, theta0_max, resolution):
            self.workspaceEnvelopePointList.append(self.forwardKin([theta0, theta1_max]))

        for theta1 in np.linspace(theta1_min, theta1_max, resolution):
            self.workspaceEnvelopePointList.append(self.forwardKin([theta0_min, theta1]))

        for theta0 in np.linspace(theta0_min, theta0_max, resolution * 2):
            self.workspaceEnvelopePointList.append(self.forwardKin([theta0, theta0]))


# Problem 3
    def makeCPath(self, goal):
        self.path = []
        self.numStep = 0

        goal_angles, out_of_workspace = self.inverseKin(goal)

        for pair in interpPoints(self.theta, goal_angles, 20):
            self.path.append(pair)

        self.numStep = 1


    def stepCPath(self):
        if self.numStep < len(self.path):
            self.theta = self.path[self.numStep]
            self.numStep += 1



# Problem 4
    def Jacobian(self):

        J = np.array([
              [-self.L[0] * np.sin(self.theta[0]), -self.L[1] * np.sin(self.theta[1]), ],
              [ self.L[0] * np.cos(self.theta[0]),  self.L[1] * np.cos(self.theta[1]), ]
            ])
        return J

    def stepJTranspose(self, goal):
        delta_p = goal - self.xy
        distance = np.linalg.norm(delta_p)

        alpha = 0.2
        delta_theta = alpha * np.matmul(np.transpose(self.Jacobian()), delta_p)

        self.theta += delta_theta
        self.xy = self.forwardKin(self.theta)

# Problem 5
    def stepJPinv(self, goal):
        delta_p = goal - self.xy
        distance = np.linalg.norm(delta_p)

        alpha = 0.1
        delta_theta = alpha * np.matmul(np.linalg.pinv(self.Jacobian()), delta_p)

        self.theta += delta_theta
        self.xy = self.forwardKin(self.theta)

# Problem 6
    def forceToTorque(self, F):
        alpha = 1
        T = alpha * np.matmul(np.linalg.pinv(self.Jacobian()), F)

        return T

    def impedanceRect(self, LRTB, gain=500):
        F = np.array([0.0, 0.0])

        L = LRTB[0]
        R = LRTB[1]
        T = LRTB[2]
        B = LRTB[3]

        if(self.xy[0] < L):
            F[0] = L - self.xy[0]
        if(self.xy[0] > R):
            F[0] = R - self.xy[0]
        if(self.xy[1] > T):
            F[1] = T - self.xy[1]
        if(self.xy[1] < B):
            F[1] = B - self.xy[1]

        return F * gain

# Problem 7
    def impedanceCircle(self, center, radius, gain=500):
        center_to_endpoint = center - self.xy
        distance = np.linalg.norm(center_to_endpoint)

        # if outside the ball or at exact center
        if distance > radius or distance <= 0:
            return np.array([0.0, 0.0]) # no force
        else:
            # unit vector from center to endpoint
            center_to_endpoint_u = center_to_endpoint / distance

            # the force is proportional to penetration depth
            penetration = distance - radius

            F = penetration * center_to_endpoint_u * gain
            return F


#=======================================================================================================================
#       STOP EDITING HERE
#=======================================================================================================================
    def inverseKin(self, p = None):
        self.outOfWorkspaceFlag = False

        if p is None:
            p = self.xy

        R = np.linalg.norm(p)
        C10 = (R**2 - self.L[0]**2 - self.L[1]**2)/(2 * self.L[0]*self.L[1])

        if np.abs(C10)>1:
            self.outOfWorkspaceFlag = True
            print("out of workspace")
            Rnorm = np.sqrt(np.sign(C10) * 2 * self.L[0]*self.L[1] +  self.L[0]**2 + self.L[1]**2)
            p = Rnorm*p/np.linalg.norm(p)
            R = np.linalg.norm(p)
            C10 = (R ** 2 - self.L[0] ** 2 - self.L[1] ** 2) / (2 * self.L[0] * self.L[1])

        if np.isclose(np.abs(C10),1.0):
            S10 = 0.0
        else:
            S10 = np.sqrt(1-C10**2)

        beta = np.arctan2(S10,C10)

        L2y = self.L[1]*np.sin(beta)
        L2x = self.L[0] + self.L[1]*np.cos(beta)
        alpha = np.arctan2(L2y, L2x)
        gamma = np.arctan2(p[1], p[0])

        theta = np.array([gamma-alpha, beta+gamma-alpha])

        if any((theta<self.thetaLim[0])|(theta>self.thetaLim[1])):
            self.outOfWorkspaceFlag = True

        return theta, self.outOfWorkspaceFlag

    def jointToEndVelocities(self, dTheta):
        return self.Jacobian() @ dTheta

    def endToJointVelocities(self, dp):
        return np.linalg.pinv(self.Jacobian()) @ dp

# Drawing methods
    def draw(self):
        
        theta = self.theta
        
        LINEWIDTH = 8
        pt0 = np.array([0,0])
        pt0b = np.array([0.25*self.L[0]*np.cos(theta[1]),0.25*self.L[0]*np.sin(theta[1])])

        pt1a = np.array([self.L[0]*np.cos(theta[0]),self.L[0]*np.sin(theta[0])])
        pt1b = pt1a + pt0b

        pt2 = self.forwardKin()

        pt0 = screenCoords(pt0, self.screenSize)
        pt0b = screenCoords(pt0b, self.screenSize)
        pt1a = screenCoords(pt1a, self.screenSize)
        pt1b = screenCoords(pt1b, self.screenSize)
        pt2 = screenCoords(pt2, self.screenSize)


        #print("%3.2f, %3.2f,%3.2f,%3.2f,%3.2f" % (pt0[1], pt0b[1], pt1a[1], pt1b[1], pt2[1]))

        pygame.draw.line(self.win, DEEPBLUE, pt0, pt0b, LINEWIDTH)
        pygame.draw.line(self.win, DEEPBLUE, pt1a, pt2, LINEWIDTH)

        pygame.draw.line(self.win, NAVY, pt0, pt1a, LINEWIDTH)
        pygame.draw.line(self.win, NAVY, pt0b, pt1b, LINEWIDTH)

        pygame.draw.circle(self.win, NAVY, pt0, 6)
        pygame.draw.circle(self.win, DEEPBLUE, pt0, 8, 3)

        pygame.draw.circle(self.win, NAVY, pt0b, 6)
        pygame.draw.circle(self.win, NAVY, pt1a, 6)
        pygame.draw.circle(self.win, NAVY, pt1b, 6)
        pygame.draw.circle(self.win, BLACK, pt2, 6)

    def drawWorkspace(self):
        for p in self.workspacePointList:
            sp = screenCoords(p, self.screenSize)
            pygame.draw.circle(self.win, WHITE, sp, 3)

    def drawWorkspaceEnvelope(self):
        for p in self.workspaceEnvelopePointList:
            sp = screenCoords(p, self.screenSize)
            pygame.draw.circle(self.win, WHITE, sp, 3)
