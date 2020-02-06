"""
HapkitViz.py

A visualisation script to be used with Hapkit (Okamura, Stanford U) and hapkit_399.ino (Roth, IU)

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
GOLD = (240, 195, 10)
CORAL = (255,125,80)

''' 
----------------------------------------------------------------------
A function to convert normalized coordinates to pixel coordinates
----------------------------------------------------------------------
'''

def norm2pix(x=np.zeros(1), y=np.zeros(1), screenSize=None):
    if screenSize is None:
        W,H = pygame.display.get_surface.get_height()

    W = screenSize[0]
    H = screenSize[1]

    xPix = np.rint((0.4 * x + 0.5)*W)
    yPix = np.rint((-0.8 * y + 0.9)*H)

    return list(zip(xPix,yPix))


''' 
----------------------------------------------------------------------
Classes for the visual terrain, the end effector, and the simulation
----------------------------------------------------------------------
'''

class Terrain(object):

    def __init__(self, tx, ty, path, color=GRAY75,  win=None, name = ''):

        self.x = tx
        self.y = ty
        self.color = color

        self.path = path
        self.name = name

        try:
            self.setWindow(win)
            self.makePoly()
        except:
            print('Warning: No display has been initiated for terrain: %s.' %(self.name))


    def setWindow(self, win=None):
        if win is None:
            win = pygame.display.get_surface()

        self.win = win
        self.screenSize = win.get_size()

    def makePoly(self):
        X = np.append([-1.2, -1.2], self.x)
        X = np.append(X, [1.2, 1.2, -1.2])

        Y = np.append([-0.2, self.y[0]], self.y)
        Y = np.append(Y, [self.y[-1],-0.2, -0.2])

        self.polyPtsList = norm2pix(X, Y, self.screenSize)

    def draw(self):

        pygame.draw.polygon(self.win, self.color, self.polyPtsList)

        pt1 = norm2pix(np.array([-1.0, -1.0]), np.array([self.y[0], -1.2]), self.screenSize)
        pt2 = norm2pix(np.array([1.0, 1.0]), np.array([self.y[-1], -1.2]), self.screenSize)

        pygame.draw.line(self.win, CORAL, pt1[0], pt1[1], 4)
        pygame.draw.line(self.win, CORAL, pt2[0], pt2[1], 4)


class Cursor(object):

    def __init__(self, tx, ty, color=GOLD, win = None):

        self.tx = tx
        self.ty = ty
        self.color = color

        try:
            self.setWindow(win)
        except:
            print('Warning: No display has been initiated for cursor.')

    def setWindow(self, win=None):
        if win is None:
            win = pygame.display.get_surface()

        self.win = win
        self.screenSize = win.get_size()

    def draw(self, Htrans):

        sx = Htrans[0]
        sy = Htrans[1]
        theta = Htrans[2]

        px = np.cos(theta)*self.tx - np.sin(theta)*self.ty + sx
        py = np.sin(theta)*self.tx + np.cos(theta)*self.ty + sy

        P = norm2pix(px, py, self.screenSize)
        pygame.draw.polygon(self.win, self.color, P)


'''
Default end effector
'''
phi = np.pi/6 * np.arange(-1, 7.01, 1)

cursorOutlineX = 0.05*np.pad(np.cos(phi), (1,1), 'constant')
cursorOutlineY = 0.05*np.pad(1.5+np.sin(phi), (1,1), 'constant')

effector = Cursor(cursorOutlineX, cursorOutlineY)

'''
Mode 0: Flat surface
'''
terr0_x = np.array([-1,-1])
terr0_y = np.array([0,0])
path0 = lambda p: (p,0,0)

terr0 = Terrain(terr0_x, terr0_y, path0, name = 'Flat')



class Simulation(object):

    def __init__(self, name = 'Hapkit Visualizer', win = None, terrain=terr0, cursor=effector, screenSize = (1000, 500)):

        self.name = name

        pygame.init()       # It is harmless to call init() again if it has already been called before.

        self.screenSize = screenSize

        if win is None:
            pygame.display.init()
            self.win = pygame.display.set_mode(screenSize)
        else:
            self.win = win

        pygame.display.set_caption(self.name)

        self.defaultTerrain = terrain

        self.setTerrain(terrain)
        self.setCursor(cursor)

    def setTerrain(self, terrain):

        if terrain is None:
            terrain = self.defaultTerrain

        self.terrain = terrain
        self.terrain.setWindow(self.win)
        self.terrain.makePoly()

    def setCursor(self, cursor):
        self.cursor = cursor
        self.cursor.setWindow(self.win)

    def draw(self, p):
        self.terrain.draw()
        S = self.terrain.path(p)
        self.cursor.draw(S)