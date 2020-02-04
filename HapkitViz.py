"""
HapkitViz.py

A visualisation script to be used with Hapkit (Okamura, Stanford U) and hapkit_399.ino (Roth, IU)

Eatai Roth, 2020
"""

import pygame
import numpy as np
import serial.tools.list_ports
import serial
import struct
from time import sleep

''' 
----------------------------------------------------------------------
Connecting to hapkit
----------------------------------------------------------------------
'''

COM_PORT = "/dev/ttyUSB0"

keyboardMode = False

if COM_PORT is None:
    ports = list(serial.tools.list_ports.comports())
    for p in \
        ports:
        print(p)

    COM_PORT = input("\nIn your terminal window, you should see a list of serial devices. \nCopy the one that corresponds to your hapkit here [ENTER - Keyboard debug mode]: ")

    if COM_PORT=='':
        keyboardMode = True
        print('KEYBOARD DEBUG MODE')

if not keyboardMode:
    hapkit = serial.Serial(port = COM_PORT, baudrate = 230400, timeout = 1, inter_byte_timeout=0.2)
    sleep(1)

packetSize = 8
packetString = '<fi'

''' 
----------------------------------------------------------------------
Parameter definitions for the visual display
----------------------------------------------------------------------
'''
FPS = 40

pygame.init()

screenSize = screenW, screenH = (1000,500)
screenCenter = screenXC, screenYC = screenW/2, screenH/2

BLACK = (0,0,0)
WHITE = (255, 255, 255)

GRAY75 = (63, 63, 63)
GRAY50 = (127, 127, 127)
GRAY25 = (191, 191, 191)

PALEBLUE = (200, 240, 255)
GOLD = (240, 195, 10)
CORAL = (255,125,80)


screen = pygame.display.set_mode(screenSize)
pygame.display.set_caption("Hapkit Visualizer")
clock = pygame.time.Clock()


''' 
----------------------------------------------------------------------
A function to convert normalized coordinates to pixel coordinates
----------------------------------------------------------------------
'''

def norm2pix(win, x=np.zeros(1), y=np.zeros(1)):
    H = win.get_height()
    W = win.get_width()

    xPix = np.rint((0.4 * x + 0.5)*W)
    yPix = np.rint((-0.8 * y + 0.9)*H)

    return list(zip(xPix,yPix))


''' 
----------------------------------------------------------------------
Classes for the visual terrain, the end effector, and the simulation
----------------------------------------------------------------------
'''
class Terrain(object):

    def __init__(self, win, tx, ty, path, color=GRAY75):

        self.x = tx
        self.y = ty
        self.color = color
        self.win = win

        self.screenW = win.get_width()
        self.screenH = win.get_height()
        self.makePoly()

        self.path = path

    def makePoly(self):
        X = np.append([-1.2, -1.2], self.x)
        X = np.append(X, [1.2, 1.2, -1.2])

        Y = np.append([-0.2, self.y[0]], self.y)
        Y = np.append(Y, [self.y[-1],-0.2, -0.2])

        self.polyPtsList = norm2pix(self.win, X, Y)

    def draw(self):

        pygame.draw.polygon(self.win, self.color, self.polyPtsList)

        pt1 = norm2pix(self.win, np.array([-1.0, -1.0]), np.array([self.y[0], -1.2]))
        pt2 = norm2pix(self.win, np.array([1.0, 1.0]), np.array([self.y[-1], -1.2]))

        pygame.draw.line(self.win, CORAL, pt1[0], pt1[1], 4)
        pygame.draw.line(self.win, CORAL, pt2[0], pt2[1], 4)


class Cursor(object):

    def __init__(self, win, tx, ty, color=GOLD):

        self.tx = tx
        self.ty = ty
        self.color = color

        self.win = win
        self.screenW = win.get_width()
        self.screenH = win.get_height()

    def draw(self, Htrans):

        sx = Htrans[0]
        sy = Htrans[1]
        theta = Htrans[2]

        px = np.cos(theta)*self.tx - np.sin(theta)*self.ty + sx
        py = np.sin(theta)*self.tx + np.cos(theta)*self.ty + sy

        P = norm2pix(self.win, px, py)
        pygame.draw.polygon(self.win, self.color, P)


'''
Default end effector
'''
phi = np.pi/6 * np.arange(-1, 7.01, 1)

cursorOutlineX = 0.05*np.pad(np.cos(phi), (1,1), 'constant')
cursorOutlineY = 0.05*np.pad(1.5+np.sin(phi), (1,1), 'constant')

effector = Cursor(screen, cursorOutlineX, cursorOutlineY)

'''
Mode 0: Flat surface
'''
terr0_x = np.array([-1,-1])
terr0_y = np.array([0,0])
path0 = lambda p: (p,0,0)

terr0 = Terrain(screen, terr0_x, terr0_y, path0)



class Simulation(object):

    def __init__(self, name = '', terrain=terr0, cursor=effector):
        self.terrain = terrain
        self.cursor = cursor
        self.name = name


    def draw(self, p):
        self.terrain.draw()
        S = self.terrain.path(p)
        self.cursor.draw(S)


''' 
----------------------------------------------------------------------
Designing some visual landscapes
----------------------------------------------------------------------
'''

'''
Mode 1: Wall to right
'''
terr1_x = np.array([-1, 0.5, 0.5, 1])
terr1_y = np.array([0, 0, 1, 1])

path1 = lambda p: (min(p,0.45), 0, 0)

terr1 = Terrain(screen, terr1_x, terr1_y, path1)


'''
Mode 2: Half Circle Valley
'''
phi = np.pi/2 * np.arange(-1,1.01,0.05)

terr2_x = np.sin(phi)
terr2_y = 1-1*np.cos(phi)
path2 = lambda p: (np.sin(p*np.pi/2), 1-np.cos(p*np.pi/2), p*np.pi/2)

terr2 = Terrain(screen, terr2_x, terr2_y, path2)

'''
Mode 3: Half Circle Hillock
'''
terr3_x = np.sin(phi)
terr3_y = np.cos(phi)
path3 = lambda p: (np.sin(p*np.pi/2), np.cos(p*np.pi/2), -p*np.pi/2)

terr3 = Terrain(screen, terr3_x, terr3_y, path3)

'''
Mode 4: Divot
'''
terr4_x = np.array([-1, -0.05, 0, 0.05, 1])
terr4_y = np.array([0, 0, -0.05, 0, 0])
path4 = lambda p: (p, min(np.abs(p)-0.05,0), 0)

terr4 = Terrain(screen, terr4_x, terr4_y, path4)

'''
Mode 5: Bumpitees
'''

terr5_x = np.arange(-1,1.01,0.05)
terr5_y = np.zeros_like(terr5_x)
terr5_y[1::2]=-0.05
path5 = lambda p: (p, min(np.abs(p%0.1-0.05)-0.05,0), 0)

terr5 = Terrain(screen, terr5_x, terr5_y, path5)


simList = [Simulation('Flat'),\
           Simulation('The Wall', terr1), \
           Simulation('Valley', terr2),
           Simulation('Hillock', terr3),
           Simulation('Notch', terr4),
           Simulation('Bumpity', terr5)
           ]



''' 
----------------------------------------------------------------------
Some setup for the visualization
----------------------------------------------------------------------
'''

fontList = pygame.font.get_fonts()
font = pygame.font.SysFont(fontList[0], 30, False, False)

newMode = 0
mode = 0

sim = simList[mode]

pos = 0.0
step = 0
stepSize = 0.01

stillPlaying = True
resetFlag = False



''' 
----------------------------------------------------------------------
The visualization loop
----------------------------------------------------------------------
'''
while stillPlaying:

    if resetFlag:
        mode = 0
        newMode = 0
        resetFlag = False
        hapkit.close()
        input("Press enter to reconnect hapkit...")
        print('Reconnecting serial...')
        hapkit = serial.Serial(port=COM_PORT, baudrate=230400, timeout=1, inter_byte_timeout=0.2)
        pos = 0
        sim.draw(pos)
        print('Ready to go.')
        sleep(4)

    sim = simList[mode]

    if not keyboardMode:
        hapkit.write(bytearray(str(mode), 'utf8'))

    running = True

    titleText = font.render(sim.name, 1, BLACK)

    while running:

        clock.tick(FPS)

        keys = pygame.key.get_pressed()

        for event in pygame.event.get():
            if event.type == pygame.QUIT or keys[pygame.K_ESCAPE]:   # User closes window
                pygame.quit()

        if keys[pygame.K_0]:
            newMode = 0
        elif keys[pygame.K_1]:
            newMode = 1
        elif keys[pygame.K_2]:
            newMode = 2
        elif keys[pygame.K_3]:
            newMode = 3
        elif keys[pygame.K_4]:
            newMode = 4
        elif keys[pygame.K_5]:
            newMode = 5
        elif keys[pygame.K_k]:
            keyboardMode = not keyboardMode
        elif keys[pygame.K_r]:
            resetFlag = True
            break

        if (newMode != mode):
            mode = newMode
            break


        if keyboardMode:
            if keys[pygame.K_RIGHT] and not keys[pygame.K_LEFT]:
                step = 1
            elif not keys[pygame.K_RIGHT] and keys[pygame.K_LEFT]:
                step = -1
            else:
                step = 0
            pos += (stepSize * step)

        else:
            hapkit.write(b'g')
            try:
                data = struct.unpack(packetString, hapkit.read(packetSize))
                thetaH = data[0]
                F = data[1]
                pos = thetaH / 30.0

            except:
                resetFlag = True
                break


        pos = min(max(pos, -1), 1)

        screen.fill(PALEBLUE)
        sim.draw(pos)
        screen.blit(titleText, (15,15))

        if keyboardMode:
            thetaH = pos*30
            F = 0

        infoText = font.render("Angle: %2.1f deg   Force: %d"%(thetaH, F), 1, BLACK)
        screen.blit(infoText, (600,15))


        pygame.display.flip()

