"""
HapkitViz.py

A visualisation script to be used with Hapkit (Okamura, Stanford U) and hapkit_399.ino (Roth, IU)

Eatai Roth, 2020
"""

from HapkitViz import *
import serial.tools.list_ports
import serial
import struct
from time import sleep

import os


''' 
----------------------------------------------------------------------
Defining the different visual landscapes
----------------------------------------------------------------------
'''

screenSize = screenW, screenH = (1000,500)

CWD = os.getcwd()
dataDir = os.path.join(CWD, 'Data')

COM_PORT = None
#COM_PORT = '/dev/cu.usbserial-AL05J160'

'''
Mode 1: Wall to right
'''
terr1_x = np.array([-1, 0.5, 0.5, 1])
terr1_y = np.array([0, 0, 1, 1])

path1 = lambda p: (min(p,0.45), 0, 0)

terr1 = Terrain(terr1_x, terr1_y, path1, name='The Wall')


'''
Mode 2: Half Circle Valley
'''
phi = np.pi/2 * np.arange(-1,1.01,0.05)

terr2_x = np.sin(phi)
terr2_y = 1-1*np.cos(phi)
path2 = lambda p: (np.sin(p*np.pi/2), 1-np.cos(p*np.pi/2), p*np.pi/2)

terr2 = Terrain(terr2_x, terr2_y, path2, name = 'Valley')

'''
Mode 3: Half Circle Hillock
'''
terr3_x = np.sin(phi)
terr3_y = np.cos(phi)
path3 = lambda p: (np.sin(p*np.pi/2), np.cos(p*np.pi/2), -p*np.pi/2)

terr3 = Terrain(terr3_x, terr3_y, path3, name = 'Hillock')

'''
Mode 4: Notch
'''
terr4_x = np.array([-1, -0.05, 0, 0.05, 1])
terr4_y = np.array([0, 0, -0.05, 0, 0])
path4 = lambda p: (p, min(np.abs(p)-0.05,0), 0)

terr4 = Terrain(terr4_x, terr4_y, path4, name = 'Notch')

'''
Mode 5: Bumpity
'''

terr5_x = np.arange(-1,1.01,0.05)
terr5_y = np.zeros_like(terr5_x)
terr5_y[1::2]=-0.05
path5 = lambda p: (p, min(np.abs(p%0.1-0.05)-0.05,0), 0)

terr5 = Terrain(terr5_x, terr5_y, path5, name = 'Bumpity')

simList = [None, terr1, terr2, terr3, terr4, terr5]



''' 
----------------------------------------------------------------------
Connecting to hapkit
----------------------------------------------------------------------
'''



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
FPS = 25

pygame.init()

screen = pygame.display.set_mode(screenSize)
pygame.display.set_caption("Hapkit Visualizer")
clock = pygame.time.Clock()


''' 
----------------------------------------------------------------------
Some setup for the visualization
----------------------------------------------------------------------
'''
fontList = pygame.font.get_fonts()
font = pygame.font.SysFont(fontList[6], 30, False, False)

newMode = 0
mode = 0

sim = Simulation(screenSize = screenSize)

pos = 0.0
step = 0
stepSize = 0.01

stillPlaying = True
resetFlag = False
dataLoggingFlag = False

def startLog(dataDirectory, filenameroot):
    if not os.path.exists(dataDirectory):
        os.mkdir(dataDirectory)
    k=0
    while k<100:
        fileString = os.path.join(dataDirectory, '%s_%02d.txt'%(filenameroot,k))
        if os.path.isfile(fileString):
            k+=1
        else:
            fid = open(fileString, 'w+')
            break

    return fid, '%s_%02d.txt'%(filenameroot,k)

def stopLog(fid):
    fid.close()

''' 
----------------------------------------------------------------------
The visualization loop
----------------------------------------------------------------------
'''
while stillPlaying:

    if dataLoggingFlag:
        dataLoggingFlag = False
        stopLog(fid)

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

    sim.setTerrain(simList[mode])

    if not keyboardMode:
        hapkit.write(bytearray(str(mode), 'utf8'))

    running = True

    titleText = font.render(sim.terrain.name, 1, BLACK)

    while running:

        clock.tick_busy_loop(FPS)

        ''' 
        Check for quitting or starting/stopping data logging
        '''

        keys = pygame.key.get_pressed()

        for event in pygame.event.get():
            if event.type == pygame.QUIT or keys[pygame.K_ESCAPE]:   # User closes window
                if dataLoggingFlag:
                    stopLog(fid)
                pygame.quit()
            elif event.type == pygame.KEYDOWN:
                if event.key==pygame.K_d and not dataLoggingFlag:
                    dataLoggingFlag = True
                    t0_log = pygame.time.get_ticks()
                    print('Data Logging ON')
                    fid, dataFileName = startLog(dataDir, sim.terrain.name)
                elif event.key==pygame.K_d and dataLoggingFlag:
                    dataLoggingFlag = False
                    print('Data Logging OFF')
                    stopLog(fid)

        ''' 
        Check for mode switch
        '''

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

        if dataLoggingFlag:
            t_elapsed = float(pygame.time.get_ticks() - t0_log) / 1000.0
            fid.write('%7.3f, %6.2f, %4d\n' % (t_elapsed, thetaH, F))

        infoText = font.render("Angle: %2.1f deg   Force: %d"%(thetaH, F), 1, BLACK)

        screen.blit(infoText, (600,15))

        if dataLoggingFlag:
            infoText2 = font.render("Data Logging: %s" % dataFileName, 1, CORAL)
            screen.blit(infoText2, (600, 45))


        pygame.display.flip()

