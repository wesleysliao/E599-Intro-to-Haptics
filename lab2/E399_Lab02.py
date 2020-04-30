from KinArm import *


def ASWD(keys, stepSize = 2):
    if keys[pygame.K_a] and not keys[pygame.K_d]:
        lr = stepSize
    elif not keys[pygame.K_a] and keys[pygame.K_d]:
        lr = -1 * stepSize
    else:
        lr = 0

    if keys[pygame.K_w] and not keys[pygame.K_s]:
        ud = stepSize
    elif not keys[pygame.K_w] and keys[pygame.K_s]:
        ud = -1 * stepSize
    else:
        ud = 0
    return lr, ud

def ASWDtext():
    asdwText = font.render("Use AD and WS keys to move arm", 1, NAVY)
    screen.blit(asdwText, (15, 450))

def clickText():
    asdwText = font.render("Click to set desired arm position", 1, NAVY)
    screen.blit(asdwText, (15, 450))

def moveText():
    moveText = font.render("Use mouse to position KinArm", 1, NAVY)
    screen.blit(moveText, (15, 450))

def drawVector(tail, head, color = CRIMSON):
    sTail = screenCoords(tail, screenSize)
    sHead = screenCoords(head, screenSize)
    pygame.draw.line(screen, color, sTail, sHead, 3)

def drawTrace(trace, color=GOLD, size = 3):
    for t in trace:
        st = screenCoords(t, screenSize)
        pygame.draw.circle(screen, color, st, size)

''' 
----------------------------------------------------------------------
Some setup for the visualization
----------------------------------------------------------------------
'''
pygame.init()

screenSize = (1000,500)
screen = pygame.display.set_mode(screenSize)
pygame.display.set_caption("KinArm Visualizer")

FPS = 40
clock = pygame.time.Clock()


fontList = pygame.font.get_fonts()
font = pygame.font.SysFont(fontList[6], 18, False, False)
titleFont = pygame.font.SysFont(fontList[6], 24, True, True)

KA = KinArm(win=screen)
KA.draw()

mode = 1
newMode = 1

titleText = '1. Forward Kinematics'

sp = None

''' 
----------------------------------------------------------------------
The visualization loop
----------------------------------------------------------------------
'''

while True:
    sp = screenCoords(KA.xy, screenSize)
    outOfWorkspaceFlag = False
    newModeFlag = True
    numSteps = 20

    pSteps = [KA.xy]
    xyTracer = []
    counter = np.inf

    print("here again")

    while True:

        clock.tick(FPS)

        screen.fill(PALEBLUE)

        keys = pygame.key.get_pressed()
        events = pygame.event.get()

        KA.drawWorkspaceEnvelope()

        for event in events:
            if event.type == pygame.QUIT or keys[pygame.K_ESCAPE]:   # User closes window
                pygame.quit()

        if keys[pygame.K_1]:
            newMode = 1
            titleText = '1. Forward Kinematics'
        elif keys[pygame.K_2]:
            newMode = 2
            titleText = '2. Mapping the Workspace'
        elif keys[pygame.K_3]:
            newMode = 3
            titleText = '3. Trajectories in C-space'
        elif keys[pygame.K_4]:
            newMode = 4
            titleText = '4. Trajectories using J transpose'
        elif keys[pygame.K_5]: # Trajectories using J Pinv
            newMode = 5
            titleText = '5. Trajectories using J pseudo-inverse'
        elif keys[pygame.K_6]: # ExtraCred in Box
            newMode = 6
            titleText = '6. Impedance law, Stay in the Box'
        elif keys[pygame.K_7]:
            newMode = 7
            titleText = '7. Impedance law, Touch the Ball'


        if (newMode != mode):
            mode = newMode
            newModeFlag = True
            print('NEW MODE')
            break


        if mode == 1:
            newModeFlag = False
            ASWDtext()

            stepSize = 0.02*np.pi
            thetaStep0, thetaStep1 = ASWD(keys, stepSize)
            KA.theta += np.array([thetaStep0, thetaStep1])

        if mode==2:

            if newModeFlag:
                KA.workspace()
                KA.workspaceEnvelope()
                newModeFlag = False

            ASWDtext()

            KA.drawWorkspace()

            stepSize = 0.02 * np.pi
            thetaStep0, thetaStep1 = ASWD(keys, stepSize)
            KA.theta += np.array([thetaStep0, thetaStep1])


        if mode==3:
            if newModeFlag:
                goalPos = None
                spGoal = None
                newModeFlag = False

            clickText()

            for event in events:
                if event.type==pygame.MOUSEBUTTONUP:
                    spGoal = pygame.mouse.get_pos()
                    goalPos = inverseScreenCoords(spGoal, screenSize)
                    KA.makeCPath(goalPos)
                    xyTracer = []

            if goalPos is not None:
                if KA.outOfWorkspace:
                    pygame.draw.circle(screen, CRIMSON, spGoal, 8, 2)
                    oowText = font.render("out of workspace", 1, CRIMSON)
                    screen.blit(oowText, (600, 400))
                else:
                    pygame.draw.circle(screen, GOLD, spGoal, 8)

            try:
                KA.stepCPath()
                xyTracer.append(KA.forwardKin())
                drawTrace(xyTracer)

            except:
                print('No path set')

        if mode == 4:
            if newModeFlag:
                goalPos = KA.xy
                spGoal = None
                xyTracer = []
                newModeFlag = False

            clickText()

            for event in events:
                if event.type == pygame.MOUSEBUTTONUP:
                    print('hello')
                    spGoal = pygame.mouse.get_pos()
                    goalPos = inverseScreenCoords(spGoal, screenSize)
                    xyTracer = []
            if spGoal is not None:
                if outOfWorkspaceFlag:
                    pygame.draw.circle(screen, CRIMSON, spGoal, 8, 2)
                    oowText = font.render("out of workspace", 1, CRIMSON)
                    screen.blit(oowText, (600, 400))
                else:
                    pygame.draw.circle(screen, GOLD, spGoal, 8)

            KA.stepJTranspose(goalPos)
            xyTracer.append(KA.xy)

            drawTrace(xyTracer)

        if mode == 5:
            if newModeFlag:
                goalPos = KA.xy
                spGoal = None
                xyTracer = []
                newModeFlag = False

            clickText()



            for event in events:
                if event.type == pygame.MOUSEBUTTONUP:
                    print('hello')
                    spGoal = pygame.mouse.get_pos()
                    goalPos = inverseScreenCoords(spGoal, screenSize)
                    xyTracer = []
            if spGoal is not None:
                if outOfWorkspaceFlag:
                    pygame.draw.circle(screen, CRIMSON, spGoal, 8, 2)
                    oowText = font.render("out of workspace", 1, CRIMSON)
                    screen.blit(oowText, (600, 400))
                else:
                    pygame.draw.circle(screen, GOLD, spGoal, 8)

            KA.stepJPinv(goalPos)
            xyTracer.append(KA.xy)

            drawTrace(xyTracer)

        if mode == 6:
            if newModeFlag:
                rectCenter = np.array([0, 0.7])

                rectWH = np.array([0.4, 0.1])
                rectLU = rectCenter+0.5*rectWH*[-1,1]

                sRectHW = METERS2PIX*rectWH
                sRectUL = screenCoords(rectLU, screenSize)
                sRect = np.hstack((sRectUL, sRectHW))
                LRTB = [rectLU[0], rectLU[0]+rectWH[0], rectLU[1], rectLU[1]-rectWH[1]]
                newModeFlag = False

            pygame.draw.rect(screen, GRAY50, [0, 0, screenSize[0], screenSize[1]])
            pygame.draw.rect(screen, PALEBLUE, sRect)

            moveText()

            p = inverseScreenCoords(pygame.mouse.get_pos(), screenSize)
            KA.theta, KA.outOfWorkspaceFlag = KA.inverseKin(p)
            KA.xy = KA.forwardKin()

            F = KA.impedanceRect(LRTB)
            tail = KA.xy - 0.01*F
            drawVector(tail, KA.xy)

            T = KA.forceToTorque(F)

            forceText = "Force x: %3.2f N, Force y: %3.2f N"% (F[0], F[1])
            screen.blit(font.render(forceText, 1, BLACK), (650, 40))

            torqueText = "Torque 1: %3.2f N m, Torque 2: %3.2f N m" % (T[0], T[1])
            screen.blit(font.render(torqueText, 1, BLACK), (650, 60))

        if mode == 7:
            if newModeFlag:
                circleCenter = [0, 0.7]
                circleRadius = 0.1

                sCircleCenter = screenCoords(circleCenter, screenSize)
                sCircleRadius = int(METERS2PIX*circleRadius)

                newModeFlag = False

            pygame.draw.circle(screen, GRAY50, sCircleCenter, sCircleRadius)
            moveText()


            p = inverseScreenCoords(pygame.mouse.get_pos(), screenSize)
            KA.theta, KA.outOfWorkspaceFlag  = KA.inverseKin(p)
            KA.xy = KA.forwardKin()

            F = KA.impedanceCircle(circleCenter, circleRadius)
            tail = KA.xy - 0.01*F
            drawVector(tail, KA.xy)

            T = KA.forceToTorque(F)

            forceText = "Force x: %3.2f N, Force y: %3.2f N" % (F[0], F[1])
            screen.blit(font.render(forceText, 1, BLACK), (700, 40))

            torqueText = "Torque 1: %3.2f N m, Torque 2: %3.2f N m" % (T[0], T[1])
            screen.blit(font.render(torqueText, 1, BLACK), (700, 60))

        KA.draw()

        infoText = titleFont.render(titleText, 1, BLACK)
        angleText = "Angle 1: %3.2f deg, Angle 2: %3.2f deg" % (np.degrees(KA.theta[0]), np.degrees(KA.theta[1]))
        infoText2 = font.render(angleText,1,BLACK)
        screen.blit(infoText, (15,15))
        screen.blit(infoText2, (650, 15))

        pygame.display.flip()



