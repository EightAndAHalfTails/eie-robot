#44:33:4c:6c:c1:92
#http://129.31.222.18:9000/

from BrickPi import *
from time import sleep, time
from math import *
from random import gauss, uniform
import sys
from particleDataStructures import *

sonar       = PORT_2
leftMotor   = PORT_C
rightMotor  = PORT_B
leftBumper  = PORT_1
rightBumper = PORT_2

wheelRadius = 1.8#cm
wheelSeparation = 20.0#cm

startX = 100#cm
startY = 100#cm
startA = 0#degrees

class orientation:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.a = 0

    def moveForward(self, d):
        sd = d/40.0
        meanR = -d/20.0
        e = gauss(0, sd)
        f = gauss(meanR, sd)

        self.x += (d + e)*cos(radians(self.a))
        self.y += (d + e)*sin(radians(self.a))
        self.a += f
        self.a = self.a

    def rotate(self, b):
        sd = 2.0
        g = gauss(0, sd)

        self.a += b + g

class particle:
    def __init__(self, w):
        self.x = orientation()
        self.w = w

    def setTo(self, x, y, a):
        self.x.x = x
        self.x.y = y
        self.x.a = a
        
    def moveForward(self, d):
        self.x.moveForward(d)

    def rotate(self, a):
        self.x.rotate(a)

class particleSet:
    def __init__(self, n):
        self.particles = list()
        for i in range(n):
            self.particles.append(particle(1.0/n))
     
    def setTo(self, x, y, a):
        weight = 1.0/len(self.particles)
        for p in self.particles:
            p.setTo(x,y,a)
            p.w = weight
       
    def moveForward(self, d):
        for p in self.particles:
            p.moveForward(d)

    def rotate(self, a):
        for p in self.particles:
            p.rotate(a)
    
    def estimatePosition(self):
        (xx, yy, sina, cosa) = (0,0,0,0)
        for p in self.particles:
            xx += p.x.x
            yy += p.x.y
            sina += sin(radians(p.x.a))
            cosa += cos(radians(p.x.a))
        size = len(self.particles)
        return (xx/size, yy/size, degrees(atan2(sina/size, cosa/size)))

    def reweight(self, sonar):
        for p in self.particles:
            p.w *= calculateLikelihood(p.x.x, p.x.y, p.x.a, sonar)

    def normalise(self):
        totalWeights = sum([p.w for p in self.particles])
        for p in self.particles:
            p.w /= totalWeights

    def resample(self):
        N = len(self.particles)
        cdf = [sum([p.w for p in self.particles][:end]) for end in range(N)]
        resampled = []
        for i in range(N):
            copyIndex = 0
            urand = uniform(0, 1)
            for index in range(len(cdf)):
                if cdf[index] > urand:
                    copyIndex = index #first element in cdf with weight greater than urand
                    break
            resampled.append(particle(1.0/N))
            resampled[i].setTo( self.particles[copyIndex].x.x, 
                                self.particles[copyIndex].x.y,
                                self.particles[copyIndex].x.a ) # deep copy
        self.particles = resampled

def printParticles(P):
    print "drawParticles:" + str([(int(p.x.x)+startX,
                                   int(p.x.y)+startY,
                                   int((p.x.a+startA)%360)) for p in P.particles])


def printSquare(d):
    global startX, startY
    p0 = (startX    , startY    )
    p1 = (startX + d, startY    )
    p2 = (startX + d, startY + d)
    p3 = (startX    , startY + d)
    print "drawLine:" + str(p0+p1)
    print "drawLine:" + str(p1+p2)
    print "drawLine:" + str(p2+p3)
    print "drawLine:" + str(p3+p0)

pose = particleSet(100)
pose.setTo(84, 30, 0)

def initialiseDiffDriveRobot():
    global leftMotor
    global rightMotor
    BrickPiSetup()
    BrickPi.MotorEnable[leftMotor] = 1 # set up Motors
    BrickPi.MotorEnable[rightMotor] = 1
    BrickPi.SensorType[leftBumper] = TYPE_SENSOR_TOUCH # set up touch sensors
    BrickPi.SensorType[rightBumper] = TYPE_SENSOR_TOUCH
    BrickPi.SensorType[sonar] = TYPE_SENSOR_ULTRASONIC_CONT # set up sonar
    BrickPiSetupSensors()
    BrickPi.Timeout = 5000 # stop motors after 10 seconds
    BrickPiSetTimeout()

def setLeftMotor(power):
    global leftMotor
    BrickPi.MotorSpeed[leftMotor] = int(power)
    BrickPiUpdateValues()

def setRightMotor(power):
    global rightMotor
    BrickPi.MotorSpeed[rightMotor] = int(power)
    BrickPiUpdateValues()

def leftCrash():
    global leftBumper
    BrickPiUpdateValues()
    return BrickPi.Sensor[leftBumper] == 1

def rightCrash():
    global rightBumper
    BrickPiUpdateValues()
    return BrickPi.Sensor[rightBumper] == 1

def goForwardsWhileAvoiding():
    global leftMotor
    global rightMotor

    while(True):
        # set motor speed
        # accelerateToSpeed(50) # too unresponsive
        setLeftMotor(150)
        setRightMotor(150)
        BrickPiUpdateValues()
        if leftCrash():
            stop()
            avoidRight()
        if rightCrash():
            stop()
            avoidLeft()

def avoidRight():
    goDistance(-10)
    rotate(90)
    goDistance(30)
    rotate(90, clockwise=False)
    
def avoidLeft():
    goDistance(-10)
    rotate(90, clockwise=False)
    goDistance(30)
    rotate(90)

def readSonar():
    global sonar
    BrickPiUpdateValues()
    return BrickPi.Sensor[sonar]

def keepDistance(desiredDistance):
    sonarMountedOnFront = False
    while(True):
        distance = readSonar()
        error = desiredDistance - distance
        gain = -10
        gain = gain if sonarMountedOnFront else -gain
        speed = gain * error
        print "Distance = {} - {} = {} Set speed to {}".format(desiredDistance, distance, error, speed)
#        accelerateToSpeed(speed) # too unresponsive
        setLeftMotor(speed)
        setRightMotor(speed)

def followRightWall(desiredDistance=30, desiredSpeed=150):
    while(True):
        dist = readSonar()
        error = desiredDistance - dist
#        gain = 2.0
        gain = 10.0

        dv = gain*error

        if dv > 255 - desiredSpeed:
            dv = 255 - desiredSpeed

        if dv < -255 + desiredSpeed:
            dv = -255 + desiredSpeed

        setLeftMotor(desiredSpeed - dv)
        setRightMotor(desiredSpeed + dv)

        print "Distance = {}. Turning {}".format(error, "left" if error>0 else "right")
        print "Speeds = ({}, {})".format(desiredSpeed-dv, desiredSpeed+dv)
        print "==================================="

def goDistance(targetDistance, desiredSpeed=40):
    if desiredSpeed < 0:
        raise ValueError

    forwards = (targetDistance > 0)

    # Initialisation
    global leftMotor
    global rightMotor
    global wheelRadius
    startTime = time()
    startAngleLeft = getMotorAngle(leftMotor)
    startAngleRight = getMotorAngle(rightMotor)
    distanceMovedLeft = 0
    distanceMovedRight = 0

    fudgeFactor = 1.0
    targetDistance *= fudgeFactor

    initialSpeed = 150 * (1 if forwards else -1)
    oldD = 0

    # Main control loop
    if forwards:
        while(abs(distanceMovedLeft) < abs(targetDistance) or abs(distanceMovedRight) < abs(targetDistance)):
#            accelerateToSpeed(desiredSpeed)
            setLeftMotor(initialSpeed)
            setRightMotor(initialSpeed)
            distanceMovedLeft = wheelRadius*(getMotorAngle(leftMotor) - startAngleLeft)
            distanceMovedRight = wheelRadius*(getMotorAngle(rightMotor) - startAngleRight)

            distanceMoved = (distanceMovedLeft + distanceMovedRight) / 2
            dx = distanceMoved - oldD
            oldD = distanceMoved
            pose.moveForward(dx)

#            print dx
            printParticles(pose)
    stop()
#    print "Distance Moved =", distanceMovedLeft, distanceMovedRight

def accelerateToSpeed(desiredSpeed):
    upperLimit = 80
    lowerLimit = 5
    if abs(desiredSpeed) < lowerLimit or abs(desiredSpeed) > upperLimit:
        return

    global leftMotor
    global rightMotor
    leftError = getVelocity(leftMotor) - desiredSpeed
    rightError = getVelocity(rightMotor) - desiredSpeed

    closeEnough =max(abs(desiredSpeed/6), 1.0)
    
    while(abs(leftError)>closeEnough or abs(rightError)>closeEnough):
        # Calculate error signal
        leftError = getVelocity(leftMotor) - desiredSpeed
        rightError = getVelocity(rightMotor) - desiredSpeed
#        print "Error =", leftError, rightError, "\r",
        sys.stdout.flush()

        # Apply feedback
        proportionalFactor=1
        leftMotorPower = int(proportionalFactor * leftError)
        rightMotorPower = int(proportionalFactor * rightError)
        BrickPi.MotorSpeed[leftMotor] -= leftMotorPower
        BrickPi.MotorSpeed[rightMotor] -= rightMotorPower
        BrickPiUpdateValues()    

        sleep(0.1)

def rotate(angle, clockwise=True): # angle to rotate in degrees
    pose.rotate(angle if clockwise else -angle)

    # Initialise target values
    global leftMotor
    global rightMotor
    global wheelRadius
    global wheelSeparation
    wheelDiameter = 2 * wheelRadius
    targetWheelAngleChange = radians(angle) * wheelSeparation/wheelDiameter

    fudgeFactor = 1.0
    targetWheelAngleChange *= fudgeFactor

    if(clockwise):
        leftTargetAngle = getMotorAngle(leftMotor) + targetWheelAngleChange
        rightTargetAngle = getMotorAngle(rightMotor) - targetWheelAngleChange
    else:
        leftTargetAngle = getMotorAngle(leftMotor) - targetWheelAngleChange
        rightTargetAngle = getMotorAngle(rightMotor) + targetWheelAngleChange
        
    # Initialise Control Loop
    leftMotorPower = 100 * (1 if clockwise else -1)
    rightMotorPower = 100 * (-1 if clockwise else 1)

    arrived = False
#    print "Distance to travel=", wheelSeparation * 0.5 * radians(angle)
    
    leftStartAngle = getMotorAngle(leftMotor)
    rightStartAngle = getMotorAngle(rightMotor)
    # main control loop
    while(not arrived):
        BrickPi.MotorSpeed[leftMotor] = leftMotorPower
        BrickPi.MotorSpeed[rightMotor] = rightMotorPower
        BrickPiUpdateValues()
        
#        leftWheelVelocity = getVelocity(leftMotor)
#        rightWheelVelocity = getVelocity(rightMotor)

        leftAngleError = getMotorAngle(leftMotor) - leftTargetAngle
        rightAngleError = getMotorAngle(rightMotor) - rightTargetAngle

        if(clockwise):
            arrived = leftAngleError > rightAngleError
        else:
            arrived = leftAngleError < rightAngleError
        
#        print "Distance Remaining=", leftAngleError*wheelRadius, rightAngleError*wheelRadius

        distanceMovedLeft = (getMotorAngle(leftMotor) - leftStartAngle) * wheelRadius
        distanceMovedRight = (getMotorAngle(rightMotor) - rightStartAngle) * wheelRadius

        angleTurned = (distanceMovedLeft - distanceMovedRight) / wheelSeparation
#        print "Left: {}, Right: {}\t Rotated {} degrees".format(distanceMovedLeft, distanceMovedRight, degrees(angleTurned))

    stop()
    
    distanceMovedLeft = (getMotorAngle(leftMotor) - leftStartAngle) * wheelRadius
    distanceMovedRight = (getMotorAngle(rightMotor) - rightStartAngle) * wheelRadius

    angleTurned = (distanceMovedLeft - distanceMovedRight) / wheelSeparation
#    print "Left: {}, Right: {}\t Rotated {} degrees".format(distanceMovedLeft, distanceMovedRight, degrees(angleTurned))

def navigateToWaypoint(x, y):
    (curX, curY, curA) = pose.estimatePosition()
    dx = x - curX
    dy = y - curY
    bearing = atan2(dy, dx)
    distance = hypot(dx, dy)
    da = (degrees(bearing) - curA)%360

    if 0 < da < 180:
        print "Rotating by {}".format(da)
        rotate(da)
    else:
        print "Rotating by -{}".format(360-da)
        rotate(360-da, clockwise=False)
        
    print "Travelling {}cm".format(distance)
    goDistance(distance)

        
def stop():
    global leftMotor
    global rightMotor
    BrickPi.MotorSpeed[leftMotor] = 0
    BrickPi.MotorSpeed[rightMotor] = 0
    BrickPiUpdateValues()

def getMotorAngle(motor): # returns motor angle in radians
    BrickPiUpdateValues()
    return radians(BrickPi.Encoder[motor]/2.0)

def getMotorAngularVelocity(motor): # returns angular velocity in radians/second
    sampleTime = 0.05
    theta1 = getMotorAngle(motor)
    sleep(sampleTime)
    theta2 = getMotorAngle(motor)
    dtheta = (theta2 - theta1)

    return dtheta/sampleTime

def getVelocity(motor):
    global wheelRadius
    angularVelocity = getMotorAngularVelocity(motor)
    return angularVelocity * wheelRadius
    
def speedTest(speed):
    global leftMotor
    global rightMotor
    BrickPi.MotorSpeed[leftMotor] = speed;
    BrickPi.MotorSpeed[rightMotor] = speed;
    while(True):
        rps = getMotorAngularVelocity(leftMotor)/(2*pi)
        vel = getVelocity(leftMotor)
        print "rps = {},\tvel = {}".format(rps, vel) 
    
def crashTest():
    global leftBumper
    global rightBumper
    while(True):
        l = leftCrash()
        r = rightCrash()

        if l and r:
            print "Both"
        if l and not r:
            print "Left"
        if not l and r:
            print "Right"
        if not l and not r:
            print "Neither"
            
def navigateWaypoints(waypointList):
    lastEncoders = readEncoders()
    waypoints = waypointList
    while(True):
        # figure out where to go
        (tarX, tarY) = waypoints[0]
        (curX, curY, curA) = pose.estimatePosition()

        dx = tarX - curX
        dy = tarY - curY
        bearing = atan2(dy, dx)

        distance = hypot(dx, dy)
        da = (degrees(bearing) - curA)%360
        
        # work out action to be taken
        minAngle = 2#degrees
        minDistance = 2#cm
        mustRotate = minAngle < da < 360-minAngle 
        mustAdvance = distance > minDistance
#        print "Must rotate {} and advance {}.".format(da, distance)
#        print "Will " + ("rotate" if mustRotate else "advance")
        # work out change from last step
        currEncoders = readEncoders()
        encoderChange = ( currEncoders[0] - lastEncoders[0],
                          currEncoders[1] - lastEncoders[1] )
        encoderDistance = map(encoderToDistance, encoderChange)
        lastEncoders = currEncoders
#        print "\tCurrent Encoders = {}\n\tLast Encoders = {}\n\tEncoder distance change = {}".format(currEncoders, lastEncoders, encoderDistance)

        # update motors and particle model
        if mustRotate:
            if 2 < da < 180:
                print "Rotating Clockwise..."
                navigateClockwise()
                
            if 180 < da < 358:
                print "Rotating Anticlockwise..."
                navigateAnticlockwise()
        
            (l, r) = encoderDistance
            angleRotated = degrees((l - r) / wheelSeparation)
#            print "Rotated {} degrees".format(angleRotated)
            pose.rotate(angleRotated)

        if not mustRotate and mustAdvance:
            print "Advancing..."                
            navigateForwards()
            distanceTravelled = sum(encoderDistance)/len(encoderDistance)
            pose.moveForward(distanceTravelled)

        if not mustRotate and not mustAdvance: # arrived
            print "Waypoint {} reached!".format(waypoints[0])
            waypoints = waypoints[1:] # remove first waypoint

        # update particle weights
        reading = readSonar()
        pose.reweight(reading)

        # resample particle set
        pose.normalise()
        pose.resample()
#        printParticles(pose)
            
def setMotors(left, right):
    setLeftMotor(left)
    setRightMotor(right)
                 
def navigateClockwise():
    setMotors(100, -100)

def navigateAnticlockwise():
    setMotors(-100, 100)

def navigateForwards():
    setMotors(200, 200)

def readEncoders():
    global leftMotor
    global rightMotor
    BrickPiUpdateValues()
    return ( BrickPi.Encoder[leftMotor],
             BrickPi.Encoder[rightMotor] )

def encoderToDistance(encode):
    return wheelRadius * radians(encode/2.0)        

def calculateLikelihood(x, y, a, z):
    sd = 3.0
    k = 0.05
    tmp = findWall(x, y, a)
    m = tmp[1]
    wall = tmp[0]
    Ax = wall[0]
    Ay = wall[1]
    Bx = wall[2]
    By = wall[3]

#    print "m = {}".format(m)

    beta = acos((cos(radians(a))*(Ay-By)+sin(radians(a))*(Bx-Ax))/(sqrt(pow((Ay - By),2) + pow((Bx - Ax),2))))
   
   
    threshold = 360
    if abs(beta) < radians(threshold):
#        print m, sd, z
    	likelihood = getGaussian(m, sd, z) + k
    else:
        likelihood = 0.0

#    print "likelihood = {}".format(likelihood)
    return likelihood

def findWall(x, y , a):

    tmpList = []
    for w in mymap.walls:
        (Ax, Ay, Bx, By) = w
    
        denom = ((By - Ay)*cos(radians(a)) - (Bx - Ax)*sin(radians(a)))
        if denom != 0:
            m = ((By - Ay)*(Ax - x)-(Bx - Ax)*(Ay - y))/denom
        else:
            m = float("inf")
        
        hitsWallAt = (x + m*cos(radians(a)), y + m*sin(radians(a)))
#        print "Robot at {} hits wall {} at {}".format((x, y, a), w, hitsWallAt)

        if min(Ax, Bx)-0.005 <= hitsWallAt[0] <= max(Ax, Bx)+0.005 and min(Ay, By)-0.005 < hitsWallAt[1] < max(Ay, By)+0.005 and m > 0:

            tmpList.append((w, m))
#            print "Hits in front within endpoints of wall. Adding {} to list: {}".format(w, tmpList)

    if tmpList == []:
        e = Exception("No walls found in front of estimated position: Has the robot left the maze?")
        raise e
    else:
        return (min(tmpList, key = lambda x: x[1]))

def getGaussian(m, sd, z):
	return math.e**((-((z-m)*(z-m)))/2*sd*sd)


