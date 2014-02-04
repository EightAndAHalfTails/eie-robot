#44:33:4c:6c:c1:92

from BrickPi import *
from time import sleep, time
from math import pi, floor
import sys

sonar       = PORT_3
leftMotor   = PORT_C
rightMotor  = PORT_B
leftBumper  = PORT_2
rightBumper = PORT_1


wheelRadius = 2.9#cm

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
    BrickPi.MotorSpeed[leftMotor] = power
    BrickPiUpdateValues()

def setRightMotor(power):
    global rightMotor
    BrickPi.MotorSpeed[rightMotor] = power
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

def keepDistance(dist):
    sonarMountedOnFront = False
    while(True):
        error = dist - readSonar()
        gain = 10
        gain = -gain if sonarMountedOnFront else gain
        speed = gain * error
        print "Distance =", error
#        accelerateToSpeed(speed) # too unresponsive
        setLeftMotor(speed)
        setRightMotor(speed)
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
    
    # Main control loop
    if forwards:
        while(distanceMovedLeft < targetDistance or distanceMovedRight < targetDistance):
            accelerateToSpeed(desiredSpeed)
            distanceMovedLeft = wheelRadius*(getMotorAngle(leftMotor) - startAngleLeft)*pi/360
            distanceMovedRight = wheelRadius*(getMotorAngle(rightMotor) - startAngleRight)*pi/360
    else:
        while(distanceMovedLeft > targetDistance or distanceMovedRight > targetDistance):
            accelerateToSpeed(-desiredSpeed)
            distanceMovedLeft = wheelRadius*(getMotorAngle(leftMotor) - startAngleLeft)*pi/360
            distanceMovedRight = wheelRadius*(getMotorAngle(rightMotor) - startAngleRight)*pi/360
    stop()
    print "Distance Moved =", distanceMovedLeft, distanceMovedRight

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
    
#    while(abs(leftError)>abs(desiredSpeed/6.0) and abs(rightError)>abs(desiredSpeed/6.0)):
    while(abs(leftError)>closeEnough or abs(rightError)>closeEnough):
        # Calculate error signal
        leftError = getVelocity(leftMotor) - desiredSpeed
        rightError = getVelocity(rightMotor) - desiredSpeed
        print "Error =", leftError, rightError, "\r",
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
    # Initialise target values
    global leftMotor
    global rightMotor
    global wheelRadius
#    wheelRadius = 2.9
    wheelDiameter = 2 * wheelRadius
#    wheelCircumference = 2*pi*wheelRadius
    wheelSeparation = 13.7
#    targetDistance = angle*wheelSeparation/2
#    targetWheelRotations = targetDistance/wheelCircumference
#    targetWheelAngleChange = targetWheelRotations * 720
    targetWheelAngleChange = 2 * angle * wheelSeparation/wheelDiameter

    if(clockwise):
        leftTargetAngle = getMotorAngle(leftMotor) + int(targetWheelAngleChange)
        rightTargetAngle = getMotorAngle(rightMotor) - int(targetWheelAngleChange)
    else:
        leftTargetAngle = getMotorAngle(leftMotor) - int(targetWheelAngleChange)
        rightTargetAngle = getMotorAngle(rightMotor) + int(targetWheelAngleChange)
        
    # Initialise Control Loop
    leftMotorPower = 0
    rightMotorPower = 0
    if(clockwise):
        leftDesiredSpeed = 20
        rightDesiredSpeed = -20
    else:
        leftDesiredSpeed = -20
        rightDesiredSpeed = 20

    arrived = False
    print "Distance to travel=", wheelSeparation * pi* angle/360
    
    # main control loop
    while(not arrived):
        BrickPi.MotorSpeed[leftMotor] = leftMotorPower
        BrickPi.MotorSpeed[rightMotor] = rightMotorPower
        BrickPiUpdateValues()
        
        leftWheelVelocity = getVelocity(leftMotor)
        rightWheelVelocity = getVelocity(rightMotor)
#        print "Wheel Velocity=", leftWheelVelocity, rightWheelVelocity

        leftError = leftWheelVelocity - leftDesiredSpeed
        rightError = rightWheelVelocity - rightDesiredSpeed
#        print "Velocity Error=", leftError, rightError

        proportionalFactor=1
        leftMotorPower -= int(proportionalFactor * leftError)
        rightMotorPower -= int(proportionalFactor * rightError)
#        print "Motor Power=", leftMotorPower, rightMotorPower

        leftAngleError = getMotorAngle(leftMotor) - leftTargetAngle
        rightAngleError = getMotorAngle(rightMotor) - rightTargetAngle
#        print leftAngleError, rightAngleError
        if(clockwise):
            arrived = leftAngleError > 0 or rightAngleError < 0
        else:
            arrived = leftAngleError < 0 or rightAngleError > 0
        print "Distance Remaining=", motorAngleToRadians(leftAngleError)*wheelRadius, motorAngleToRadians(rightAngleError)*wheelRadius
    stop()
        
def motorAngleToRadians(motorAngle):
    return pi*motorAngle/360

def stop():
    global leftMotor
    global rightMotor
    BrickPi.MotorSpeed[leftMotor] = 0
    BrickPi.MotorSpeed[rightMotor] = 0
    BrickPiUpdateValues()

def getMotorAngle(motor): # returns motor angle in semidegrees
    BrickPiUpdateValues()
    return BrickPi.Encoder[motor]

def getMotorAngularVelocity(motor): # returns angular velocity in radians/second
    sampleTime = 0.01
    theta1 = getMotorAngle(motor)
    sleep(sampleTime)
    theta2 = getMotorAngle(motor)
    dtheta = (theta2 - theta1)
    return (dtheta/sampleTime)*pi/360

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
        print "rps = ", getMotorAngularVelocity(leftMotor)/(2*pi)
        print "velocity = ", getVelocity(leftMotor)
    
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
            
        
