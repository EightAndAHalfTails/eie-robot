#44:33:4c:6c:c1:92

from BrickPi import *
from time import sleep, time
from math import pi, floor

leftMotor   = PORT_B
rightMotor  = PORT_A
leftBumper  = PORT_4
rightBumper = PORT_3


wheelRadius = 2.9#cm

def initialiseDiffDriveRobot():
    global leftMotor
    global rightMotor
    BrickPiSetup()
    BrickPi.MotorEnable[leftMotor] = 1 # set up Motors
    BrickPi.MotorEnable[rightMotor] = 1
    BrickPi.SensorType[leftBumper] = TYPE_SENSOR_TOUCH # set up touch sensors
    BrickPi.SensorType[rightBumper] = TYPE_SENSOR_TOUCH
    BrickPiSetupSensors()
    BrickPi.Timeout = 10000 # stop motors after 10 seconds
    BrickPiSetTimeout()

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
        BrickPi.MotorSpeed[leftMotor] = 100
        BrickPi.MotorSpeed[rightMotor] = 100
        BrickPiUpdateValues()
        if leftCrash():
            # move out the way
            pass
        if rightCrash():
            # move out the way
            pass
        

def goForwardsForDistance(targetDistance):

    # Initialisation
    global leftMotor
    global rightMotor
    global wheelRadius
    startTime = time()
    startAngle = getMotorAngle(leftMotor)
    leftMotorPower = 0
    rightMotorPower = 0
    distanceMoved = 0
    desiredSpeed = -40 # Simply accelerate to limiting speed for now, add more speed control later

    # Main control loop
    while(distanceMoved < targetDistance):
        # Send values to BrickPi
        BrickPi.MotorSpeed[leftMotor] = leftMotorPower
        BrickPi.MotorSpeed[rightMotor] = rightMotorPower
        BrickPiUpdateValues()

        # Calculate error signal
        leftVelocity = getVelocity(leftMotor)
        rightVelocity = getVelocity(rightMotor)
        leftError = leftVelocity - desiredSpeed
        rightError = rightVelocity - desiredSpeed

        # Apply feedback
        proportionalFactor=1
        leftMotorPower -= int(proportionalFactor * leftError)
        rightMotorPower -= int(proportionalFactor * rightError)
        distanceMoved = wheelRadius*(getMotorAngle(leftMotor) - startAngle)*pi/360
    stop()
    print "Distance Moved =", distanceMoved

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
        leftDesiredSpeed = -20
        rightDesiredSpeed = 20
    else:
        leftDesiredSpeed = 20
        rightDesiredSpeed = -20

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
            
        
