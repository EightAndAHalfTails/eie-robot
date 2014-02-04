from BrickPi import *
from time import sleep, time
from math import pi, floor

left = PORT_A
right= PORT_B

wheelRadius = 2.9#cm

def initialiseDiffDriveRobot():
    BrickPiSetup()
    BrickPi.MotorEnable[left] = 1
    BrickPi.MotorEnable[right] = 1
    BrickPiSetupSensors()
    BrickPi.Timeout = 10000 # stop motors after 10 seconds
    BrickPiSetTimeout()

def goForwardsForDistance(targetDistance):

    # Initialisation
    global left
    global right
    global wheelRadius
    startTime = time()
    startAngle = getMotorAngle(left)
    leftMotorPower = 0
    rightMotorPower = 0
    distanceMoved = 0
    desiredSpeed = 40 # Simply accelerate to limiting speed for now, add more speed control later

    # Main control loop
    while(distanceMoved < targetDistance):
        # Send values to BrickPi
        BrickPi.MotorSpeed[left] = leftMotorPower
        BrickPi.MotorSpeed[right] = rightMotorPower
        BrickPiUpdateValues()

        # Calculate error signal
#        vel = (getVelocity(left)+getVelocity(right))/2
        leftVelocity = getVelocity(left)
        rightVelocity = getVelocity(right)
#        error = vel - desiredSpeed
        leftError = leftVelocity - desiredSpeed
        rightError = rightVelocity - desiredSpeed

        # Apply feedback
        proportionalFactor=1
        leftMotorPower -= int(proportionalFactor * leftError)
        rightMotorPower -= int(proportionalFactor * rightError)
        distanceMoved = wheelRadius*(getMotorAngle(left) - startAngle)*pi/360
    stop()
    print "Distance Moved =", distanceMoved

def rotate(angle, clockwise=True): # angle to rotate in degrees
    # Initialise target values
    global left
    global right
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
        leftTargetAngle = getMotorAngle(left) + int(targetWheelAngleChange)
        rightTargetAngle = getMotorAngle(right) - int(targetWheelAngleChange)
    else:
        leftTargetAngle = getMotorAngle(left) - int(targetWheelAngleChange)
        rightTargetAngle = getMotorAngle(right) + int(targetWheelAngleChange)
        
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
        BrickPi.MotorSpeed[left] = leftMotorPower
        BrickPi.MotorSpeed[right] = rightMotorPower
        BrickPiUpdateValues()
        
        leftWheelVelocity = getVelocity(left)
        rightWheelVelocity = getVelocity(right)
#        print "Wheel Velocity=", leftWheelVelocity, rightWheelVelocity

        leftError = leftWheelVelocity - leftDesiredSpeed
        rightError = rightWheelVelocity - rightDesiredSpeed
#        print "Velocity Error=", leftError, rightError

        proportionalFactor=1
        leftMotorPower -= int(proportionalFactor * leftError)
        rightMotorPower -= int(proportionalFactor * rightError)
#        print "Motor Power=", leftMotorPower, rightMotorPower

        leftAngleError = getMotorAngle(left) - leftTargetAngle
        rightAngleError = getMotorAngle(right) - rightTargetAngle
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
    global left
    global right
    BrickPi.MotorSpeed[left] = 0
    BrickPi.MotorSpeed[right] = 0
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
    global left
    global right
    BrickPi.MotorSpeed[left] = speed;
    BrickPi.MotorSpeed[right] = speed;
    while(True):
        print "rps = ", getMotorAngularVelocity(left)/(2*pi)
        print "velocity = ", getVelocity(left)
    
