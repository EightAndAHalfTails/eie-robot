from BrickPi import *
from robot import *
initialiseDiffDriveRobot()
odometer_left = getMotorAngle(left)
goForwardsForDistance(40)
odometer_left = getMotorAngle(left) - odometer_left
print "distance travelled = ", 2.9*odometer_left*pi/360, "cm"
