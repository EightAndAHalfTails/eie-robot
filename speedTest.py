from robot import *

initialiseDiffDriveRobot()
while(True):
    goDistance(50, desiredSpeed=50)
    sleep(2)
    goDistance(-50)
    sleep(2)
