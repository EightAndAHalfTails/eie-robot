from robot import *

initialiseDiffDriveRobot()
while(True):
    accelerateToSpeed(50)
    sleep(2)
    accelerateToSpeed(-50)
    sleep(2)
