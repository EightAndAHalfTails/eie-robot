from robot import *

initialiseDiffDriveRobot()

waypoints = [ (50, 50),
              (50, -20),
              (0, 0) ]

print "drawLine:(100, 100, 150, 150)"
print "drawLine:(150, 150, 150, 80)"
print "drawLine:(150, 80, 100, 100)"

#for w in waypoints:
#    navigateToWaypoint(*w)

navigateWaypoints(waypoints)
