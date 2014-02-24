from robot import *

initialiseDiffDriveRobot()

waypoints = [ (84 , 30),
              (180, 30),
              (180, 54),
              (126, 54),
              (126, 168),
              (126, 126),
              (30, 54),
              (84, 54),
              (84, 30) ]

#print "drawLine:(100, 100, 150, 150)"
#print "drawLine:(150, 150, 150, 80)"
#print "drawLine:(150, 80, 100, 100)"

#for w in waypoints:
#    navigateToWaypoint(*w)

navigateWaypoints(waypoints)
