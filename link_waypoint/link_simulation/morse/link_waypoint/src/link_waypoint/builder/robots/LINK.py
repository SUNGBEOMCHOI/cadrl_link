from morse.builder import *

class LINK(WheeledRobot):
    def __init__(self, name = None, debug = True):

        WheeledRobot.__init__(self, 'link_waypoint/robots/LINK.blend', name)
        self.properties(classpath = "link_waypoint.robots.LINK.LINK",
                        HasSuspension = False, 
                        Influence = 0.1, Friction = 0.8, FixTurningSpeed = 1.16,
                        WheelFLName = "wheel1", WheelFRName = "wheel2",
                        WheelRLName = "wheel3", WheelRRName = "wheel4")
