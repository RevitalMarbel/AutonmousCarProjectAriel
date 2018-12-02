from node import *
import geometry as geo
import datetime
from dronekit import *
import threading



class Car(object):
    def __init__(self, gpsTimeout=10):
        self.connection_string = '/dev/ttyS0'
        #self.connection_string = '127.0.0.1:14550'
        self.gpsTimeOut = gpsTimeout
        self.connectAndObtainGPS()


    def connectAndObtainGPS(self):
        # Connect to the Vehicle.
        print(" - Connecting to vehicle on: %s - " % (self.connection_string))
        self.vehicle = connect(self.connection_string, baud=57600, wait_ready=True)

        # Wait for FIX for 'n' seconds
        print(" - GPS fix Status - : %s " % (self.vehicle.gps_0.fix_type))
        print(" - Waiting GPS - ")
        self.waitForGPS(self.gpsTimeOut)
        # Change mode to GUIDED
        self.changeToGuide()
        self.vehicle.mode = VehicleMode("HOLD")


    def gotoLocation(self, destLatLon, delay):
        destination = LocationGlobalRelative(float(destLatLon.lat), float(destLatLon.lon), int(0))
        print "goto " + str(destination)
        try:
            self.vehicle.simple_goto(destination, groundspeed=int(1))
        except APIException as apie:
            pass
        # init timer
        initTime = datetime.datetime.now()
        lastTime = datetime.datetime.now()
        deltaTime = lastTime - initTime
        currentLocation = LatLon(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon)
        distance = geo.getDistanceInMeters(currentLocation, destLatLon)
        while (distance > 0.5 and deltaTime.seconds < delay):
            print("Distance to targer : %f , time %s" % (distance, deltaTime.seconds))
            time.sleep(1)
            # calc timer
            lastTime = datetime.datetime.now()
            deltaTime = lastTime - initTime
            # establish current location
            currentLocation = LatLon(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon)
            distance = geo.getDistanceInMeters(currentLocation, destLatLon)
        # stop vehicle at current location
        try:
            # vehicle.simple_goto(currentLocation, groundspeed=int(1))
            self.vehicle.mode = VehicleMode("HOLD")
        except APIException as apie:
            print apie

    def goToAstar(self, destLatLon, delay):
        # self.changeToGuide()
        # destination = LocationGlobalRelative(float(destLatLon.lat), float(destLatLon.lon), int(0))
        # try:
        #     self.vehicle.simple_goto(destination, groundspeed=int(1))
        # except APIException as apie:
        #     pass
        self.gotoThread = GoToThread()
        self.gotoThread.init(self.vehicle, destLatLon, delay)
        self.gotoThread.start()

    def stopCar(self):
        try:
            self.gotoThread.stop()
        except AttributeError:
            pass
        time.sleep(0.3)
        self.vehicle.mode = VehicleMode("HOLD")
        time.sleep(0.3)
        self.vehicle.mode = VehicleMode("HOLD")
        time.sleep(0.3)
        self.vehicle.mode = VehicleMode("HOLD")
        time.sleep(1)

    def arm(self):
        print(" - Arming motors - ")
        self.vehicle.armed = True
        while self.vehicle.mode != "GUIDED":
            time.sleep(1)

    def isStoped(self):
        return self.vehicle.mode == "HOLD"

    def getPosition(self):
        return LatLon(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon)

    def getCompass(self):
        return self.vehicle.heading

    def waitForGPS(self, seconds):
        # init timer
        initTime = datetime.datetime.now()
        lastTime = datetime.datetime.now()
        deltaTime = lastTime - initTime
        while deltaTime.seconds < seconds:
            print(" - GPS fix Status - : %s , %s " % (self.vehicle.gps_0.fix_type, deltaTime.seconds))
            if self.vehicle.gps_0.fix_type < 3:
                # init timer
                initTime = datetime.datetime.now()
                lastTime = datetime.datetime.now()
            else:
                lastTime = datetime.datetime.now()
            deltaTime = lastTime - initTime
            time.sleep(1)

    def changeToGuide(self):
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        while self.vehicle.mode != "GUIDED":
            time.sleep(1)
        print (" - GUIDED - ")


class GoToThread(threading.Thread):
    def __init__(self):
        super(GoToThread, self).__init__()

    def init(self,vehicle, latlon, delay):
        self.vehicle = vehicle
        self.latlon = latlon
        self.gotodelay = delay
        self.keepGoing = True
        self.daemon = True

    def stop(self):
        self.keepGoing = False

    def changeToGuide(self):
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        while self.vehicle.mode != "GUIDED":
            time.sleep(1)
        print (" - GUIDED - ")

    def arm(self):
        print(" - Arming motors - ")
        self.vehicle.armed = True
        while self.vehicle.mode != "GUIDED":
            time.sleep(1)

    def run(self):
        self.changeToGuide()
        self.arm()
        destination = LocationGlobalRelative(float(self.latlon.lat), float(self.latlon.lon), int(0))
        print "GotoThread dest " + str(destination)
        # try:
        self.vehicle.simple_goto(destination, groundspeed=int(1))
        # except APIException as apie:
        #     pass
        # init timer
        initTime = datetime.datetime.now()
        lastTime = datetime.datetime.now()
        deltaTime = lastTime - initTime
        currentLocation = LatLon(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon)
        distance = geo.getDistanceInMeters(currentLocation, self.latlon)
        #while (distance > 0.5 and deltaTime.seconds < self.gotodelay and self.keepGoing):
		while (distance > 0.5 and self.keepGoing):
            print("Distance to targer : %f , time %s" % (distance, deltaTime.seconds))
            time.sleep(1)
            # calc timer
            lastTime = datetime.datetime.now()
            deltaTime = lastTime - initTime
            # establish current location
            currentLocation = LatLon(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon)
            distance = geo.getDistanceInMeters(currentLocation, self.latlon)
        # stop vehicle at current location
        try:
            # vehicle.simple_goto(currentLocation, groundspeed=int(1))
            print "GoToThread Stoped"
            self.vehicle.mode = VehicleMode("HOLD")
        except APIException as apie:
            print apie


def test():
    c = Car()
    while True:
        print c.getCompass()

        # test()
