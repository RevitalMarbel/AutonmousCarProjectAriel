from car_dronekit import *
from gridAlgorithm import *
from lidar import *
import threading
import Queue
import sys
import time


class InputThread(threading.Thread):
    def __init__(self):
        super(InputThread, self).__init__()
        self.isStop = False
        self.daemon = True

    def run(self):
        while not self.isStop:
            c = raw_input("For Exit Press : 'q'+Enter !\n")
            if c == 'q':
                self.isStop = True


def readList(fileName):
    file = open(fileName, 'r')
    lines = file.readlines()
    file.close()
    dests = []
    q = Queue.Queue()
    for line in lines:
        stringLatLon = line.split(',')
        lat = stringLatLon[0]
        lon = stringLatLon[1]
        ll = LatLon(lat, lon)
        dests.append(ll)
        q.put(ll)

    print dests
    return q

def simpleMission():

    arg=sys.argv
    repeat = int(1)
    carspeed = 0.75
    try:
        repeat = int(arg[1])
    except IndexError:
        pass

    for i in range(0,repeat):
        try:
            # Get destinations
            destQueue = readList('dest_list.csv')
            car = Car(gpsTimeout=10)
            car.changeToGuide()
            car.arm()
            while not destQueue.empty():
                dest = destQueue.get()
                delay = geo.getDistanceInMeters(dest,car.getPosition())/float(carspeed)
                print "Delay : "+str(delay)
                car.gotoLocation(dest,delay)
                car.changeToGuide()
        except KeyboardInterrupt:
            car.stopCar()

def mission():
    # Get destinations
    destQueue = readList('destlist.csv')
    # Init
    car = Car(gpsTimeout=3)
    car_pos = car.getPosition()
    alg = CarAstarAlgorithm(car_pos, gridSize=10, density=2)
    lidar = Lidar(2000,45)

    # Input thread to stop the loop
    stopInput = InputThread()
    stopInput.start()

    carspeed = 0.75

    # Main loop
    date = datetime.datetime.now()
    file = open('raw_data.txt'+str(date), 'w')
    #file = open('raw_data.txt', 'w')
    pathList = []
        
    while not destQueue.empty() and not stopInput.isStop:
        end = destQueue.get()
        print('END = '+str(end))
        pathQueue = alg.runAlgo(car_pos, car.getCompass(), end)
	pathList = list(destQueue.queue)
        # while not pathQueue.empty():
        #     print pathQueue.get()
        while not pathQueue.empty() and not stopInput.isStop:
            if car.isStoped():
                # get point
                point = pathQueue.get()
                # calc delay
		carPos = car.getPosition();
                delay = geo.getDistanceInMeters(point, carPos) / float(carspeed)
                # delay = 10
                #send car
                car.goToAstar(point,delay)
		time.sleep(3)
	    carPos = car.getPosition()
	    millis = int(round(time.time() * 1000))
            file.write(str(millis)+'\n')
       	    file.write(str(carPos)+'\n')
	    rawLidar = lidar.printScan()
	    file.write(str(rawLidar)+'\n')
            #check lidar
            obsList = lidar.getObsticaleScan()
            for obs in obsList:
                bearing = (car.getCompass()+obs.getAngle())%360
                obsLatLon = geo.getLatLon(car.getPosition(),obs.getDistanceMeters(),bearing)
                alg.addObsticle(obsLatLon)
            if alg.isNewObsticle():
                car.stopCar()
                pathQueue = alg.runAlgo(car.getPosition(),car.getCompass(),end)
		pathList = list(destQueue.queue)
		time.sleep(2)
        delay = geo.getDistanceInMeters(end, car.getPosition()) / float(carspeed)
        car.goToAstar(end,delay)

    if stopInput.isStop:
        car.stopCar()
    #alg.runAlgo(car.getPosition(),compass=car.getCompass(), _end=end)
    file.close()
    print "KML"
    alg.toKML(endDestination=end, start=car_pos)
    alg.obsToKML(endDestination=end, start=car_pos)
    alg.pathToKML(pathList)
    #alg.toKML(endDestination=end, start=car_pos)
    print "Lidar close"
    lidar.closeLidar()

def checkLidarAndAlgo(): 
    # lidar mapping test
    car = Car(gpsTimeout=3)
    car_pos = car.getPosition()
    end = LatLon(32.103237, 35.209693)
    alg = CarAstarAlgorithm(car_pos, gridSize=25, density=2)
    lidar = Lidar(max_distance=3000)

    # Input thread to stop the loop
    stopInput = InputThread()
    stopInput.start()

    while True:
        try:
            if stopInput.isStop:
                break
            # time.sleep(0.1)
            robotPos = car.getPosition()
            compass = car.getCompass()
            obsList = lidar.getObsticaleScan()
            print obsList
            for obs in obsList:
                obsLatLon = geo.getLatLon(robotPos, obs.getDistanceMeters(), (obs.getAngle() + compass) % 360)
                print obsLatLon
                alg.addObsticle(obsLatLon)
            if alg.isNewObsticle():
                #car.stopCar()
                alg.runAlgo(car.getPosition(),car.getCompass(),end)
        except KeyboardInterrupt:
            break

    print "KML"
    alg.toKML(endDestination=end, start=car_pos)
    alg.obsToKML(endDestination=end, start=car_pos)
    alg.pathToKML(endDestination=end, start=car_pos)
    print "Lidar close"
    lidar.closeLidar()

def recordRawData():
    car = Car(gpsTimeout=3)
    car_pos = car.getPosition()
    lidar = Lidar(max_distance=3000)
    date = datetime.datetime.now()
    file = open('raw_data.txt'+str(date), 'w')
	
    while True:
	sleep(0.1)
	rawLidar = lidar.printScan()
	carPos = car.getPosition()
	millis = int(round(time.time() * 1000))
	file.write(str(millis)+'\n')
	file.write(str(carPos)+'\n')
	file.write(str(rawLidar)+'\n')
    file.close()

def main():


    return
    destQueue = readList('dest_list.csv')

    while not destQueue.empty():
        end = destQueue.get()

    checkLidarAndAlgo()
    alg.runAlgo(compass=car.getCompass(), _end=end)
    print "KML"
    alg.toKML(endDestination=end, start=car_pos)
    print "Lidar close"
    lidar.closeLidar()

    return
    d1 = LatLon(32.103101, 35.209710)
    d2 = LatLon(32.102917, 35.209621)
    car = Car()
    car.arm()
    car.changeToGuide()
    car.gotoLocation(d1, 10)

    return
    start = LatLon(32.102490, 35.207479)  # my location
    end = LatLon(32.102696, 35.208138)  #
    print start
    # print ll_e
    compass = -45

    a = CarAstarAlgorithm(start, compass, 30)
    path = a.runAlgo(compass, end)
    print path
    a.toKML(start, end)
    a.pathToKML(path)
    a.print_field()


# main()
#mission()
#simpleMission()
#checkLidarAndAlgo()
mission()
