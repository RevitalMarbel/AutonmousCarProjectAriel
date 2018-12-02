import serial
import math
import time
from hokuyo.driver import hokuyo
from hokuyo.tools import serial_port


class Obsticle(object):
    def __init__(self, angle, distance):
        self.angle = angle;
        self.distance = distance

    def getAngle(self):
        return self.angle

    def getDistanceMeters(self):
        return self.distance/1000.0

    def __str__(self):
        return 'Obs : (' + str(self.angle) + " , " + str(self.distance) + ")"
    def __repr__(self):
        return 'Obs : (' + str(self.angle) + " , " + str(self.distance) + ")"


class Lidar(object):

    def __init__(self,max_distance=3000, maxmin_angle=90,uart_port = '/dev/ttyACM0'):
        #uart_port = '/dev/ttyACM1'
        uart_speed = 19200
        laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
        port = serial_port.SerialPort(laser_serial)
        self.laser = hokuyo.Hokuyo(port)
        self.laser.laser_on()

        # Params
        self.maxMinAngle = maxmin_angle
        self.maxDistance = max_distance
		
		
    def printScan(self):
        return self.laser.get_single_scan()
		
    def getObsticaleScan(self):
        output = []
        try:
            scan = self.laser.get_single_scan()

            scan_list = sorted(scan.iterkeys())
            list_d = []
            # print(len(scan))
            for deg in scan_list:
                if deg < -self.maxMinAngle or deg > self.maxMinAngle:
                    list_d.append(0)
                    continue
                rad = math.radians(deg - 90)
                d = scan[deg]
                if d > self.maxDistance:
                    d = 0
                    # continue
                list_d.append(d)

                # d=d/10.0
            # derivative list
            derivative = self.getDerivative(list_d)
            # obsticles indexes
            indexes = self.find_obsticle(derivative)
            for i in indexes:
                derivative[i] = 5000
            obsticles = [scan_list[i] for i in indexes]
            #Preaper output
            #print obsticles
            for angle in obsticles:
                distanceMM = scan[angle]
                obs = Obsticle(angle,distanceMM)
                output.append(obs)
                #print scan[key]
        except serial.serialutil.SerialException:
            pass
	except AttributeError:
	    pass
        return output


    def getDerivative(self, list):
        list_derivative = [0, 0]
        for i in range(2, len(list)):
            d = (list[i] - list[i - 2]) / 2
            list_derivative.append(d)
        return list_derivative

    def find_obsticle(self, deravitive_list):
        max_abs_thr = 1200
        min_abs_thr = 50
        lr = []
        index_s = 0;
        index_e = 0
        indexes = []
        for i in range(0, len(deravitive_list)):
            val = deravitive_list[i]
            if len(lr) == 0:
                if val > min_abs_thr and val < max_abs_thr:
                    lr.append(val)
                    index_s = i
            else:
                if val < -min_abs_thr and val > -max_abs_thr:
                    lr.append(val)
                    index_e = i

            if len(lr) == 2:
                indexes.append((index_s + index_e) / 2)
                lr = []
        return indexes

    def closeLidar(self):
        self.laser.laser_off()



def main():

    lidar = Lidar(2000)
    while True:
        #print lidar.getObsticaleScan()
        for obs in lidar.getObsticaleScan():
            print obs
            print obs.getAngle()
        print "----------------"
        #time.sleep(0.1)

    lidar.closeLidar()




#main()







