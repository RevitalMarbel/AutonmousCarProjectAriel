from node import *
import geometry as geo
import numpy as np
import datetime
import Queue

class CarAstarAlgorithm:

    def __init__(self, startingPostition, gridSize = 20.0,density=1):

        self.gridDensity = density
        self.distanceEpsilon = 0.1

        self.isNewObs = False;

        self.deltaDistance = (self.gridDensity-self.distanceEpsilon)/2.0

        self.robotPos = startingPostition

        self.gridSize = gridSize
        self.distanceFromCenter = gridSize
        self.stepSize = (self.gridDensity * self.distanceFromCenter / self.gridSize) * 1

        self.open = []
        self.closed = []
        self.obsticales=[]
        self.field = [[0 for x in range(int(gridSize))] for y in range(int(gridSize))]


        # self.startLatLon = self.get_first_latLon(compass)
        # print self.startLatLon
        # self.init_field(compass)

    def addObsticle(self,obsLatLon):
        for obs in self.obsticales:
            if geo.getDistanceInMeters(obs,obsLatLon) < self.deltaDistance:

                return
        self.isNewObs=True;
        self.obsticales.append(obsLatLon)

    def isNewObsticle(self):
        ans = self.isNewObs
        self.isNewObs = False
        return ans

    def runAlgo(self,carPos, compass, _end):
        self.robotPos=carPos
        # firstLatLon = self.startLatLon
        firstLatLon= self.get_first_latLon(compass)
        for i in range(len(self.field)):
            # firstLatLon = getLatLon(firstLatLon.lat,firstLatLon.lon,0.002,compass-180)
            for j in range(len(self.field[0])):
                # latLon = getLatLon(firstLatLon.lat,firstLatLon.lon,0.002*j,compass+90)
                latLon = geo.getLatLon(firstLatLon, self.stepSize * j, compass + 90)
                n = Node((i, j), latLon.lat, latLon.lon)
                self.field[i][j] = n
                #if j==20 and i < len(self.field)-2:
                #    n.isBlack=True
                #n.h = self.hueristit_distance(_end, (i, j))
                n.h =self.hueristit_distance_ll(_end,n.latLon)
                for obs in self.obsticales:
                    if geo.getDistanceInMeters(obs,n.latLon) < self.deltaDistance:
                        n.isBlack=True
            firstLatLon = geo.getLatLon(firstLatLon, self.stepSize, compass - 180)
        #Loop to get path
        # path=[]
        path = Queue.LifoQueue()
        _startIndex = self.latLonToIndex(self.robotPos);
        self.check_neighboors(_startIndex)
        minNode = self.find_min_f()

        _endIndex=self.latLonToIndex(_end);
        # print _endIndex
        while len(self.open) > 0 and not self.cmpNodes(_endIndex, minNode.index):
            self.check_neighboors(minNode.index)
            minNode = self.find_min_f()

        # path.append(minNode.latLon)
        path.put(minNode.latLon)
        while not self.cmpNodes(minNode.index, _startIndex):
            try:
                minNode = minNode.parent
                # path.append(minNode.latLon)
                path.put(minNode.latLon)
            except AttributeError:
                break
            # print minNode.index
        return path

    def latLonToIndex(self,ll):
        ans = self.field[0][0]
        mindis = 999999
        for i in range(len(self.field)):
            for j in range(len(self.field[0])):
                node = self.field[i][j]
                dis = geo.getDistanceInMeters(node.latLon,ll)
                if dis < mindis:
                    ans = node
                    mindis = dis
        return ans.index

    def hueristit_distance_ll(self,ll1,ll2):
        # print geo.getDistanceInMeters(ll1, ll2)
        return geo.getDistanceInMeters(ll1, ll2)**2

    def get_first_latLon(self,compass):
        latLon_s = geo.getLatLon(self.robotPos, int(self.gridSize), compass - 90)
        latLon_s = geo.getLatLon(latLon_s, int(self.gridSize), compass + 0)
        return latLon_s

    #Algorithm path
    def is_in_list(self,list, node):
        for n in list:
            if n.id == node.id:
                return True
        return False

    def check_neighboors(self,index):
        pi = index[0]
        pj = index[1]
        sourceNode = self.field[pi][pj]
        self.closed.append(sourceNode)
        for r in range(pi - 1, pi + 2):
            for c in range(pj - 1, pj + 2):
                if r == pi and c == pj or r < 0 or c < 0 or r >= len(self.field) or c >= len(self.field[0]):
                    continue
                currentNode = self.field[r][c]
                if (currentNode.isBlack) or (self.is_in_list(self.closed, currentNode)):
                    continue
                if (not self.is_in_list(self.open, currentNode)):
                    currentNode.parent = sourceNode
                    currentNode.g = self.move_cost((pi, pj), (r, c)) + sourceNode.g
                    currentNode.f = currentNode.g + currentNode.h
                    self.open.append(currentNode)
                else:
                    move = self.move_cost((pi, pj), (r, c))
                    if currentNode.g > move + sourceNode.g:
                        currentNode.g = move + sourceNode.g
                        currentNode.f = currentNode.g + currentNode.h
                        currentNode.parent = sourceNode
    # my ang start=0
    # def move_cost(self,_from,_to):
    #     sub = tuple(np.subtract(_from, _to))
    #     if sub[0] == 0 or sub[1] == 0:
    #         return 10
    #     return 14
    def find_min_f(self):
        min = self.open[0].f
        ans = self.open[0]
        for node in self.open:
            if node.f < min:
                min = node.f
                ans = node
        self.open.remove(ans)
        return ans

    def cmpNodes(self,t1, t2):
        return sorted(t1) == sorted(t2)

    def move_cost(self,_from, _to):
        sub = tuple(np.subtract(_from, _to))
        if sub[0] == 0 or sub[1] == 0:
            return 10
        if sub[0] == -1 or sub[1] == -1:
            return 20
        return 14

    #Print
    def toKML(self, endDestination, start=LatLon(0, 0)):
        date = datetime.datetime.now()
	file = open('filed'+str(date)+'.kml', 'w')
        file.write('<?xml version="1.0" encoding="UTF-8"?>\n<kml xmlns="http://www.opengis.net/kml/2.2">')
        # file.write('  <Placemark>\n<LineString>\n<coordinates>')
        file.write('<Folder xmlns:gx="http://www.google.com/kml/ext/2.2" xmlns:atom="http://www.w3.org/2005/Atom" xmlns="http://www.opengis.net/kml/2.2">')
        for i in range(len(self.field)):
            for j in range(len(self.field[0])):
                if self.field[i][j].isBlack:
                    continue;
                file.write('  <Placemark>')
                # file.write('<name>')
                # file.write(''+str(i)+","+str(j))
                # file.write('</name>')
                file.write('<name>')
                file.write('('+str(i)+str(j)+')')
                file.write('</name>')
                file.write('\n<Point>\n<coordinates>')
                file.write(str(self.field[i][j].latLon.lon) + ',' + str(self.field[i][j].latLon.lat) + ',0\n')
                file.write('</coordinates>\n</Point>\n</Placemark>')
        # file.write('</coordinates>\n</LineString>\n</Placemark>')

        file.write('  <Placemark>')
        file.write('<name>')
        file.write('START')
        file.write('</name>')
        file.write('\n<Point>\n<coordinates>')
        file.write(str(start.lon) + ',' + str(start.lat) + ',0\n')
        file.write('</coordinates>\n</Point>\n</Placemark>')

        file.write('  <Placemark>')
        file.write('<name>')
        file.write('END')
        file.write('</name>')
        file.write('\n<Point>\n<coordinates>')
        file.write(str(endDestination.lon) + ',' + str(endDestination.lat) + ',0\n')
        file.write('</coordinates>\n</Point>\n</Placemark>')


        file.write('</Folder>')
        file.write('</kml>')
        file.closed
    def obsToKML(self, endDestination, start=LatLon(0, 0)):
        date = datetime.datetime.now()
        file = open('obs'+str(date)+'.kml', 'w')

        file.write('<?xml version="1.0" encoding="UTF-8"?>\n<kml xmlns="http://www.opengis.net/kml/2.2">')
        # file.write('  <Placemark>\n<LineString>\n<coordinates>')
        file.write('<Folder xmlns:gx="http://www.google.com/kml/ext/2.2" xmlns:atom="http://www.w3.org/2005/Atom" xmlns="http://www.opengis.net/kml/2.2">')
        for i in range(len(self.field)):
            for j in range(len(self.field[0])):
                if self.field[i][j].isBlack:
		   file.write('  <Placemark>')
		   # file.write('<name>')
      		   # file.write(''+str(i)+","+str(j))
		   # file.write('</name>')
		   file.write('<name>')
	           file.write('('+str(i)+str(j)+')')
		   file.write('</name>')
	           file.write('\n<Point>\n<coordinates>')
		   file.write(str(self.field[i][j].latLon.lon) + ',' + str(self.field[i][j].latLon.lat) + ',0\n')
		   file.write('</coordinates>\n</Point>\n</Placemark>')
		   # file.write('</coordinates>\n</LineString>\n</Placemark>')

        file.write('  <Placemark>')
        file.write('<name>')
        file.write('START')
        file.write('</name>')
        file.write('\n<Point>\n<coordinates>')
        file.write(str(start.lon) + ',' + str(start.lat) + ',0\n')
        file.write('</coordinates>\n</Point>\n</Placemark>')

        file.write('  <Placemark>')
        file.write('<name>')
        file.write('END')
        file.write('</name>')
        file.write('\n<Point>\n<coordinates>')
        file.write(str(endDestination.lon) + ',' + str(endDestination.lat) + ',0\n')
        file.write('</coordinates>\n</Point>\n</Placemark>')


        file.write('</Folder>')
        file.write('</kml>')
        file.closed

    def pathToKML(self,path):
	date = datetime.datetime.now()
        file = open('path'+str(date)+'.kml', 'w')

        #file = open('path.kml', 'w')
	file.write('<?xml version="1.0" encoding="UTF-8"?>\n<kml xmlns="http://www.opengis.net/kml/2.2">')
	file.write('<Folder xmlns:gx="http://www.google.com/kml/ext/2.2" xmlns:atom="http://www.w3.org/2005/Atom" xmlns="http://www.opengis.net/kml/2.2">')
	
	for ll in path:
  		file.write('  <Placemark>')
		file.write('<name>')
		file.write('</name>')
		file.write('\n<Point>\n<coordinates>')
		file.write(str(ll.lon) + ',' + str(ll.lat) + ',0\n')
		file.write('</coordinates>\n</Point>\n</Placemark>')
	file.write('</Folder>')
	file.write('</kml>')
	file.closed

    def print_field(self):
        s = ''
        for i in range(len(self.field)):
            s += '{'
            for j in range(len(self.field[0])):
                s += '[' + str(self.field[i][j]) + ']'
            s += '}\n'
        print s
