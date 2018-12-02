ID = 1
class Node(object):
    def __init__(self,index,lat=0,lon=0):
        global ID
        self.id=ID
        ID+=1
        self.h=0
        self.g=0
        self.f=0
        self.isBlack=False
        self.index=index
        self.latLon = LatLon(lat,lon)
        self.bearing = 0


    def __str__(self):
        # return str(self.id)+','+str(self.h)+','+str(self.g)+','+str(self.f)+','+str(self.isBlack);
        return str(self.h) + ',' + str(self.g) + ',' + str(self.f) + ',' + str(self.isBlack);
        # return str(self.latLon)

class LatLon(object):
    def __init__(self,lat,lon):
        self.lat=lat
        self.lon=lon

    def __str__(self):
        return '('+str(self.lat) + ',' + str(self.lon)+')'
    def __repr__(self):
        return '('+str(self.lat) + ',' + str(self.lon)+')'