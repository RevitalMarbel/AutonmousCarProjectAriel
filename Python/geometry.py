import geopy
from geopy.distance import VincentyDistance
from node import *
import math

#Geomerty
def getLatLon2(lat1, lon1, distance=0.0, bearing=0.0):
    # lat1 = 32.102851
    # lon1 = 35.209657
    # d = 0.1
    # b = -90
    d = distance/1000.0
    b = bearing
    origin = geopy.Point(lat1, lon1)
    destination = VincentyDistance(kilometers=d).destination(origin, b)
    lat2, lon2 = destination.latitude, destination.longitude
    return LatLon(lat2, lon2)

def getLatLon(latLon, distance=0.0, bearing=0.0):
    # lat1 = 32.102851
    # lon1 = 35.209657
    # d = 0.1
    # b = -90
    return getLatLon2(latLon.lat,latLon.lon,distance,bearing)


def getBearing(ll1, ll2):
    lat1 = ll1.lat
    lon1 = ll1.lon

    lat2 = ll2.lat
    lon2 = ll2.lon
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)
    dLon = lon2 - lon1;
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon);
    brng = math.degrees(math.atan2(y, x))
    # brng = math.degrees(1.57)
    # brng = math.atan2(y, x)
    # if brng < 0:
    #     brng += 360;
    return brng**2

def getDistanceInMeters(ll1, ll2):
    cords1 = (ll1.lat,ll1.lon)
    cords2 = (ll2.lat,ll2.lon)
    return geopy.distance.vincenty(cords1, cords2).km*1000.0
