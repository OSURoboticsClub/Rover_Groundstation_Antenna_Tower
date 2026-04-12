# Vector math functions to support the antenna positioning system
# Nolan Kessler - 2026
# kesslnol@oregonstate.edu

import math
import typing

EARTH_RADIUS_M = 6371 * 1000

class Vec3:
    _x: float
    _y: float
    _z: float

    def __init__(self, x: float, y: float, z: float):
        self._x = x
        self._y = y
        self._z = z

    def x(self):
        return self._x

    def y(self):
        return self._y

    def z(self):
        return self._z

    def set(self, x: float, y: float, z: float):
        self._x = x
        self._y = y
        self._z = z

    def mag(self) -> float:
        return math.sqrt(self._x ** 2 + self._y ** 2 + self._z ** 2)
    
    def __add__(self, other: object):
        if isinstance(other, Vec3):
            return Vec3(self._x + other._x, self._y + other._y, self._z + other._z)
        else:
            raise TypeError(f"__add__ not defined for Vec3 and {type(other)}")
        
    def __invert__(self):
        return Vec3(-self._x, -self._y, -self._z)
    
    def __sub__(self, other: object):
        if isinstance(other, Vec3):
            return Vec3(
                self._x - other._x,
                self._y - other._y,
                self._z - other._z
            )
        else:
            raise TypeError(f"__add__ not defined for Vec3 and {type(other)}")
        
    def __mul__(self, other: object):
        if isinstance(other, float) or isinstance(other, int):
            return Vec3(self._x * other, self._y * other, self._z * other)
        else:
            raise TypeError(f"__mul__ not defined for Vec3 and {type(other)}")
        
    def __rmul__(self, other:object):
        return self.__mul__(other)
        
    def __str__(self) -> str:
        return f"[x:{self._x}, y:{self._y}, z:{self._z}]"
    
    def dot(self, other: "Vec3") -> float:
        return  self._x*other._x + self._y*other._y + self._z*other._z
    
    def cross(self, other: "Vec3") -> "Vec3":
        return Vec3 (
              self._y * other._z - self._z * other._y,
            -(self._x * other._z - self._z * other._x),
              self._x * other._y - self._y * other._x
        )
    
    def norm(self) -> "Vec3":
        return self * (1 / self.mag())
    
    #TODO: function to project vector onto plane, get angle between two vectors

class LatLong(Vec3):

    def r(self):
        return self.mag()
    
    def lat(self):
        lat = math.degrees(math.atan2(self._z, math.sqrt(self._x * self._x + self._y * self._y)))
        return lat
    
    def long(self):
        return math.degrees(math.atan2(self._y, self._x))
      
    def setLatLong(self, r:float, lat:float, long:float) -> None:
        self._x = r * math.cos(math.radians(lat)) * math.cos(math.radians(long))
        self._y = r * math.cos(math.radians(lat)) * math.sin(math.radians(long))
        self._z = r * math.sin(math.radians(lat))
    
    def __init__(self, r: float=0, lat: float=0, long: float=0, vec3:typing.Optional[Vec3]=None):
        self.setLatLong(r, lat, long)
        if vec3 != None:
            super().set(vec3.x(), vec3.y(), vec3.z())
    
    def __add__(self, other: object):
        return LatLong(vec3=super().__add__(other))
    
    def __sub__(self, other: object):
        return LatLong(vec3=super().__sub__(other))
    
    def __invert__(self):
        return LatLong(vec3=super().__invert__())
    
    def __mul__(self, other: object):
        return LatLong(vec3=super().__mul__(other))
    
    def __rmul__(self, other:object):
        return self.__mul__(other)
    
    def cross(self, other: Vec3) -> "LatLong":
        return LatLong(vec3=super().cross(other))
    
    def norm(self) -> "LatLong":
        return LatLong(vec3=super().norm())
    
    def __str__(self) -> str:
        return super().__str__() + f" [r:{self.r()}, lat:{self.lat()}, long:{self.long()}]"

def getPanAngleDegrees(v1: LatLong, v2: LatLong):
    #difference vector from v1 to v2
    vd  = v2 - v1
    #a vector pointing in the north direction
    vn  = LatLong(r=v1.r(), lat=v1.lat() + 90, long=v1.long())
    #the projection of the difference vector onto the ground plane
    vdg = vd - (v1 * ((vd.dot(v1)) / (v1.mag() ** 2)))

    #obtain the cross product
    cross = vdg.cross(vn)

    #This assumes angle is clockwise positive (when viewed from above earth's surface), with zero being true north
    return math.degrees(
        math.acos(vdg.dot(vn) / (vdg.r() * vn.r()))
    ) * (-1 if math.isclose((v1.norm() + cross.norm()).mag(), 0, abs_tol=0.1) else 1)

def getElevationAngleDegrees(v1: LatLong, v2: LatLong):
    vd = v2 - v1
    return -math.degrees(math.acos(
        vd.dot(v1) / (vd.mag() * v1.mag())
    )) + 90

def getMagneticNorthOffsetDegrees(v1: LatLong) -> float:
    #location of magnetic north
    mn = LatLong(EARTH_RADIUS_M, 86.494, 162.876)
    #vector pointing toward true north
    vn = LatLong(r=v1.r(), lat=v1.lat() + 90, long=v1.long())
    vd = mn - v1
    #the projection of the difference vector onto the ground plane
    vdg = vd - (v1 * ((vd.dot(v1)) / (v1.mag() ** 2)))
    #obtain the cross product
    cross = vdg.cross(vn)
    return math.degrees(
        math.acos(vdg.dot(vn) / (vdg.r() * vn.r()))
    ) * (-1 if math.isclose((v1.norm() + cross.norm()).mag(), 0, abs_tol=0.1) else 1)



