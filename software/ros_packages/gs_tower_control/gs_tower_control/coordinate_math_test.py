from coordinate_math import *

def main():
    v1 = LatLong(70 + EARTH_RADIUS_M, 44.43, -123.16)
    v2 = LatLong(90 + EARTH_RADIUS_M, 44.43, -123.15)


    print(f"Elevation Angle (deg) {getElevationAngleDegrees(v1, v2)}")
    print(f"Pan Angle (Deg) {getPanAngleDegrees(v1, v2)}")


if __name__ == "__main__":
    main()