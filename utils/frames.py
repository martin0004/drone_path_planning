import numpy as np
import re
from typing import Tuple
from utils.mapping import *
import utm


# ----- REFERENCE FRAMES ----- #
#
# global = global geodetic frame = [latitude, longitude, altitude positive UP]
# utm    = local UTM frame       = [easting, northing, zone number, zone letter]
# local  = local ECEF frame      = [north, east, altitude negative UP]
# grid   = local grid frame      = [index north, index east]
#
# Conversions are performed in this order:
#
# GLOBAL <-> UTM <-> LOCAL <-> GRID
#
# In other words, from left to right:
#
# - global_to_utm converts from GLOBAL to UTM directly
# - utm_to_local converts from UTM to LOCAL directly
# - local_to_grid converts from LOCAL to GRID directly
# - all other conversion functions call these functions under the hood
#
# Same thing from right to left.


# ----- SPEFICIC LOCATIONS ----- #

def get_global_home(file_path: str) -> Tuple[float, float, float]:
    """Home location in global geodetic frame."""

    with open(file_path) as f:
        line = f.readline()

    # Find floats in line
    matches = re.findall(r'[+-]?\d+\.\d*', line)

    lat = float(matches[0])
    lon = float(matches[1])
    alt = 0.0

    return (lat, lon, alt)


# ----- GLOBAL <-> UTM ----- #

def global_to_utm(point: Tuple[float, float, float]
                 ) -> Tuple[float, float, str, str]:
    """Convert a point from the global geodetic frame to the local UTM frame."""

    utm_point = utm.from_latlon(point[0], point[1])

    return utm_point


def utm_to_global(point: Tuple[float, float, str, str],
                  global_altitude: float
                  ) -> Tuple[float, float, float]:
    """Convert a point from the local UTM frame to the global geodetic frame."""

    lat, lon = utm.to_latlon(point[0], point[1], point[2], point[3])
    global_point = (lat, lon, global_altitude)

    return global_point


# ----- UTM <-> LOCAL ----- #

def utm_to_local(point: Tuple[float, float, str, str],
                 global_home: Tuple[float, float, float],
                 local_altitude: float
                 ) -> Tuple[float, float, float]:
    """Convert a point from the local UTM frame to the local ECEF frame."""

    utm_home = global_to_utm(global_home)
    local_home = (point[1] - utm_home[1], point[0] - utm_home[0], local_altitude)

    return local_home


def local_to_utm(point: Tuple[float, float, float],
                 global_home: Tuple[float, float, float]
                 ) -> Tuple[float, float, str, str]:
    """Convert a point from the local ECEF frame to the local UTM frame."""

    utm_home = global_to_utm(global_home)
    utm_point = (point[1] + utm_home[0], point[0] + utm_home[1], utm_home[2], utm_home[3])

    return utm_point


# ----- LOCAL <-> GRID ----- #

def local_to_grid(point: Tuple[float, float, float],
                  north_min: float,
                  east_min: float,
                  cell_size: float
                  ) -> Tuple[int, int]:
    """Convert a point from the local ECEF frame to the local grid frame."""

    north_idx = int(np.floor((point[0] - north_min) / cell_size))
    east_idx = int(np.floor((point[1] - east_min) / cell_size))

    return (north_idx, east_idx)


def grid_to_local(point: Tuple[int, int],
                  altitude: float,
                  north_min: float,
                  east_min: float,
                  cell_size: float
                  ) -> Tuple[float, float, float]:
    """Convert a point from the local grid frame to the local ECEF frame."""

    north = point[0]*cell_size + north_min
    east = point[1]*cell_size + east_min

    return (north, east, altitude)


# ----- GLOBAL <-> (UTM) <-> LOCAL ----- #

def global_to_local(point: Tuple[float, float, float],
                    global_home: Tuple[float, float, float]
                   ) -> Tuple[float, float, float]:
    """Convert a point from the global geodetic frame to the local ECEF frame."""

    utm_point = global_to_utm(point)
    local_point = utm_to_local(utm_point, global_home, -point[2])

    return local_point


def local_to_global(point: Tuple[float, float, float],
                    global_home: Tuple[float, float, float]
                   ) -> Tuple[float, float, float]:
    """Convert a point from the local ECEF frame to the global geodetic frame.""" 

    utm_point = local_to_utm(point, global_home)
    global_point = utm_to_global(utm_point, -point[2])

    return global_point


# ----- UTM <-> (LOCAL) <-> GRID ----- #

def utm_to_grid(point: Tuple[float, float, str, str],
                global_home: Tuple[float, float, float],
                north_min: float,
                east_min: float,
                cell_size: float
               ) -> Tuple[int, int]:
    """Convert a point from the local UTM frame to the local grid frame."""

    local_point = utm_to_local(point, global_home, 0.0)
    grid_point = local_to_grid(local_point, north_min, east_min, cell_size)

    return grid_point


def grid_to_utm(point: Tuple[int, int],
                global_home: Tuple[float, float, float],
                north_min: float,
                east_min: float,
                cell_size: float
               ) -> Tuple[int, int, str, str]:
    """Convert a point from the local grid frame to the local UTM frame."""

    local_point = grid_to_local(point, 0.0, north_min, east_min, cell_size)
    utm_point = local_to_utm(local_point, global_home)

    return utm_point


# ----- GLOBAL <-> (UTM) <-> (LOCAL) <-> GLOBAL ----- #

def global_to_grid(point: Tuple[float, float, float],
                   global_home: Tuple[float, float, float],
                   north_min: float,
                   east_min: float,
                   cell_size: float
                  ) -> Tuple[int, int]:
    """Convert a point from the global geodesic frame to the local grid frame."""

    utm_point = global_to_utm(point,)
    local_point = utm_to_local(utm_point, global_home, 0.0)
    grid_point = local_to_grid(local_point, north_min, east_min, cell_size)

    return grid_point


def grid_to_global(point: Tuple[int, int],
                   global_home: Tuple[float, float, float],
                   global_altitude: float,
                   north_min: float,
                   east_min: float,
                   cell_size: float
                  ) -> Tuple[float, float, float]:
    """Convert a point from the local grid frame to the global geodetic frame."""

    local_point = grid_to_local(point, 0.0, north_min, east_min, cell_size)
    utm_point = local_to_utm(local_point, global_home)
    global_point = utm_to_global(utm_point, global_altitude)

    return global_point



