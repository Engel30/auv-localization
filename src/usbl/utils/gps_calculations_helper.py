import logging

import pymap3d
import pynmea2


def parse_gga_string(gga_string: str) -> tuple[float, float, float] | None:
    """
    Parses a GPGGA string and returns a dictionary with key data.
    :param gga_string: The string to be parsed having the correct format
    :return: If parsing is successful, it returns ``(latitude,longitude,altitude)`` otherwise `None`
    """
    try:
        # Example:
        # $GPGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
        #
        # | Field | Value       | Description
        # | 1     | `123519.00` | UTC Time: 12:35:19
        # | 2     | `4807.038`  | Latitude: 48 degrees, 07.038 minutes
        # | 3     | `N`         | North/South: North
        # | 4     | `01131.000` | Longitude: 11 degrees, 31.000 minutes
        # | 5     | `E`         | East/West: East
        # | 6     | `1`         | Fix Quality: 0=Invalid, 1=GPS fix, 2=DGPS fix
        # | 7     | `08`        | Number of Satellites being tracked
        # | 8     | `0.9`       | Horizontal Dilution of Precision (HDOP)
        # | 9     | `545.4`     | Altitude (above mean sea level)
        # | 10    | `M`         | Units of Altitude: Meters
        # | 11    | `46.9`      | Height of geoid above WGS84 ellipsoid
        # | 12    | `M`         | Units of Geoid Height: Meters
        # | 13    |             | (empty) Time since last DGPS update
        # | 14    |             | (empty) DGPS station ID number
        # | *CS   | `*47`       | Checksum
        msg = pynmea2.parse(gga_string)

        # Check if it's a GPGGA message
        if not isinstance(msg, pynmea2.types.talker.GGA):
            logging.error("Not a GPGGA message.")

        # Check for a valid GPS fix.
        # 0=No Fix, 1=GPS, 2=DGPS, etc.
        if msg.gps_qual < 1:
            logging.warning("GPGGA message has no valid fix.")
        else:
            return msg.latitude, msg.longitude, msg.altitude

    except ValueError:
        logging.error(f"Error parsing GPGGA string: string could not be parsed or the checksum did not match.")
    except Exception as e:
        logging.critical(f"Unknown error occurred when parsing GPGGA string: {e}")


def calculate_new_gps_coordinates(
        reference_point: tuple[float, float, float],
        ENU: tuple[float, float, float],
) -> tuple[float, float, float]:
    """
    Calculates new GPS coordinates by applying an ENU offset to a reference point.

    :param ENU: East, North and Up offset in meters
    :param reference_point: reference GPS coordinates
    :return: A tuple: (new_latitude, new_longitude, new_altitude)
    """
    # It performs the transformation based on the WGS84 ellipsoid model
    latitude, longitude, altitude = reference_point
    east, north, up = ENU
    new_latitude, new_longitude, new_altitude = pymap3d.enu2geodetic(east, north, up, latitude, longitude, altitude)

    return new_latitude, new_longitude, new_altitude
