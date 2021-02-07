import socket
import time
import hmath
import math
import numpy

socket_logger = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    
socket_logger.connect(('127.0.0.1', 5002))
socket_logger.settimeout(2)

# start socket 
Range_Socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
Range_Socket.bind(('127.0.0.1', 5003))
Range_Socket.settimeout(20)


bufferRead = 1024

balloon1_ID = 1
balloon2_ID = 2
balloon3_ID = 253
balloon4_ID = 254
balloon5_ID = 255


class stMercator:

    def __init__(self, System: int, Latitude: float, Longitude: float, Altitude: float, Epoch: int, x: float, y: float, z: float):

        self.Latitude = Latitude
        self.Longitude = Longitude
        self.Altitude = Altitude
        self.Epoch = Epoch
        self.System = System
        self.x = x 
        self.y = y 
        self.z = z 


def stringToMercator(raw_data: str):

    try:

        raw_data.index("'system':")
        raw_data.index("'altitude':")
        raw_data.index("'latitude':")
        raw_data.index("'longitude':")
        raw_data.index("'time':")

    except ValueError:
        
        return False, stMercator(0,0,0,0,0,0,0,0)

    t_Mercator = stMercator(0,0,0,0,0,0,0,0)

    try:

        system_begin_idx = raw_data.find("'system': ")
        system_end_idx = raw_data.find(';', system_begin_idx)
        t_Mercator.System = int(raw_data[system_begin_idx + 10:system_end_idx])

        altitude_begin_idx = raw_data.find("'altitude': ")
        altitude_end_idx = raw_data.find(';', altitude_begin_idx)
        t_Mercator.Altitude = float(raw_data[altitude_begin_idx + 12:altitude_end_idx])    

        latitude_begin_idx = raw_data.find("'latitude': ")
        latitude_end_idx = raw_data.find(';', latitude_begin_idx)
        t_Mercator.Latitude = float(raw_data[latitude_begin_idx + 12:latitude_end_idx])

        longitude_begin_idx = raw_data.find("'longitude': ")
        longitude_end_idx = raw_data.find(';', longitude_begin_idx)
        t_Mercator.Longitude = float(raw_data[longitude_begin_idx + 13:longitude_end_idx])

        time_begin_idx = raw_data.find("'time': ")
        time_end_idx = raw_data.find('}', time_begin_idx)
        t_Mercator.Epoch = float(raw_data[time_begin_idx + 8:time_end_idx])

        return True, t_Mercator

    except ValueError:

        return False, stMercator(0,0,0,0,0,0,0,0)

last_known_location_set = False;
last_known_location = stMercator(0,0,0,0,0,0,0,0)

while True:

    # read the socket 
    try:
        data_bytes = socket_logger.recv(bufferRead)
    except:
        print("Connection error.")
        break
    
    data_str = data_bytes.decode('utf-8')

    string_list = []

    iterator = 0
    character = ''

    while data_str.find('}', iterator) != -1:

        substring_end = data_str.find('}', iterator)

        string_list.append(data_str[iterator:substring_end + 1])

        iterator = substring_end + 1;

    if len(string_list) > 0:

        mercators = []

        for string in string_list:

            received, t_Mercator = stringToMercator(string)

            if received:

                mercators.append(t_Mercator)

        if last_known_location_set == False:

            for mercator in mercators:

                if mercator.System == 0:

                    last_known_location = mercator
                    last_known_location_set = True

        if last_known_location_set == True:

            ranges = ""

            idx = 0

            last_known_location_vector = hmath.Vector3(0,0,0)

            lkl_lat_rad = last_known_location.Latitude * (math.pi / 180)
            lkl_lon_rad = last_known_location.Longitude * (math.pi / 180)

            last_known_location_vector.a = earth_radius * (math.cos(lkl_lat_rad)) * (math.cos(lkl_lon_rad))
            last_known_location_vector.b = earth_radius * (math.cos(lkl_lat_rad)) * (math.sin(lkl_lon_rad))
            last_known_location_vector.c = earth_radius * (math.sin(lkl_lat_rad))

            while idx < len(mercators):

                if mercators[idx].System != 0:

                    mercator_vector = hmath.Vector3(0,0,0)

                    mv_lat_rad = mercators[idx].Latitude * (math.pi / 180)
                    mv_lon_rad = mercators[idx].Longitude * (math.pi / 180)

                    mercator_vector.a = earth_radius * (math.cos(mv_lat_rad)) * (math.cos(mv_lon_rad))
                    mercator_vector.b = earth_radius * (math.cos(mv_lat_rad)) * (math.sin(mv_lon_rad))
                    mercator_vector.c = earth_radius * (math.sin(mv_lat_rad))

                    system_to_system = hmath.Vector3.difference(last_known_location_vector, mercator_vector)

                    ranges += "{"

                    ranges += "'system': " + str(mercators[idx].System) + ";'range': " + str(system_to_system.norm / 1000)

                    ranges += "}"

                idx += 1

            messageStr_bytes = ranges.encode('utf-8')

            try:
                Range_Connection.sendall(messageStr_bytes)
            except Exception as e:
                print("Exception: " + str(e.__class__))

socket_logger.close()