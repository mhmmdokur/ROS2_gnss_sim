#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import datetime

class NMEAData:
    def __init__(self, latitude, longitude, altitude, date_time, prn_numbers, prn_count):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.date_time = date_time
        self.prn_numbers = prn_numbers
        self.prn_count = prn_count

def calculate_checksum(nmea_message):
    checksum = 0
    for char in nmea_message[1:]:
        checksum ^= ord(char)
    return f"{checksum:02X}"

def create_gga(data):
    time_str = data.date_time.strftime("%H%M%S")

    lat_degrees = int(data.latitude)
    lat_minutes = (data.latitude - lat_degrees) * 60.0
    lat_direction = 'N' if data.latitude >= 0 else 'S'

    lon_degrees = int(data.longitude)
    lon_minutes = (data.longitude - lon_degrees) * 60.0
    lon_direction = 'E' if data.longitude >= 0 else 'W'

    nmea_message = (
        f"$GPGGA,{time_str},"
        f"{abs(lat_degrees):02d}{lat_minutes:07.4f},{lat_direction},"
        f"{abs(lon_degrees):03d}{lon_minutes:07.4f},{lon_direction},"
        f"1,8,0.9,{data.altitude:.1f},M,0.0,M,,"
    )
    
    checksum = calculate_checksum(nmea_message)
    return f"{nmea_message}*{checksum}"

def create_gprmc(data):
    time_str = data.date_time.strftime("%H%M%S")
    date_str = data.date_time.strftime("%d%m%y")

    lat_degrees = int(data.latitude)
    lat_minutes = (data.latitude - lat_degrees) * 60.0
    lat_direction = 'N' if data.latitude >= 0 else 'S'

    lon_degrees = int(data.longitude)
    lon_minutes = (data.longitude - lon_degrees) * 60.0
    lon_direction = 'E' if data.longitude >= 0 else 'W'

    nmea_message = (
        f"$GPRMC,{time_str},A,"
        f"{abs(lat_degrees):02d}{lat_minutes:07.4f},{lat_direction},"
        f"{abs(lon_degrees):03d}{lon_minutes:07.4f},{lon_direction},"
        f"0.0,0.0,{date_str},,,"
    )

    checksum = calculate_checksum(nmea_message)
    return f"{nmea_message}*{checksum}"

def create_gpgsv(data):
    total_satellites = len(data.prn_numbers)
    num_messages = (total_satellites + 3) // 4  # Calculate the number of messages needed
    message_number = 1
    satellite_info = [f"{prn:02d},00,00,00" for prn in data.prn_numbers]
    
    gsv_messages = []
    
    for i in range(0, total_satellites, 4):
        satellites_chunk = satellite_info[i:i+4]
        nmea_message = (
            f"$GPGSV,{num_messages},{message_number},{total_satellites}," +
            ",".join(satellites_chunk)
        )
        checksum = calculate_checksum(nmea_message)
        gsv_messages.append(f"{nmea_message}*{checksum}")
        message_number += 1

    return gsv_messages

def create_gpgsa(data):
    prn_numbers_str = ",".join(f"{prn:02d}" for prn in data.prn_numbers[:data.prn_count])
    while len(prn_numbers_str.split(',')) < data.prn_count:
        prn_numbers_str += ","

    nmea_message = (
        f"$GPGSA,A,3,{prn_numbers_str},1.0,1.0,1.0"
    )

    checksum = calculate_checksum(nmea_message)
    return f"{nmea_message}*{checksum}"

def create_gpvtg(data):
    nmea_message = (
        f"$GPVTG,0.0,T,,M,0.0,N,0.0,K,A"
    )

    checksum = calculate_checksum(nmea_message)
    return f"{nmea_message}*{checksum}"

def create_zda():
    current_time = datetime.datetime.now(datetime.timezone.utc)
    time_str = current_time.strftime("%H%M%S")
    day = current_time.day
    month = current_time.month
    year = current_time.year

    nmea_message = (
        f"$GPZDA,{time_str},{day:02d},{month:02d},{year},00,00"
    )

    checksum = calculate_checksum(nmea_message)
    return f"{nmea_message}*{checksum}"

class NavSatFixSubscriber(Node):

    def __init__(self, prn_count):
        super().__init__('navsatfix_subscriber')
        self.prn_count = prn_count
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        current_time = datetime.datetime.now(datetime.timezone.utc)
        prn_numbers = list(range(1, self.prn_count + 1))  # Generate example PRN numbers
        data = NMEAData(
            latitude=msg.latitude,
            longitude=msg.longitude,
            altitude=msg.altitude,
            date_time=current_time,
            prn_numbers=prn_numbers,
            prn_count=self.prn_count
        )
        gga_message = create_gga(data)
        gprmc_message = create_gprmc(data)
        gpgsv_messages = create_gpgsv(data)
        gpgsa_message = create_gpgsa(data)
        gpvtg_message = create_gpvtg(data)
        zda_message = create_zda()
        
        self.get_logger().info(f'GGA Message: {gga_message}')
        self.get_logger().info(f'GPRMC Message: {gprmc_message}')
        for gsv_message in gpgsv_messages:
            self.get_logger().info(f'GPGSV Message: {gsv_message}')
        self.get_logger().info(f'GPGSA Message: {gpgsa_message}')
        self.get_logger().info(f'GPVTG Message: {gpvtg_message}')
        self.get_logger().info(f'ZDA Message: {zda_message}')

def main(args=None):
    prn_count = 12  # Set the desired PRN count here, or fetch it from arguments
    rclpy.init(args=args)

    navsatfix_subscriber = NavSatFixSubscriber(prn_count)

    rclpy.spin(navsatfix_subscriber)

    navsatfix_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
