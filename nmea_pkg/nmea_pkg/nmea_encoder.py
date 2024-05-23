import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import datetime

class NMEAData:
    def __init__(self, latitude, longitude, altitude, date_time):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.date_time = date_time

def calculate_checksum(nmea_message):
    checksum = 0
    for char in nmea_message[1:]:
        checksum ^= ord(char)
    return f"{checksum:02X}"

def create_gga(data):
    time_str = data.date_time.strftime("%H%M%S")

    lat_degrees = int(data.latitude)
    lat_minutes = (data.latitude - lat_degrees) * 60.0
    lat_direction = 'N' if lat_degrees >= 0 else 'S'

    lon_degrees = int(data.longitude)
    lon_minutes = (data.longitude - lon_degrees) * 60.0
    lon_direction = 'E' if lon_degrees >= 0 else 'W'

    nmea_message = (
        f"$GPGGA,{time_str},"
        f"{abs(lat_degrees):02d}{lat_minutes:07.4f},{lat_direction},"
        f"{abs(lon_degrees):03d}{lon_minutes:07.4f},{lon_direction},"
        f"1,8,0.9,{data.altitude:.1f},M,0.0,M,,"
    )
    
    checksum = calculate_checksum(nmea_message)
    return f"{nmea_message}*{checksum}"

class NavSatFixSubscriber(Node):

    def __init__(self):
        super().__init__('navsatfix_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        current_time = datetime.datetime.now(datetime.timezone.utc)
        data = NMEAData(
            latitude=msg.latitude,
            longitude=msg.longitude,
            altitude=msg.altitude,
            date_time=current_time
        )
        gga_message = create_gga(data)
        self.get_logger().info(f'GGA Message: {gga_message}')

def main(args=None):
    rclpy.init(args=args)

    navsatfix_subscriber = NavSatFixSubscriber()

    rclpy.spin(navsatfix_subscriber)

    navsatfix_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
