#!/usr/bin/env python3

import rclpy
import calendar
import datetime
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from RSP.parse_rinex import read_rinex
from RSP.satpos import calculate_satpos, calculate_positions
from Transform.geo import ecef_to_geodetic, geodetic_to_enu, ecef_to_enu

class RinexPublisher(Node):
    def __init__(self):
        super().__init__('rinex_publisher')

        self.sat_data = read_rinex('./wdc51200.24n')
        self.sat_pos = calculate_positions(self.sat_data)

        self.position_publisher = self.create_publisher(NavSatFix, 'satellite_position', 10)

        self.timer = self.create_timer(10, self.publish_rinex_data)

    def publish_rinex_data(self):
        date = datetime.datetime.now()
        unix_time = calendar.timegm(date.utctimetuple())
        unix_time_yesterday = unix_time - 86400

        for key, satellite_data in self.sat_data.items():
            if 'sqrt_A' not in satellite_data:
                self.get_logger().error(f"'sqrt_A' key is missing for satellite {key}. Cannot calculate position.")
                continue

            sat_prn = int(satellite_data['PRN'])
            epoch_data = satellite_data['EPOCH']
            year = int(epoch_data['YEAR'])
            month = int(epoch_data['MONTH'])
            day = int(epoch_data['DAY'])
            hour = int(epoch_data['HOUR'])
            minute = int(epoch_data['MINUTE'])
            second = int(float(epoch_data['SECOND']))

            rnx_dateTime = datetime.datetime((year+2000), month, day, hour, minute, second)
            rnx_unixTime = calendar.timegm(rnx_dateTime.timetuple())

            if((rnx_unixTime <= (unix_time_yesterday+3600)) and (rnx_unixTime >= (unix_time_yesterday-3600))):
                xk, yk, zk = calculate_satpos(satellite_data)
                llh = ecef_to_geodetic(xk, yk, zk)

                position_msg = NavSatFix()
                position_msg.header.stamp = self.get_clock().now().to_msg()
                position_msg.latitude = llh[0]
                position_msg.longitude = llh[1]
                position_msg.altitude = llh[2]
                self.position_publisher.publish(position_msg)

                self.get_logger().info(f"Satellite {sat_prn} Position: lat={llh[0]}, lon={llh[1]}, height={llh[2]}, Date={rnx_dateTime}, Time={rnx_unixTime}")
            else:
                self.get_logger().info("No Satellite")

def main(args=None):
    rclpy.init(args=args)
    node = RinexPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__== "__main__":
    main()
