#!/usr/bin/env python3

import rclpy
import calendar
import datetime
from rclpy.node import Node
from RSP.parse_rinex import read_rinex
from RSP.satpos import calculate_satpos, calculate_positions


def main(args=None):
    rclpy.init(args=args)
    node = Node("rinex_test")

    sat_data = read_rinex('./gmez0380.21n')
    sat_pos = calculate_positions(sat_data)

    date = datetime.datetime.now();
    unix_time = calendar.timegm(date.utctimetuple())

    for key, satellite_data in sat_data.items():
        if 'sqrt_A' not in satellite_data:
            node.get_logger().error(f"'sqrt_A' key is missing for satellite {key}. Cannot calculate position.")
            continue

        epoch_data = satellite_data['EPOCH']
        year = int(epoch_data['YEAR'])
        month = int(epoch_data['MONTH'])
        day = int(epoch_data['DAY'])
        hour = int(epoch_data['HOUR'])
        minute = int(epoch_data['MINUTE'])
        second = int(float(epoch_data['SECOND']))

        rnx_dateTime = datetime.datetime((year+2000), month, day, hour, minute, second)
        rnx_unixTime = calendar.timegm(rnx_dateTime.timetuple())

        if((unix_time <= rnx_unixTime+3) or (unix_time >= rnx_unixTime-3)):
            xk, yk, zk = calculate_satpos(satellite_data)
            node.get_logger().info(f"Satellite {key} Position: x={xk}, y={yk}, z={zk}, Date={rnx_dateTime},Time={rnx_unixTime}")


    #print(sat_pos)

    rclpy.shutdown()

if __name__== "__main__":
    main()

