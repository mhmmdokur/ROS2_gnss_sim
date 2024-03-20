#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from RSP.parse_rinex import read_rinex
from RSP.satpos import calculate_satpos, calculate_positions


def main(args=None):
    rclpy.init(args=args)
    node = Node("rinex_test")

    sat_data = read_rinex('./gmez0380.21n')
    sat_pos = calculate_positions(sat_data)
    #print(sat_pos)

    node.get_logger().info(sat_pos)
    rclpy.shutdown()

if __name__== "__main__":
    main()

