#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from RSP.parse_rinex import read_rinex
from RSP.satpos import calculate_satpos, calculate_positions, calculate_single_positions
from Transform.geo import ecef_to_geodetic, geodetic_to_enu, ecef_to_enu


class NavSatFixSubscriber(Node):
    def __init__(self):
        super().__init__('navsatfix_subscriber')

        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.fix_callback,
            10)

    def fix_callback(self, msg):
        # /fix topic'inden gelen verileri işle
        self.enlem = msg.latitude
        self.boylam = msg.longitude
        self.irtifa = msg.altitude

class RinexPublisher(Node):
    def __init__(self):
        super().__init__('rinex_publisher')

        # Yayın oluşturma
        self.publisher = self.create_publisher(NavSatFix, '/rinex_data', 10)
        # Rinex dosyasından veri okuma
        self.sat_data = read_rinex('./gmez0380.21n')
        # Yayın işlemi
        self.timer = self.create_timer(1, self.publish_rinex_data)

    def publish_rinex_data(self):
        # Veri işleme ve yayın yapma işlemleri
        #sat_pos = calculate_positions(self.sat_data)
        
        #first_key = next(iter(sat_pos))
        #llh = ecef_to_geodetic(sat_pos[first_key]['x'], sat_pos[first_key]['y'], sat_pos[first_key]['z'])
        # İlk rinex verisini almak için
        sat_pos = calculate_single_positions(self.sat_data)
        single_rinex_data = next(sat_pos)
        llh = ecef_to_geodetic(single_rinex_data['x'], single_rinex_data['y'], single_rinex_data['z'])

        # NavSatFix mesajını oluştur
        msg = NavSatFix()
        msg.latitude = llh[0]
        msg.longitude = llh[1]
        msg.altitude = llh[2]

        # Yayın yapma
        self.publisher.publish(msg)


def main(args=None):
    
    rclpy.init(args=args)
    navSatFixSubsNode = NavSatFixSubscriber()
    rinexPublisherNode = RinexPublisher()

    # Her iki Node'u da aynı anda çalıştır
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(navSatFixSubsNode)
    executor.add_node(rinexPublisherNode)
    executor.spin()

    # Spin döngüsünden çıktığımızda ROS 2'yi kapat
    rclpy.shutdown()


if __name__== "__main__":
    main()

