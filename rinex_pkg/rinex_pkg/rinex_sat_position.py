#!/usr/bin/env python3

import rclpy
import calendar
import datetime
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from RSP.parse_rinex import read_rinex
from RSP.satpos import calculate_satpos, calculate_positions
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

        self.date = datetime.datetime.now();
        self.unix_time = calendar.timegm(self.date.utctimetuple())
        
        for key, satellite_data in self.sat_data.items():
            self.epoch_data = satellite_data['EPOCH']
            self.year = int(self.epoch_data['YEAR'])
            self.month = int(self.epoch_data['MONTH'])
            self.day = int(self.epoch_data['DAY'])
            self.hour = int(self.epoch_data['HOUR'])
            self.minute = int(self.epoch_data['MINUTE'])
            self.second = int(float(self.epoch_data['SECOND']))  # Convert string to float, then to int

            
            self.rnx_dateTime = datetime.datetime(self.year, self.month, self.day, self.hour, self.minute, self.second)
            self.rnx_unixTime = calendar.timegm(self.rnx_dateTime.timetuple())


            if((self.unix_time <= self.rnx_unixTime+3600) or (self.unix_time >= self.rnx_unixTime-3600)):
                xk, yk, zk = calculate_satpos(self.sat_data)
                self.llh = ecef_to_geodetic(xk, yk, zk)
                
                # NavSatFix mesajını oluştur
                msg = NavSatFix()
                msg.latitude = self.llh[0]
                msg.longitude = self.llh[1]
                msg.altitude = self.llh[2]

                # Yayın yapma
                self.publisher.publish(msg)
                self.get_logger().info(self.llh)



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

