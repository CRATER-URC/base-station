import rclpy
import traceback

from math import pi, sin, cos, atan2, sqrt

from rclpy.node         import Node
from geometry_msgs.msg  import PointStamped

import time
import board
import busio
import serial

import adafruit_gps

gps = None

#
#   GPS Node Class
#
class GPSNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)
        
        # Subscribers
        
        # Publishers
        self.pubgps = self.create_publisher(PointStamped, 'gps_pose', 10)
        
        # Report and return.
        self.get_logger().info("GPS running")


        # Create a serial connection for the GPS connection using default speed and
        # a slightly higher timeout (GPS modules typically update once a second).
        # These are the defaults you should use for the GPS FeatherWing.
        # For other boards set RX = GPS module TX, and TX = GPS module RX pins.
        uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=10)

        # for a computer, use the pyserial library for uart access
        # uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)


        # If using I2C, we'll create an I2C interface to talk to using default pins
        # i2c = board.I2C()  # uses board.SCL and board.SDA
        # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

        # Create a GPS module instance.
        gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial
        # gps = adafruit_gps.GPS_GtopI2C(i2c, debug=False)  # Use I2C interface

        # Initialize the GPS module by changing what data it sends and at what rate.
        # These are NMEA extensions for PMTK_314_SET_NMEA_OUTPUT and
        # PMTK_220_SET_NMEA_UPDATERATE but you can send anything from here to adjust
        # the GPS module behavior:
        #   https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf

        # Turn on the basic GGA and RMC info (what you typically want)
        gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        # Turn on just minimum info (RMC only, location):
        # gps.send_command(b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        # Turn off everything:
        # gps.send_command(b'PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        # Turn on everything (not all of it is parsed!)
        # gps.send_command(b'PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0')

        # Set update rate to once a second (1hz) which is what you typically want.
        gps.send_command(b"PMTK220,1000")
        # Or decrease to once every two seconds by doubling the millisecond value.
        # Be sure to also increase your UART timeout above!
        # gps.send_command(b'PMTK220,2000')
        # You can also speed up the rate, but don't go too fast or else you can lose
        # data during parsing.  This would be twice a second (2hz, 500ms delay):
        # gps.send_command(b'PMTK220,500')

    
    def cb_timer(self):
        # Make sure to call gps.update() every loop iteration and at least twice
        # as fast as data comes from the GPS unit (usually every second).
        # This returns a bool that's true if it parsed new data (you can ignore it
        # though if you don't care and instead look at the has_fix property).
        gps.update()
        if gps.has_fix:
            msg = new PointStamped()
            msg.point = new Point(gps.latitude, gps.longitude, gps.altitude_m)
            self.pubgps.publish(msg)

    # Shutdown
    def shutdown(self):
        # Nothing to do except shut down the node.
        self.destroy_node()

#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the Move node.
    node = GPSNode('gps')

    # Spin the node until interrupted.
    try:
        rclpy.spin(node)
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
