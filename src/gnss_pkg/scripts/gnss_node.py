#!/usr/bin/env python

import serial
import time
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped

ser = serial.Serial('/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00', 115200)

def gnss_publisher():
    fix_pub = rospy.Publisher('fix', NavSatFix, queue_size=1)
    rospy.init_node('gnss_node', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        process_data(fix_pub)
        rate.sleep()

def process_data(fix_pub):
    line = ser.readline()
    
    if line[3:6] == b'GGA':
        print("**GGA")
        print(line.decode().split(','))
        split = line.decode().split(',')
        utc_time = split[1]

        lat = split[2]
        if(lat != ''): #if lat is not empty
            lat = float(lat)
            lat = lat//100 + (lat % 100)/60
            if(split[3] == 'S'):
                lat = -lat

        lon = split[4]
        if(lon != ''): #if lon is not empty
            lon = float(lon)
            lon = lon//100 + (lon % 100)/60
            if(split[5] == 'W'):
                lon = -lon

        fix = split[6]
        sats = split[7]

        print("Lat: " + str(lat))
        print("Lon: " + str(lon))
        if(fix == '0'):
            print("No Fix")
        elif(fix == '1'):
            print("2D/3D")
        elif(fix == '2'):
            print("DGNSS")
        elif(fix == '4'):
            print("RTK Fix")
        elif(fix == '5'):
            print("RTK Float")

        msg = NavSatFix()
        msg.header.stamp = rospy.Time.now() # time of measurement
        msg.header.frame_id = "gps" # frame of measurement
        msg.latitude = lat
        msg.longitude = lon
        
        msg.altitude = float(split[9]) if (msg.altitude) else 0.0
        msg.status.status = int(fix)
        msg.status.service = 1
        msg.position_covariance_type = 0
        fix_pub.publish(msg)

if __name__ == '__main__':
    try:
        gnss_publisher()
    except rospy.ROSInterruptException:
        pass
