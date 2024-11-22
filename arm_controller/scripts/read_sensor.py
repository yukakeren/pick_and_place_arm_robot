#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Float32

try:
    ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
except serial.SerialException as e:
    rospy.logerr(f"Failed to open serial port: {e}")
    exit(1)

def read_distance():
    try:
        line = ser.readline().decode('utf-8').strip()
        if line:
            rospy.loginfo(f"Received raw data: {line}")
            try:
                # Convert the line to a float
                distance = float(line)
                return distance
            except ValueError:
                rospy.logwarn(f"Invalid data received: {line}")
    except serial.SerialException as e:
        rospy.logerr(f"Serial read error: {e}")
    return None

def publish_sensor_data(event):
    distance = read_distance()
    if distance is not None:
        msg = Float32()
        msg.data = distance
        pub.publish(msg)
        rospy.loginfo(f"Published distance: {distance}")
    else:
        rospy.logwarn("No valid sensor data to publish.")

if __name__ == "__main__":
    rospy.init_node('sensor_data_publisher', anonymous=True)
    pub = rospy.Publisher('sensor_data', Float32, queue_size=10)
    publish_rate = rospy.get_param('~publish_rate', 60.0)
    rospy.Timer(rospy.Duration(1.0 / publish_rate), publish_sensor_data)

    rospy.loginfo(f"Sensor data publisher started at {publish_rate} Hz.")

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        ser.close()
        rospy.loginfo("Serial connection closed.")
