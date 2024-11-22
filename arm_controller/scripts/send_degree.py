#!/usr/bin/env python3

import rospy
from custom_msg.msg import degrees
import serial
import time
import numpy as np

def interpolate_np(angles):
    angle_range = (-90.0, 90.0)
    digital_range = (1000, 5000)
    digital_units = np.interp(angles, angle_range, digital_range)
    return digital_units.astype(int)

class ServoController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('servo_controller', anonymous=True)
        
        # Serial communication parameters
        self.port = '/dev/ttyACM1'
        self.baudrate = 9600
        self.serial_connection = None
        
        # Initialize serial connection
        try:
            self.serial_connection = serial.Serial(self.port, self.baudrate)
            rospy.loginfo(f"Successfully connected to {self.port}")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open serial port: {e}")
            return
        
        # Subscribe to the degree topic
        self.degree_subscriber = rospy.Subscriber("degree", degrees, self.degree_callback)
        
        rospy.loginfo("Servo controller node initialized")

    def degree_callback(self, msg):
        """
        Callback function for processing incoming degree messages
        """
        try:
            # Extract angles from the message
            angles = [
                msg.teta0,
                msg.teta1,
                msg.teta2,
                msg.teta3,
                msg.teta4
            ]
            
            # Convert angles to digital units using interpolation
            digital_units = interpolate_np(angles)
            
            # Format the digital units with padding
            formatted_units = ' '.join(f'{value:04d}' for value in digital_units)
            
            # Send to serial port
            if self.serial_connection and self.serial_connection.is_open:
                self.serial_connection.write(formatted_units.encode())
                rospy.loginfo(f"Sent angles: {angles}")
                rospy.loginfo(f"Sent digital units: {formatted_units}")
            else:
                rospy.logerr("Serial connection is not available")
                
        except Exception as e:
            rospy.logerr(f"Error processing message: {e}")

    def cleanup(self):
        """
        Cleanup function to close serial connection
        """
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            rospy.loginfo("Serial connection closed")

def main():
    controller = ServoController()
    
    try:
        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.cleanup()

if __name__ == '__main__':
    main()