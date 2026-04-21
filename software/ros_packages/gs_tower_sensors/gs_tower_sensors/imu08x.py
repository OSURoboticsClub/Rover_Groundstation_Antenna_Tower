#!/usr/bin/env python3
#####################################
# Imports
#####################################
# Python native imports
from time import time, sleep
import adafruit_bno08x.i2c
import board
import busio
import adafruit_bno08x
import math
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node
from gs_tower_sensors.coordinate_math import *

#####################################
# Global Variables
#####################################
NODE_NAME = "gs_tower_imu"

DEFAULT_IMU_TOPIC = "gs_tower_imu/data"
DEFAULT_MAG_TOPIC = "gs_tower_imu/mag"
IMU_HEADING_TOPIC = "gs_tower_imu/heading"

DEFAULT_HERTZ = 100  # Changed from 1000 to reasonable rate


#####################################
# IMU Node Class Definition
#####################################
class IMUNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        self.imu_data_topic = self.declare_parameter("imu_data_topic", DEFAULT_IMU_TOPIC).value
        self.mag_topic = self.declare_parameter("mag_topic", DEFAULT_MAG_TOPIC).value
        self.imu_heading_topic = self.declare_parameter("imu_heading_topic", IMU_HEADING_TOPIC).value
        self.wait_time = 1.0 / self.declare_parameter('hertz', DEFAULT_HERTZ).value

        # Publishers
        self.imu_data_publisher = self.create_publisher(Imu, self.imu_data_topic, 10)
        self.mag_publisher = self.create_publisher(MagneticField, self.mag_topic, 10)
        self.imu_heading_publisher = self.create_publisher(Float32, self.imu_heading_topic, 10)
        
        self.get_logger().info(f'Publishing at {1.0/self.wait_time} Hz')
        
        # Initialize I2C and BNO08X
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = adafruit_bno08x.i2c.BNO08X_I2C(self.i2c)
        
        # Magnetic declination (adjust for your location)
        self.magnetic_declination = 0
        
        # Configure sensor, enable reports
        self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
        self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
        self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION)
        self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)

        # Start timer
        self.timer = self.create_timer(self.wait_time, self.main_loop)

    def main_loop(self):
        """Main processing loop"""
        try:
            self.publish_imu_data()
            self.publish_heading()
        except Exception as error:
            self.get_logger().error(f'Error in main loop: {error}')

    def publish_imu_data(self):
        try:
            # Get current time
            current_time = self.get_clock().now().to_msg()
            
            # === Publish standard IMU message ===
            imu_msg = Imu()
            imu_msg.header.stamp = current_time
            imu_msg.header.frame_id = "imu_link"
            
            # Orientation (quaternion from BNO055 sensor fusion)
            quat = self.imu.quaternion
            if quat and None not in quat:
                imu_msg.orientation.x = float(quat[0])
                imu_msg.orientation.y = float(quat[1])
                imu_msg.orientation.z = float(quat[2])
                imu_msg.orientation.w = float(quat[3])
                
                # Orientation covariance (adjust based on your sensor accuracy)
                imu_msg.orientation_covariance[0] = 0.01
                imu_msg.orientation_covariance[4] = 0.01
                imu_msg.orientation_covariance[8] = 0.01
            else:
                # If quaternion not available, mark as invalid
                imu_msg.orientation_covariance[0] = -1
            
            # Angular velocity (gyroscope) in rad/s
            gyro = self.imu.gyro
            if gyro and None not in gyro:
                imu_msg.angular_velocity.x = float(gyro[0])
                imu_msg.angular_velocity.y = float(gyro[1])
                imu_msg.angular_velocity.z = float(gyro[2])
                
                # Angular velocity covariance
                imu_msg.angular_velocity_covariance[0] = 0.001
                imu_msg.angular_velocity_covariance[4] = 0.001
                imu_msg.angular_velocity_covariance[8] = 0.001
            else:
                imu_msg.angular_velocity_covariance[0] = -1
            
            # Linear acceleration in m/s²
            accel = self.imu.linear_acceleration  # Gravity already removed
            if accel and None not in accel:
                imu_msg.linear_acceleration.x = float(accel[0])
                imu_msg.linear_acceleration.y = float(accel[1])
                imu_msg.linear_acceleration.z = float(accel[2])
                
                # Linear acceleration covariance
                imu_msg.linear_acceleration_covariance[0] = 0.01
                imu_msg.linear_acceleration_covariance[4] = 0.01
                imu_msg.linear_acceleration_covariance[8] = 0.01
            else:
                imu_msg.linear_acceleration_covariance[0] = -1
            
            self.imu_data_publisher.publish(imu_msg)
            
            # === Publish magnetometer data separately ===
            mag = self.imu.magnetic
            if mag and None not in mag:
                mag_msg = MagneticField()
                mag_msg.header.stamp = current_time
                mag_msg.header.frame_id = "imu_link"
                
                # Magnetometer data in micro-Tesla (µT)
                mag_msg.magnetic_field.x = float(mag[0])
                mag_msg.magnetic_field.y = float(mag[1])
                mag_msg.magnetic_field.z = float(mag[2])
                
                # Magnetometer covariance
                mag_msg.magnetic_field_covariance[0] = 0.01
                mag_msg.magnetic_field_covariance[4] = 0.01
                mag_msg.magnetic_field_covariance[8] = 0.01
                
                self.mag_publisher.publish(mag_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing IMU data: {e}')

    def publish_heading(self):
        """Publish compass heading in degrees"""
        try:
            quat = self.imu.quaternion
            hdg = None
            if quat is not None:
                hdg = angle(planeProj(Vec3(quat[0], quat[1], quat[2]), Vec3(0, 0, 1)), Vec3(1, 0, 0), Vec3(0, 0, 1))
            if hdg is not None:
                # Get heading and apply magnetic declination
                
                heading_msg = Float32()
                heading_msg.data = float(hdg)
                self.imu_heading_publisher.publish(heading_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing heading: {e}')


def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
