import board
import busio
import adafruit_bno08x
import adafruit_bno08x.i2c
import adafruit_bno08x.uart
import serial
import time

def main():
	i2c = busio.I2C(board.SCL, board.SDA)
	imu = adafruit_bno08x.i2c.BNO08X_I2C(i2c)
	#uart = serial.Serial("/dev/serial0", 115200)
	#imu = adafruit_bno08x.uart.BNO08X_UART(uart)

	imu.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
	imu.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
	imu.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION)
	imu.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)

	while(True):
		try:
			quat = imu.quaternion
			print(f"Orientation Quat: \n\ti: {quat[0]}, \n\tj: {quat[1]}, \n\tk: {quat[2]}, \n\tw: {quat[3]}")
			print("Linear Acceleration: " + str(imu.linear_acceleration))
			print("Angular Accleration: " + str(imu.gyro))
			print("Magnetometer: " + str(imu.magnetic))
			time.sleep(0.5)
		except Exception as e:
			print (f"Failed to parse packet! {type(e)}: {e}")


if __name__ == "__main__":
	main()
