import serial
import time
import sys
import os
from time import sleep
# Configure the serial port
# Replace 'COMx' with your actual serial port (e.g., 'COM1' on Windows, '/dev/ttyUSB0' on Linux)
# Set the baudrate to match your device
ser = serial.Serial('/dev/ttyGPS', baudrate=38400, timeout=1) # timeout in seconds
config_dir = "/home/makemorerobot/source/Rover_Groundstation_Antenna_Tower/software/util/configuration/configuration.txt"

cur_dir = os.getcwd()


while True:
    try:
        print(f"Opening configuration file: {config_dir}")
        config_file = open(config_dir, 'r')
        all_lines = config_file.readlines()
        print("Sending configuration commands...")
    
        for line in all_lines:
            command_str = line[3:]
            binary_data = bytes.fromhex(command_str)
            ser.write(binary_data)
            print(command_str)
        print("Successfull write")    
        break

    except serial.SerialException as e:
        print(f"Serial port error: {e}\n Reconnecting...")
        sleep(1)
    except KeyboardInterrupt:
        print("Program terminated by user.")
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed.")
