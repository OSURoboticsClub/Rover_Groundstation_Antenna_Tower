# Modes
# 0x00 Disabled
# 0x01 Manual Control
# 0x02 Automatic Control
# 0x03 Homing

#Error flags
# bit, flag
# 0 no comms with pan axis
# 1 no comms with elev axis
# 2 rover gps timeout exceeded
# 3 rover imu timeout exceeded
# 4 tower gps timeout exeeded
# 5 tower imu timeout exeeded
# 6 pan axis not homed (+ homing enforced)
# 7 elev axis not homed (+ homing enforced)
# 8-15 not assigned


uint8   operating_mode
float32 current_pan_deg
float32 current_elevation_deg
float32 current_pan_deg_sec
float32 current_elevation_deg_sec
float32 current_pan_setpoint
float32 current_elevation_setpoint
int16   errors

