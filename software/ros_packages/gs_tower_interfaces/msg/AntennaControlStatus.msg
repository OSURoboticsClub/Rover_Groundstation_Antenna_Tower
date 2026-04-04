# Modes
# 0x00 Disabled
# 0x01 Manual Control
# 0x02 Automatic Control
# 0x03 Homing

uint8 operating_mode
float32 current_pan_deg
float32 current_elevation_deg
float32 current_pan_deg_sec
float32 current_elevation_deg_sec
bool at_elevation_setpoint
bool at_pan_setpoint
char[] status_msg

