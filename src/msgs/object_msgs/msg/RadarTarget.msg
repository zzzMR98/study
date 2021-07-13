# This message describes a target (detection) from a RADAR sensor.

uint8 target_id    # ID of the target, as set by the sensor
float32 range       # Distance from sensor to target along sensor x-axis
float32 x       
float32 y
float32 speed       # Speed (range rate of change) of target along sensor x-axis
float32 azimuth     # Angle of target relative to sensor within x-y plane
