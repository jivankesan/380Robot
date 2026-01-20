# Line observation from vision pipeline
# Describes the detected line relative to the robot

builtin_interfaces/Time stamp

# Lateral error in meters (+right of center)
float32 lateral_error_m

# Heading error in radians (+counter-clockwise)
float32 heading_error_rad

# Curvature estimate (1/radius in meters, signed)
float32 curvature_1pm

# Whether the line detection is valid
bool valid
