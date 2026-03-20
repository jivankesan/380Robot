# Claw actuator command

# Mode constants
uint8 MODE_OPEN = 0
uint8 MODE_CLOSE = 1
uint8 MODE_HOLD = 2
uint8 MODE_ROTATE = 3

# Command mode (0=open, 1=close, 2=hold, 3=rotate)
uint8 mode

# Optional position (0.0 to 1.0, where 0=fully open/down, 1=fully closed/up)
float32 position
