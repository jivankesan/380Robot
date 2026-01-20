# Claw actuator command

# Mode constants
uint8 MODE_OPEN = 0
uint8 MODE_CLOSE = 1
uint8 MODE_HOLD = 2

# Command mode (0=open, 1=close, 2=hold)
uint8 mode

# Optional position (0.0 to 1.0, where 0=fully open, 1=fully closed)
float32 position
