from pybricks.parameters import Port
import umath

#PID
DRIVE_P = 60
DRIVE_I = 0
DRIVE_D = 30

POS_P = 60
POS_I = 0
POS_D = 30

TURN_P = 16
TURN_I = 0
TURN_D = 16

# lx is the distance of one wheel to another in the x axis.
# ly is the same but in the y axis.
# this measures the distance of any wheel center to the 
# actual center of the robot.
LY = 7.978 / 2
LX = 12.818 / 2

K_SIZE = LX+LY

#Motors Minos
F_R_PORT = Port.F
B_R_PORT = Port.E
F_L_PORT = Port.B
B_L_PORT = Port.A

#Wheels angle
F_R_POS = 45
B_R_POS = 135
F_L_POS = -45
B_L_POS = -135

MAX_SPEED = 700 #degrees per second

#Centimeters in one degree of the wheel circunference
WHEEL_RADIUS = 3


# 15 (Wall to centre of place) - (4.2 (Centre of color sensor to wheel centre) + (7.966/2) (Wheel centre to centre of robot, divided by two))
WALL_TO_ROAD = 8

#Claw 
CLAW_PORT = Port.C   #Sisyphus
CLAW_PORT_2 = Port.A #Sisyphus

# #Sensors
LEFT_COLOR_SENSOR = Port.D          # Minos
RIGHT_COLOR_SENSOR = Port.C         # Minos
PERSON_COLOR_SENSOR = Port.E        # Sisyphus
PERSON_ULTRASONIC_DETECT = Port.B

PERSON_ULTRASONIC_SENSOR = Port.D   # Sisyphus
MILK_ULTRASONIC_SENSOR = Port.F     # Sisyphus

#Bluetooth communication
BLE_CHANNEL_S2M = 144 #Sisyphus to Minos
BLE_CHANNEL_M2S = 141 #Minos to Sisyphus