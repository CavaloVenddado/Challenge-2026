from pybricks.hubs import PrimeHub
from pybricks.tools import StopWatch
import utils
import umath
from constants import BLE_CHANNEL_S2M, BLE_CHANNEL_M2S
from Buildings import MUSEUM, LIBRARY, BAKERY, DRUGSTORE, SCHOOL, CITYHALL, PARK

from DriveSubsystem import DriveSubsystem
from SensorSubsystem import SensorSubsystem
from Locate import Locate
from Map import Map
from TakeDeliver import TakeDeliver

drive = DriveSubsystem()
sensors = SensorSubsystem()
localizer = Locate(drive, sensors)
take_deliver = TakeDeliver(drive)

# drive.set_looking(90)
# drive.run_cm(0, 30)

localizer.localizeSelf()

while True:
    take_deliver
#     print(sensors.front_left_sensor.hsv())

# while True:
# drive.run_cm(0, 30)

# y = 0
# while y < 30:
#     y = drive.global_pos[1]
#     print(y)
#     drive.run(0, 300)