from pybricks.hubs import PrimeHub
from pybricks.tools import StopWatch

from pybricks.parameters import Color

from constants import BLE_CHANNEL_S2M, BLE_CHANNEL_M2S
from Buildings import MUSEUM, LIBRARY, BAKERY, DRUGSTORE, SCHOOL, CITYHALL, PARK

from SensorSubsystem import SensorSubsystem
from DriveSubsystem import DriveSubsystem
from TakeDeliver import TakeDeliver
from Locate import Locate
from Map import Map

map = Map()
hub = PrimeHub(broadcast_channel = BLE_CHANNEL_M2S, observe_channels = [BLE_CHANNEL_S2M])
hub.light.on(Color.VIOLET)
drive = DriveSubsystem(map)
sensors = SensorSubsystem(drive)

stopwatch = StopWatch()

localizer = Locate(hub, drive, sensors, map)
take_deliver = TakeDeliver(drive, map, localizer, sensors)

# ==========================================================

# Describing the entraces <--
entrances = ((0, 4, LIBRARY, 180),
             (1, 1, SCHOOL, -90),
             (1, 1, CITYHALL, 90),
             (1, 3, CITYHALL, -90),
             (1, 3, LIBRARY, 90),
             (2, 0, SCHOOL, 0),
             (2, 2, DRUGSTORE, 180),
             (2, 4, MUSEUM, 180),
             (3, 1, BAKERY, -90),
             (3, 3, MUSEUM, 90),
             (4, 0, BAKERY, 0),
             (4, 0, PARK, 180),
             (4, 2, DRUGSTORE, 0),
             (4, 2, PARK, 180),
             (4, 4, PARK, 180))

map.set_entrances(entrances)
del entrances       

# Defining the obstacle positions (Places where you deliver people but cannot go throught) <--
obj = tuple(([1,0],[1,2],[1,4],[3,0],[3,2],[3,4]))
map.set_obstacles(obj)
del obj

# ==========================================================
# while True:
#     print(sensors.front_left_sensor.hsv(), "----", sensors.left_filter_alt())
    # drive.run(0, 600)
# ==========================================================

# Locate yourself <--
localizer.localizeSelf(True)

# Take and deliver Loop <--
while True:
    take_deliver.loop()