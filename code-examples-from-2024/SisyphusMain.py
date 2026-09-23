from pybricks.hubs import PrimeHub
from pybricks.pupdevices import UltrasonicSensor
from constants import BLE_CHANNEL_S2M, BLE_CHANNEL_M2S, MILK_ULTRASONIC_SENSOR, PERSON_ULTRASONIC_SENSOR, PERSON_COLOR_SENSOR
from ClawSubsystem import ClawSubsystem
from pybricks.parameters import Color, Axis


claw = ClawSubsystem()

hub = PrimeHub(top_side = Axis.X, front_side = Axis.Z, broadcast_channel = BLE_CHANNEL_S2M, observe_channels = [BLE_CHANNEL_M2S])
obstacle = False

a = 100

icon = ((0, a, a, a, 0),
        (a, a, 0, a, a),
        (0, a, a, a, a),
        (a, a, 0, a, a),
        (0, a, a, a, 0)
        )

hub.display.icon(icon)
hub.light.on(Color.VIOLET)
del icon

# while True:
    # print((claw.close_sensor.filtered_color(), claw.close_sensor.hsv()))
    # print(claw.ultrasonic_2.distance())
    # print(claw.get_person())
    # print(hub.imu.heading())
    # print(claw.person_sensor.filtered_color())

garra = 1

while True:
    data = hub.ble.observe(BLE_CHANNEL_M2S)
    
    if (data != None):
        garra = data

        #based on the number recieved it opens or closes the claw
        if(garra == 2):
            claw.openClaw()
        elif(garra == 1):
            claw.closeClaw()
        elif(garra == 0):
            claw.stopClaw()

    #check if it sees anythin in 27.5cm
    if (claw.milk_sensor.distance() < 275):
        obstacle = True
    else:
        obstacle = False

    if (claw.ultrasonic.distance() < 275):
        obstacle_front = True
    else:
        obstacle_front = False

    if (claw.ultrasonic.distance() < 120):
        obstacle_front_close = True
    else:
        obstacle_front_close = False

    print(claw.ultrasonic.distance())

    claw.detect_close()
    output = (claw.get_person(), obstacle, obstacle_front, claw.place, obstacle_front_close, hub.imu.heading())

    # print(output)
    #Broadcast the building based on the pipe and if it is seing an obstacle
    hub.ble.broadcast(output)