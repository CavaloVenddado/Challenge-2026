from pybricks.hubs import PrimeHub
from pybricks.parameters import Color
from constants import WALL_TO_ROAD, BLE_CHANNEL_M2S, BLE_CHANNEL_S2M
from DriveSubsystem import DriveSubsystem
from SensorSubsystem import SensorSubsystem
from Locate import Locate
from Map import Map
from pybricks.tools import StopWatch

class TakeDeliver():
    def __init__(self, inputDrive:DriveSubsystem, inputMap:Map, inputLocate:Locate, inputSensors:SensorSubsystem) -> None:
        self.hub = PrimeHub(broadcast_channel = BLE_CHANNEL_M2S, observe_channels = [BLE_CHANNEL_S2M])

        self.drive = inputDrive
        self.map = inputMap
        self.locate = inputLocate
        self.sensors = inputSensors

        self.stopwatch = StopWatch()
        self.claw_open = 2
        self.data = (-1, False, False, -1, False, -1, 0)
        self.path = None
        self.entrance_angle = 0

        A = 100

        self.icon_id = ((0, A, A, A, 0),
                        (0, 0, 0, A, 0),
                        (0, 0, A, A, 0),
                        (0, 0, 0, 0, 0),
                        (0, 0, A, 0, 0))
        
        self.icon_found = ((0, 0, A, 0, 0),
                           (0, 0, A, 0, 0),
                           (0, 0, A, 0, 0),
                           (0, 0, 0, 0, 0),
                           (0, 0, A, 0, 0))
        
        self.icon_take = ((0, 0, A, 0, 0),
                          (0, A, A, A, 0),
                          (A, 0, A, 0, A),
                          (0, 0, A, 0, 0),
                          (0, 0, A, 0, 0))
        
        self.icon_deliver = ((0, 0, A, 0, 0),
                             (0, 0, A, 0, 0),
                             (A, 0, A, 0, A),
                             (0, A, A, A, 0),
                             (0, 0, A, 0, 0))

    def take(self):
        #align, go back, close the claw a little, go foward, take the pipe and go back
        self.hub.display.icon(self.icon_take)

        self.drive.run_cm(self.drive.target_looking, -2)

        self.drive.run_cm(self.drive.target_looking + 90, 10)
        self.drive.brake()

        self.hub.ble.broadcast(1)
        watch = StopWatch()
        while watch.time() < 2500:
            self.drive.run(self.drive.target_looking, 150)

        self.claw_open = 0
        self.hub.ble.broadcast(self.claw_open)

        if(self.hub.ble.observe(BLE_CHANNEL_S2M) != None):
            self.data = self.hub.ble.observe(BLE_CHANNEL_S2M)
        else:
            self.data = (-1, False, False, -1, False, -1, 0)
       

    def deliver(self):
        #go forward untill see yellow, open claw, go back
        self.hub.display.icon(self.icon_deliver)

        self.sensors.ground_reset()

        while(self.sensors.on_wall() == False or self.sensors.on_wall() == None):
            self.drive.run(self.entrance_angle - 90, 300)
        self.align_door()
        self.drive.brake()

        self.drive.run_cm(self.drive.target_looking, -10)
        self.drive.brake()

        self.sensors.ground_reset()

        while(self.sensors.on_wall() == False or self.sensors.on_wall() == None):
            self.drive.run(self.entrance_angle - 90, 200)

        self.drive.run_cm(self.entrance_angle - 90, 5) #offset to deliver pipe
        self.drive.brake()

        self.claw_open = 2
        self.hub.ble.broadcast(self.claw_open)
        self.drive.brake()

        self.stopwatch.reset()
        while (self.stopwatch.time() < 1000):
            continue
        self.claw_open = 0
        self.hub.ble.broadcast(self.claw_open)

        self.drive.run_cm(self.entrance_angle - 90, -WALL_TO_ROAD -5) #offset to continue and follow path

    def loop(self):

        self.hub = PrimeHub(broadcast_channel = BLE_CHANNEL_M2S, observe_channels = [BLE_CHANNEL_S2M])
        self.drive.reset_odometry()
        self.hub.display.icon(self.icon_id)
        dir = 1
        
        # self.drive.run_cm(self.drive.target_looking, WALL_TO_ROAD + 2) #offset to scan pipes
        
        while (True):
            
            self.get_to_blue()
            self.data = (-1, False, False, -1, False, -1, 0)
            while(self.data[0] == -1):
                #checks if it is recieveing data from HubToHub, if not, make it a fixed value
                if(self.hub.ble.observe(BLE_CHANNEL_S2M) != None):
                    self.data = self.hub.ble.observe(BLE_CHANNEL_S2M)
                else:
                    self.data = (-1, False, False, -1, False, -1, 0)
                    
                #prevent going out of the arena if it doesnt see any pipe
                if(self.drive.global_pos[0] > 120):
                    self.realign()
                    return
                # elif(self.drive.global_pos[0] < 5):
                self.drive.run(self.drive.target_looking +90, 250)

            self.take()

            self.stopwatch.reset()
            while (self.stopwatch.time() < 500):
                continue

            #checks if it realy took the pipe
            if(self.hub.ble.observe(BLE_CHANNEL_S2M) != None):
                self.data = self.hub.ble.observe(BLE_CHANNEL_S2M)
            else:
                self.data = (-1, False, False, -1, False, -1, 0)

            if(self.data[3] == -1):
                # self.get_to_blue()
                self.claw_open = 2
                self.hub.ble.broadcast(self.claw_open)
                watch = StopWatch()
                while watch.time() < 1000:
                    continue

                self.hub.ble.broadcast(0)
                
                self.drive.run_cm(self.drive.target_looking, -15)
                # self.drive.run_cm(self.drive.target_looking +90, 10)
                return
            else:
                self.drive.run_cm(self.drive.target_looking, -WALL_TO_ROAD - 6)

            #localize, follow path and set the entrance as occupied
            self.realign()
            self.drive.set_odometry((0, 0), -90)

            on_end = False
            current = tuple([0, 0])
            
            while not on_end:
                #make the path to the best entrance
                if (self.data[3] != -1):
                    
                    self.hub.display.icon(self.icon_found)
                    all_occupied = True
                    place_entrances = self.map.get_specified_entrance(self.data[3])

                    # print(place_entrances)

                    for entrance in place_entrances:
                        this_place = self.map.map[entrance[0]][entrance[1]]
                        if (this_place.obstacle == False and this_place.occupied == False):
                            self.entrance_angle = entrance[2]
                            self.entrance_loc = (entrance[0], entrance[1])

                            self.path = self.map.make_path(tuple(current), tuple([entrance[0],entrance[1]]))
                            all_occupied = False
                            break

                    if all_occupied:
                        for entrance in place_entrances:
                            if (self.map.map[entrance[0]][entrance[1]].obstacle == False):
                                self.entrance_angle = entrance[2]
                                self.entrance_loc = (entrance[0], entrance[1])

                                self.path = self.map.make_path(tuple(current), tuple([entrance[0],entrance[1]]))
                                break
                        

                    on_end = self.drive.follow_path(self.path, self.entrance_angle - 90)

                    current = tuple([round(self.drive.global_pos[0]/30), round(self.drive.global_pos[1]/30)])

            self.map.occupy(self.entrance_loc[0], self.entrance_loc[1])

            self.deliver()

            #reset odometry, make and follow path to return to the start corner
            self.drive.set_odometry([self.entrance_loc[0]*30, self.entrance_loc[1]*30], self.entrance_angle - 90)
            
            current = tuple([self.entrance_loc[0], self.entrance_loc[1]])
            on_end = False
            while not on_end:
                self.path = self.map.make_path(current, tuple([0, 1]))
                on_end = self.drive.follow_path(self.path, -90)
                current = tuple([round(self.drive.global_pos[0]/30), round(self.drive.global_pos[1]/30)])

            self.drive.reset_odometry()
            self.drive.brake()
            self.realign()


    def align_door(self):
        while(self.sensors.at_color_left() != Color.YELLOW or self.sensors.at_color_right() != Color.YELLOW):
            mydir = 0
            if(self.sensors.at_color_left() != Color.YELLOW):
                mydir += 1

            if(self.sensors.at_color_right() != Color.YELLOW):
                mydir -= 1
            
            self.drive.run(self.drive.target_looking + (mydir * 90), 250)
        
    def get_to_blue(self):
        self.sensors.ground_reset()
        while not (self.sensors.on_blue()):
            self.drive.run(self.drive.target_looking, 300)
        self.drive.run_cm(self.drive.target_looking, 2)
            
    def realign(self):
        self.sensors.ground_reset()
        while (self.sensors.on_white()):
            self.drive.run(self.drive.target_looking, 200)
        self.drive.brake()
        self.drive.run_cm(self.drive.target_looking, -WALL_TO_ROAD)
        self.drive.turn(-90)

        self.sensors.ground_reset()
        while (self.sensors.on_white()):
            self.drive.run(self.drive.target_looking, 300)
        
        self.drive.run_cm(self.drive.target_looking, -4)
        
        #actual aligning

        lr = self.sensors.front_left_sensor.color_sensor.reflection()
        rr = self.sensors.front_right_sensor.color_sensor.reflection()

        while not (lr <= 52 and lr >= 48 and rr <= 52 and rr >= 48):# and (watch.time() < 1000):
        
            lr = self.sensors.front_left_sensor.color_sensor.reflection()
            rr = self.sensors.front_right_sensor.color_sensor.reflection()

            lspd = lr - 50
            lspd *= 2
            self.drive.fl_motor.run(-lspd)
            self.drive.bl_motor.run(-lspd)
            
            rspd = rr - 50
            rspd *= 2

            self.drive.fr_motor.run(rspd)
            self.drive.br_motor.run(rspd)


        self.drive.hub.imu.reset_heading(0)
        self.drive.set_looking(0)

        # end of aligning

        self.drive.run_cm(self.drive.target_looking, -WALL_TO_ROAD)
        self.drive.hub.imu.reset_heading(0)
        self.drive.set_looking(0)
        self.drive.turn(90)

        self.drive.reset_odometry()

    def align_to_take_pipes(self, direction):
        lr = self.sensors.front_left_sensor.color_sensor.reflection()
        rr = self.sensors.front_right_sensor.color_sensor.reflection()
        speed = 150 * -direction
        
        lr = self.sensors.front_left_sensor.color_sensor.reflection()
        rr = self.sensors.front_right_sensor.color_sensor.reflection()

        lspd = lr - 15
        lspd *= 2

        self.drive.fl_motor.run(-lspd - speed)
        self.drive.bl_motor.run(-lspd + speed)
        
        rspd = rr - 15
        rspd *= 2

        self.drive.fr_motor.run(rspd - speed)
        self.drive.br_motor.run(rspd + speed)

        self.drive.odometry()