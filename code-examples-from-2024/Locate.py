from pybricks.tools import StopWatch
from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis
import umath
from constants import (MAX_SPEED, WALL_TO_ROAD, BLE_CHANNEL_M2S, BLE_CHANNEL_S2M)
from SensorSubsystem import SensorSubsystem as Sensors
from DriveSubsystem import DriveSubsystem as Drive
from Map import Map

class Locate():
    
    def __init__(self, inputHub : PrimeHub, inputDrive: Drive, inputSensors : Sensors, inputMap : Map) -> None:
        
        self.drive = inputDrive
        self.sensors = inputSensors
        self.map = inputMap

        self.stopwatch = StopWatch()

        self.seen_wall = False
        self.seen_red = False
        self.seen_blue = False
        self.seen_obstacle = False
        self.on_place = False

        self.hub = inputHub
        self.hub = PrimeHub(broadcast_channel = BLE_CHANNEL_M2S, observe_channels = [BLE_CHANNEL_S2M])

        self.seen_obstacle = False

        A = 100

        self.icon = ((0, A, A, A, 0),
                     (0, A, 0, A, 0),
                     (0, A, A, A, 0),
                     (0, 0, A, 0, 0),
                     (0, 0, 0, 0, 0))

    def check_obstacle(self):
        data = self.hub.ble.observe(BLE_CHANNEL_S2M)

        if(data != None and self.pipe_dist == True):
            if(data[4] == True):
                self.drive.brake()
                return(True)
            else:
                return False

    def search(self):
        self.stopwatch.resume()
        running = True
        while (running):
            
            print(self.sensors.left_filter_alt())
            self.drive.run(self.drive.target_looking, MAX_SPEED * 0.65)
            data = self.hub.ble.observe(BLE_CHANNEL_S2M)
            if (self.stopwatch.time() > 450):
                if (self.pipe_dist == True):
                    running = (self.sensors.on_white()) and (data != None and data[4] == False) # or data[5] < 15 or data[5] < -15)

                else:
                    running = self.sensors.on_white() # and (data != None and self.pipe_dist == True and data[4] == False)

        self.drive.run_cm(self.drive.target_looking, 1)
        self.drive.turn(0)


        if (data != None and self.pipe_dist == True and data[4] == True and not self.sensors.on_blue()): # if seen an object

            self.drive.run_cm(self.drive.target_looking, -7 -WALL_TO_ROAD)
            if (self.seen_obstacle or self.seen_red or self.seen_wall): # if already seen anything
                obj_left = False
                obj_right = False

                innitial_rot = self.drive.target_looking
                
                #check left
                self.drive.turn(-90)
                data = self.hub.ble.observe(BLE_CHANNEL_S2M) # check for obstacles on left
                if (data != None and self.pipe_dist == True and data[2] == True): # if seen another obstacle (now on left)
                    obj_left = True

                    self.drive.turn(180)
                
                    data = self.hub.ble.observe(BLE_CHANNEL_S2M) # check for obstacles on right
                    if (data != None and self.pipe_dist == True and data[2] == True):
                        obj_right = True

                if(obj_left and obj_right):
                    self.drive.turn(innitial_rot - self.drive.target_looking)
                    self.drive.run_cm(self.drive.target_looking, -60)
                    
                    self.drive.turn(-90)
                    
                    completed_left = self.go_left()
                    if (completed_left):
                        return
                    
                    else:
                        
                        self.drive.turn(90)
                        completed_right = self.go_right()
                        if (completed_right):
                            return
                    
                else:

                    completed_right = False
                    completed_left = False


                    if not (obj_left):
                        self.drive.turn(innitial_rot - self.drive.target_looking - 90)
                        
                        completed_left = self.go_left()
                        if(completed_left):
                            return
                        else:
                            self.drive.turn(90)
                
                            data = self.hub.ble.observe(BLE_CHANNEL_S2M) # check for obstacles on right
                            
                            if (data != None and self.pipe_dist == True and data[2] == True):
                                obj_right = True
    
                    if not (obj_right):
                        self.drive.turn(innitial_rot - self.drive.target_looking + 90)
                        completed_right = self.go_right()

                        if(completed_right):
                            return
                    else:
                        self.drive.turn(-90)
                    
                    
                    # self.drive.turn(innitial_rot - self.drive.target_looking)
                    self.drive.run_cm(self.drive.target_looking, -60)

                    self.drive.turn(innitial_rot - self.drive.target_looking - 90)
                    
                    if (data != None and self.pipe_dist == True and data[2] == True):
                        
                        self.drive.turn(180)
                        
                    else:
                        completed_left = self.go_left()
                        if(completed_left):
                            return
                        
                        else:
                            self.drive.turn(90)
                            
                    completed_right = self.go_right()
                    if(completed_right):
                        return
                    
                        
            else:
                self.drive.turn(-90)
                self.seen_obstacle = True
                return

        
        if (self.sensors.on_red() == True):
            if (self.seen_blue == True):
                
                self.final_align()

                self.on_place = True
                # self.drive.run_cm(self.drive.target_looking, -WALL_TO_ROAD)
                # self.drive.turn(90)

                return
            else:
                self.seen_red = True
                dir = self.drive.target_looking
                self.drive.run_cm(dir, -30 -WALL_TO_ROAD)
                self.drive.set_looking(self.drive.target_looking -90)
                return
        
        if (self.sensors.on_wall() == True and (self.seen_red == True or self.seen_wall == True or self.seen_obstacle == True)):
            self.drive.set_looking(self.drive.target_looking + 180)
            self.seen_wall = False
            return
        
        if (self.sensors.on_blue() == True):
            
            # self.drive.brake()
            self.seen_blue = True
            dir = self.drive.target_looking
            self.drive.set_looking(self.drive.target_looking - 90)
            self.drive.run_cm(dir, -WALL_TO_ROAD)            
            
            return

        if (self.sensors.on_wall() == True):
            self.seen_wall = True
            dir = self.drive.target_looking
            # self.drive.set_looking(self.drive.target_looking - 90)
            self.drive.run_cm(dir, -WALL_TO_ROAD) 
            
            self.drive.turn(-90)
            self.seen_obstacle = False
            return

#--------------------------------------------------------------------------------

    def localizeSelf(self, pipe_dist_input:bool) -> bool:
        self.pipe_dist = pipe_dist_input
        self.seen_wall = False
        self.seen_red = False
        self.seen_blue = False
        self.seen_obstacle = False
        self.on_place = False


        self.hub.display.icon(self.icon)

        while(self.on_place == False):
            self.stopwatch.pause()
            self.stopwatch.reset()
            self.search()
            self.sensors.ground_reset()
        self.drive.reset_odometry()


    def go_left(self) -> bool:
        completed = False
        self.drive.run_cm(self.drive.target_looking, 60, (self.sensors.on_wall, self.sensors.on_red))
        if not (self.sensors.on_white()):
            if (self.sensors.on_wall()):
                self.drive.run_cm(self.drive.target_looking, -WALL_TO_ROAD)
            elif (self.sensors.on_red()):
                self.drive.run_cm(self.drive.target_looking, -WALL_TO_ROAD -30)
            
            completed = False
        else:
            completed = True
        self.drive.turn(90)
        return completed

    def go_right(self) -> bool:

        completed = False
        self.drive.run_cm(self.drive.target_looking, 60, (self.sensors.on_wall, self.sensors.on_red))
        if not (self.sensors.on_white()):
            if (self.sensors.on_wall()):
                self.drive.run_cm(self.drive.target_looking, -WALL_TO_ROAD)
            elif (self.sensors.on_red()):
                self.drive.run_cm(self.drive.target_looking, -WALL_TO_ROAD -30)
            
            completed = False
        else:
            completed = True
        self.drive.turn(-90)
        return completed
    
    def final_align(self):
        self.drive.run_cm(self.drive.target_looking, -5)
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
