from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch
from pybricks.parameters import Axis
import umath

from Map import Map
from PID import PID as pid
from constants import (BLE_CHANNEL_M2S, BLE_CHANNEL_S2M, F_R_PORT, F_R_POS, F_L_PORT, F_L_POS, B_R_PORT, B_R_POS, B_L_PORT, B_L_POS, MAX_SPEED)
from constants import (DRIVE_P, DRIVE_I, DRIVE_D, TURN_P, TURN_I, TURN_D, POS_P, POS_I, POS_D)
from constants import (WHEEL_RADIUS, K_SIZE)

class DriveSubsystem():
    def __init__(self, inputMap : Map) -> None:

        self.pid_x = pid(POS_P, POS_I, POS_D)
        self.pid_y = pid(POS_P, POS_I, POS_D)

        self.pid_drive = pid(DRIVE_P, DRIVE_I, DRIVE_D)

        self.pid_turn = pid(TURN_P, TURN_I, TURN_D)

        # Reference to each motor
        self.fr_motor = Motor(F_R_PORT)
        self.fl_motor = Motor(F_L_PORT)
        self.br_motor = Motor(B_R_PORT)
        self.bl_motor = Motor(B_L_PORT)

        # StopWatch for the time variable of PID
        self.stopwatch = StopWatch()

        # Reference to hub, needed to use the gyroscope
        self.hub = PrimeHub(broadcast_channel = BLE_CHANNEL_M2S, observe_channels = [BLE_CHANNEL_S2M])

        self.target_looking : float = 0

        self.wheels_pos = (0, 0, 0, 0)
        self.wheels_last = (0, 0, 0, 0)
        self.wheels_delta = (0, 0, 0, 0)

        # position in relation to start, organized in 3 variables: (x, y, r). 
        # x is the position in the x axis, in cm.
        # y is the position in the y axis, in cm.
        # r is the current rotation of the robot acording to the wheels, in degrees.
        self.global_pos = (0, 0)
        self.last_pos = (0, 0)
        
        self.desired_dir = 0

        self.map = inputMap

        self.reset_odometry()

        self.lombada = False

    def set_odometry(self, pos : tuple, head : int):
        self.global_pos = (pos[0], pos[1])
        self.last_pos = (pos[0], pos[1])
        self.set_looking(head)
        self.hub.imu.reset_heading(head)

    #resets the odometry, for when you need to restart the robot position or for when ending localization    
    def reset_odometry(self):
        self.set_odometry(tuple([0, 0]), 0)

    #odometry code, runs each time the robot run_raw is called
    def odometry(self):
        
        #sets the wheels positions in degrees
        self.wheels_pos = (
            -self.fl_motor.angle(),
            self.fr_motor.angle(),
            -self.bl_motor.angle(),
            self.fr_motor.angle()
        )
        
        # calculates the delta of the wheels, to use as velocity value in the odometry
        # instead of having to use time as a measure, as that can oscilate in unmeasurable 
        # ways, we use the delta, that oscilates by ticks, because it scales automatically 
        # with lag spikes and in a better way.

        # (delta = diference between the current wheels position and the 
        # last position recorded, can also be read as deg/tick, or DPT).
        
        self.wheels_delta  = (self.wheels_pos[0] - self.wheels_last[0],
                              self.wheels_pos[1] - self.wheels_last[1],
                              self.wheels_pos[2] - self.wheels_last[2],
                              self.wheels_pos[3] - self.wheels_last[3])

        # after calculating the delta, the wheels_last values are not needed, 
        # so it is already set to the current position for the next iteration.

        self.wheels_last = (self.wheels_pos[0],
                            self.wheels_pos[1],
                            self.wheels_pos[2],
                            self.wheels_pos[3])
        
        # sets four variables as the values of the corresponding wheel delta.
        # the names are very short so it is easier to write complex equations with.
        fl = (self.wheels_delta[0])
        fr = (self.wheels_delta[1])
        bl = (self.wheels_delta[2])
        br = (self.wheels_delta[3])

        # the motor functions work in degrees, but the formulas for 
        # mecanum odometry use radians, so we convert them here.
        fl = umath.radians(fl)
        fr = umath.radians(fr)
        bl = umath.radians(bl)
        br = umath.radians(br)

        # calculates the velocity in x and y, along with the angular velocity of the robot.
        # these are not our formulas, but we use them as they are quite standard, if needed, 
        # we will reference a source for them later.
        vx = (WHEEL_RADIUS/4) * (fl - fr - bl + br)
        vy = (WHEEL_RADIUS/4) * (fl + fr + bl + br)
        # vr = (WHEEL_RADIUS/4) * (-(fl/K_SIZE) + (fr/K_SIZE) - (bl/K_SIZE) + (br/K_SIZE))

        # the formula also returns rotation in radians, so we convert them 
        # back to degrees for better understanding of the tiny human brain.
        # vr = umath.degrees(vr)

        self.last_pos = (self.global_pos[0], self.global_pos[1])

        move_angle = umath.degrees(umath.atan2(vx, vy))
        # print(self.desired_dir)
        correction_angle = self.desired_dir - move_angle
        
        corrected_vel : tuple = (
                                    (vx * umath.cos(umath.radians(correction_angle))) + (vy * umath.sin(umath.radians(correction_angle))),
                                    (vy * umath.cos(umath.radians(correction_angle))) - (vx * umath.sin(umath.radians(correction_angle)))
        )

        self.global_pos = (self.last_pos[0] + corrected_vel[0], self.last_pos[1] + corrected_vel[1]) # , self.hub.imu.heading()

        #----------------------------------------------------------------------------
        # ramp correction
        rotationx = self.hub.imu.rotation(Axis.X)
        rotationy = self.hub.imu.rotation(Axis.Y)
        if(rotationx > 15 or rotationx < -15 or rotationy > 15 or rotationy < -15):
            if(self.lombada == False):  
                self.lombada = True
                self.global_pos = (self.global_pos[0] -9, self.global_pos[1] -9)
        else:
            self.lombada = False
        #----------------------------------------------------------------------------

    def set_looking(self, new_looking : float):
        # self.hub.imu.reset_heading(self.hub.imu.heading() - ((new_looking - self.target_looking)/50))
        self.target_looking = new_looking

    def run_pos(self, x, y):
        
        # get the current position from the robot
        pos = self.global_pos

        # the pid on run_pos is divided between x and y coordinates.
        # It uses the current robot position as reference and defines the desired x position as target

        #calculate the pid for x velocity
        self.pid_x.SetPoint = x # define the target
        self.pid_x.update(pos[0]) # update current position
        vx = self.pid_x.output # get the calculated value

        #calculate the pid for y velocity
        self.pid_y.SetPoint = y # define the target
        self.pid_y.update(pos[1]) # update current position
        vy = self.pid_y.output # get the calculated value
        
        # rx and ry are the pure remaining distance in their respective coordinates, used to calculate the angle of movement
        rx = x - pos[0] 
        ry = y - pos[1]


        direction = umath.degrees(umath.atan2(rx, ry)) # calculate the movement direction

        speed = umath.sqrt(umath.pow(vx, 2) + umath.pow(vy, 2)) # calculate the speed

        turn = (self.target_looking - self.hub.imu.heading()) * 14

        maxTurn = 300


        if (turn < -maxTurn):
            turn = -maxTurn

        if (turn > maxTurn):
            turn = maxTurn

        maxDrive = MAX_SPEED - abs(turn)

        if (speed < -maxDrive):
            speed = -maxDrive

        if (speed > maxDrive):
            speed = maxDrive 

        self.desired_dir = direction
        self.run_raw(direction - self.hub.imu.heading(), speed, turn)

    def run_raw(self, direction : float, speed : float, rotation):

        fr = umath.sin(umath.radians(F_R_POS - direction)) * speed
        fl = umath.sin(umath.radians(F_L_POS - direction)) * speed
        br = umath.sin(umath.radians(B_R_POS - direction)) * speed
        bl = umath.sin(umath.radians(B_L_POS - direction)) * speed

        self.fr_motor.run(fr - rotation)
        self.fl_motor.run(fl - rotation)
        self.br_motor.run(br - rotation)
        self.bl_motor.run(bl - rotation)

        
        self.odometry()

    def run(self, direction, speed):

        turn = (self.target_looking - self.hub.imu.heading()) * 14

        maxTurn = 400

        if (turn < -maxTurn):
            turn = -maxTurn

        if (turn > maxTurn):
            turn = maxTurn

        maxDrive = MAX_SPEED - abs(turn)

        if (speed < -maxDrive):
            speed = -maxDrive

        if (speed > maxDrive):
            speed = maxDrive 

        final_dir = direction - self.hub.imu.heading()

        self.desired_dir = final_dir

        self.run_raw(final_dir, speed, turn)

    def run_cm(self, direction, cm, stop_conditions=None):
        if(stop_conditions is None):
            stop_conditions = []

        pid_drive = pid(DRIVE_P, DRIVE_I, DRIVE_D)

        pid_drive.setKp(DRIVE_P)
        pid_drive.setKi(DRIVE_I)
        pid_drive.setKd(DRIVE_D)
        
        pid_drive.SetPoint = cm

        margin = 0.1
        current_pos = 0

        starting_pos = self.global_pos

        while not (current_pos > (cm - margin) and current_pos < (cm + margin)):
            for condition in stop_conditions:
                if condition():
                    self.brake()
                    return

            relative_pos = (
                self.global_pos[0] - starting_pos[0],
                self.global_pos[1] - starting_pos[1],
            )

            pid_drive.SetPoint = cm

            current_pos = umath.copysign(umath.sqrt(umath.pow(relative_pos[0], 2) + umath.pow(relative_pos[1], 2)), cm)
            pid_drive.update(current_pos)
            speed = pid_drive.output

            self.run(direction, speed)
        # self.turn(0)

    def turn(self, rotation):
        self.set_looking(self.target_looking + rotation)

        margin = 0.5
        gyro = self.hub.imu.heading()

        while (gyro < self.target_looking - margin or gyro > self.target_looking + margin):

            gyro = self.hub.imu.heading()
        
            self.run(0, 0)

        # self.hub.imu.reset_heading(self.target_looking)
        self.brake()

    def brake(self):
        #Stop all motors
        self.fr_motor.brake()
        self.fl_motor.brake()
        self.br_motor.brake()
        self.bl_motor.brake()

        self.stopwatch.pause()
        self.stopwatch.reset()
        self.stopwatch.resume()
        while (self.stopwatch.time() < 300):
            pass
        return
    
    def follow_path(self, path : tuple, end_rot : int):
        ended_path = False
        end_place = path[-1]
        end_III = tuple([end_place[0]*30, end_place[1]*30])
        
        obstacles = self.map.get_obstacles()

        clearence = 0
        end_clearence = 0

        rots : tuple = ()
        
        #assemble rots list
        for index in range(0, len(path)):
            
            if(index == (len(path) - 1)):
                rots += tuple([end_rot])
                break
            else:
                angle = umath.degrees(umath.atan2(path[index+1][0] - path[index][0], path[index+1][1] - path[index][1]))

            angle -= umath.copysign(180, angle)

            rots += tuple([angle])
        


        target_rot = rots[1]

        target : tuple = [path[1][0], path[1][1]]

        current_rot = rots[0]

        #start of loop

        while ended_path == False:

            #current robot position in cm
            pos = tuple((self.global_pos[0], self.global_pos[1]))
            
            #imu

            #position, but on whole cm
            int_pos = tuple([round(pos[0]), round(pos[1])])

            #position, in blocks (1 block = 30 cm)
            int_place = tuple([round(int_pos[0]/30), round(int_pos[1]/30)])

            #data from hub ble connection
            data = self.hub.ble.observe(BLE_CHANNEL_S2M)

            # if data is none and you are seeing obstacles, get the place where the obstacle is, check
            # if it is already on the map, if yes, do nothing, if not, add it and break the loop
            if (data != None):
                if (data[1] == True):
                    # target = int_place

                    next_place = (round(umath.sin(umath.radians(self.hub.imu.heading() - 180))), round(umath.cos(umath.radians(self.hub.imu.heading() - 180))))
                    next_place = (next_place[0] + int_place[0], next_place[1] + int_place[1])
                    on_frame = not (next_place[0] > 4 or next_place[0] < 0 or next_place[1] > 4 or next_place[1] < 0)
                    hasobstacle = obstacles.count(next_place) > 0

                    if (not hasobstacle and on_frame):
                        self.brake()
                        self.map.set_obstacle(next_place[0], next_place[1])
                        ended_path = False
                        break

            # check for each place in the path            
        
            place = (target[0]*30, target[1]*30)

            if (current_rot == target_rot):
                if(int_place[0] == target[0] and int_place[1] == target[1]):

                    if (target[0] == end_place[0] and target[1] == end_place[1]):
                        self.set_looking(end_rot)
                    else:
                        target = path[path.index(int_place) + 1]
                        target_rot = rots[path.index(int_place) + 1]
                        # self.set_looking(target_rot)
            
            elif(int_place[0] == target[0] and int_place[1] == target[1]):
                self.set_looking(target_rot)

            #if on end, break
            if (int_pos[0] <= end_III[0] + end_clearence and int_pos[1] <= end_III[1] + end_clearence and int_pos[0] >= end_III[0] -end_clearence and int_pos[1] >= end_III[1] -end_clearence):
                target = end_place
                target_rot = end_rot
                self.set_looking(target_rot)

                if (round(self.hub.imu.heading()) == end_rot):
                    ended_path = True

            # else if on the place, set the target as the next in the list
            elif (int_pos[0] <= place[0] + clearence and int_pos[1] <= place[1] + clearence and int_pos[0] >= place[0] -clearence and int_pos[1] >= place[1] -clearence):
                i = path.index(int_place)
                if (i+1 < (len(path))):
                    target = path[i+1]
                    target_rot = rots[i+1]
                    current_rot = rots[i]
                    if (round(self.hub.imu.heading()) != current_rot):
                        self.turn(0)
                    # self.set_looking(target_rot)

                # else:
                #     self.set_looking(end_rot)

            self.run_pos(target[0]*30, target[1]*30)

        return ended_path