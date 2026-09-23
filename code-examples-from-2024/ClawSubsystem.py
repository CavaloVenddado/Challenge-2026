from pybricks.pupdevices import Motor, UltrasonicSensor
from pybricks.parameters import Color
from constants import (CLAW_PORT, CLAW_PORT_2, MAX_SPEED, MILK_ULTRASONIC_SENSOR, PERSON_COLOR_SENSOR, PERSON_ULTRASONIC_DETECT, PERSON_ULTRASONIC_SENSOR)
from Buildings import MUSEUM, LIBRARY, BAKERY, DRUGSTORE, SCHOOL, CITYHALL, PARK
from SensorFilter import SensorFilter

class ClawSubsystem():

    def __init__(self) -> None:
        self.claw_motor = Motor(CLAW_PORT)
        self.claw_motor_2 = Motor(CLAW_PORT_2)
        self.ultrasonic = UltrasonicSensor(PERSON_ULTRASONIC_SENSOR)
        self.milk_sensor = UltrasonicSensor(MILK_ULTRASONIC_SENSOR)
        self.ultrasonic_2 = UltrasonicSensor(PERSON_ULTRASONIC_DETECT)
        
        self.close_colors = (Color.WHITE, Color.RED, Color.BLUE, Color.GREEN, Color.BROWN)
        self.close_colors_intervals = ([0, 360, 0, 100, 0, 100],
                                       [330, 360, 65, 100, 25, 70],
                                       [200, 220, 70, 90, 42, 77],
                                       [150, 160, 70, 90, 13, 50],
                                       [0, 25, 20, 65, 10, 50],
        )
        # self.person_sensor = SensorFilter(PERSON_COLOR_SENSOR2, self.person_colors, self.person_color_intervals)

        self.close_sensor = SensorFilter(PERSON_COLOR_SENSOR, self.close_colors, self.close_colors_intervals)
        self.place = -1
        
    #move the claw motor in a certain direction with a certain speed
    def move_claw(self, direction, speed):
        self.claw_motor.run((speed) * direction)
        self.claw_motor_2.run((speed) * -direction)

    def closeClaw(self):
        self.move_claw(-1, 1000)

    def openClaw(self):
        self.move_claw(1, MAX_SPEED/4)

    def stopClaw(self):
        self.claw_motor.brake()
        self.claw_motor_2.brake()

    def get_person(self):
        # print(self.ultrasonic_2.distance())
        if(self.ultrasonic_2.distance() < 80): # for LEGO sensor
        # if(self.ultrasonic_2.distance() < 100): # for pirated sensor
            return 1
        else:
            return -1
        
    def get_close_color(self):
        return(self.close_sensor.filtered_color())
    
    #get the height of the pipe
    def person_tall(self) -> bool:
        if(self.ultrasonic.distance() < 150):
            return True
        else:
            return False
    
    #checks if it sees anything with the pipe ultrasonic sensor
    def detect_person(self):
        if(self.get_person() == True):
            self.place = -1
        else:
            self.place = 1

    def detect_close(self):
        if(self.get_close_color() == Color.NONE or self.get_close_color() == Color.WHITE):
            self.place = -1
        else:
            self.identify_close()

    
    def identify_close(self):
        self.place = -1
        if(self.person_tall()):
            if(self.get_close_color() == Color.BLUE):
                self.place = MUSEUM
            if(self.get_close_color() == Color.RED):
                self.place = DRUGSTORE
            if(self.get_close_color() == Color.BROWN):
                self.place = BAKERY
            if(self.get_close_color() == Color.GREEN):
                self.place = CITYHALL
        else:
            if(self.get_close_color() == Color.BLUE):
                self.place = SCHOOL
            if(self.get_close_color() == Color.RED):
                self.place = DRUGSTORE
            if(self.get_close_color() == Color.BROWN):
                self.place = LIBRARY
            if(self.get_close_color() == Color.GREEN):
                self.place = PARK
        self.close_sensor.reset()
        return(self.place)