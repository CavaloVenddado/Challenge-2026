from pybricks.parameters import Color
from pybricks.hubs import PrimeHub
from constants import LEFT_COLOR_SENSOR, RIGHT_COLOR_SENSOR
from SensorFilter import SensorFilter
from DriveSubsystem import DriveSubsystem

class SensorSubsystem():
    
    def __init__(self, inputDrive:DriveSubsystem) -> None:
        self.drive = inputDrive
        self.ground_colors = ( Color.WHITE, Color.RED, Color.BLUE, Color.BLACK, Color.YELLOW)
        
        self.ground_color_intervals = (
                                       [0, 360, 0, 100, 90, 100],
                                       [241, 360, 0, 100, 50, 100], 
                                       [200, 240, 0, 100, 0, 89], 
                                       [0, 360, 0, 40, 0, 80], 
                                       [20, 90, 0, 100, 0, 100]
                                      )
                
        self.front_left_sensor = SensorFilter(LEFT_COLOR_SENSOR, self.ground_colors, self.ground_color_intervals)
        self.front_right_sensor = SensorFilter(RIGHT_COLOR_SENSOR, self.ground_colors, self.ground_color_intervals)

    def ground_reset(self) -> bool:
        pass
        self.front_left_sensor.reset()
        self.front_right_sensor.reset()
        
    #-----------------------------------------------------------------------------------------

    def at_color_left(self):
        return self.left_filter_alt()
    def at_color_right(self):
        return self.right_filter_alt()

    def on_white(self) -> bool:
        if (self.at_color_left() == Color.WHITE and self.at_color_right() == Color.WHITE):
            return True
    
    def on_red(self) -> bool:
        if (self.at_color_left() == Color.RED and self.at_color_right() == Color.RED):
            return True
    
    def on_blue(self) -> bool:
        if (self.at_color_left() == Color.BLUE and self.at_color_right() == Color.BLUE):
            return True
    
    def on_wall(self)-> bool:
        if ((self.at_color_right() == Color.BLACK or self.at_color_right() == Color.YELLOW) and (self.at_color_left() == Color.YELLOW or self.at_color_left() == Color.BLACK)):
            return True
        
    #-----------------------------------------------------------------------------------------

    def left_on_white(self) -> bool:
        if (self.at_color_left() == Color.WHITE):
            return True
    
    def left_on_red(self) -> bool:
        if (self.at_color_left() == Color.RED):
            return True
    
    def left_on_blue(self) -> bool:
        if (self.at_color_left() == Color.BLUE):
            return True
    
    def left_on_wall(self)-> bool:
        if (self.at_color_left() == Color.BLACK or self.at_color_left() == Color.YELLOW):
            return True
        
    #-----------------------------------------------------------------------------------------

    def right_on_white(self) -> bool:
        if (self.at_color_right() == Color.WHITE):
            return True
    
    def right_on_red(self) -> bool:
        if (self.at_color_right() == Color.RED):
            return True
    
    def right_on_blue(self) -> bool:
        if (self.at_color_right() == Color.BLUE):
            return True
    
    def right_on_wall(self)-> bool:
        if (self.at_color_right() == Color.BLACK or self.at_color_right() == Color.YELLOW):
            return True
        
    # -----------------------------------------------------------------------------------------

    def left_filter_alt(self):
        hsvL = self.front_left_sensor.hsv()
        h, s, v = hsvL.h, hsvL.s, hsvL.v

        if (s < 40) and (v > 80):
            # print("white")
            return Color.WHITE
        if (s < 40) and (v < 40):
            # print("black")
            return Color.BLACK
        if (h < 25) or (h > 310):
            # print("red")
            return Color.RED
        if (h > 140) and (h < 300) and (s > 30):
            # print("blue")
            return Color.BLUE
        if (h > 30) and (h < 100):
            # print("yellow")
            return Color.YELLOW

        return Color.WHITE
    
    def right_filter_alt(self):
        hsvL = self.front_right_sensor.hsv()
        h, s, v = hsvL.h, hsvL.s, hsvL.v

        if (s < 40) and (v > 80):
            # print("white")
            return Color.WHITE
        if (s < 40) and (v < 40):
            # print("black")
            return Color.BLACK
        if (h < 25) or (h > 310):
            # print("red")
            return Color.RED
        if (h > 140) and (h < 300):
            # print("blue")
            return Color.BLUE
        if (h > 30) and (h < 100):
            # print("yellow")
            return Color.YELLOW

        return Color.WHITE

    # def align_on_color(self):
        
    #         self.drive.run_cm(self.drive.target_looking, -5)
    #         while(self.at_color_left() == Color.WHITE or self.at_color_right() == Color.WHITE):
    #             if(self.right_on_white()):
    #                 self.drive.br_motor.run(150)
    #                 self.drive.fr_motor.run(150)
    #             else:
    #                 self.drive.br_motor.run(-150)
    #                 self.drive.fr_motor.run(-150)

    #             if(self.left_on_white()):
    #                 self.drive.bl_motor.run(-150)
    #                 self.drive.bl_motor.run(-150)
    #             else:
    #                 self.drive.bl_motor.run(150)
    #                 self.drive.bl_motor.run(150)
                
    #         self.drive.brake()