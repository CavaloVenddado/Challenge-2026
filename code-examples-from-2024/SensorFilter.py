from pybricks.pupdevices import ColorSensor
from pybricks.parameters import Color
from pybricks.parameters import Port

class SensorFilter():
    def __init__(self, port: Port, colors : tuple, intervals : tuple) -> None:
        self.sensor_port = port
        self.colors_to_filter = colors
        self.color_intervals = intervals
        self.color_sensor = ColorSensor(port)
        self.current_color:Color = Color.WHITE

    def hsv(self):
        return self.color_sensor.hsv()

    #Reset the color to white
    def reset(self):
        self.current_color = Color.WHITE

    #filter the color based on the h, s and v intervals given
    def filtered_color(self) -> Color:
        input = self.hsv()
        for x in range(0, len(self.colors_to_filter)):
            if(input.h >= self.color_intervals[x][0] and input.h <= self.color_intervals[x][1] and input.s >= self.color_intervals[x][2] and input.s <= self.color_intervals[x][3] and input.v >= self.color_intervals[x][4] and input.v <= self.color_intervals[x][5]):
                self.current_color = self.colors_to_filter[x]
        # print((self.current_color, self.hsv()))
        return self.current_color