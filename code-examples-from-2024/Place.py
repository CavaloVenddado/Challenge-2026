from Buildings import MUSEUM, LIBRARY, BAKERY, DRUGSTORE, SCHOOL, CITYHALL, PARK

class Place():
    def __init__(self) -> None:
        self.obstacle = False
        self.entrance : tuple = (())
        self.occupied = False

    def set_obstacle(self, input:bool):
        self.obstacle = input

    def set_entrance(self, input:tuple):
        self.entrance += tuple([input])

    def set_occupied(self, input:bool):
        self.occupied = input