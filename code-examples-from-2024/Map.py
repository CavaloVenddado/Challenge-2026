from Place import Place

class Map():
    #Arena map in a 5x5 matrix
    def __init__(self) -> None:
        self.map = ((Place(), Place(), Place(), Place(), Place()),
                    (Place(), Place(), Place(), Place(), Place()),
                    (Place(), Place(), Place(), Place(), Place()),
                    (Place(), Place(), Place(), Place(), Place()),
                    (Place(), Place(), Place(), Place(), Place()))
    
    def __call__(self) -> tuple:
        return self.map

    #set the entrances of the buildings on the place with the angle the robot need to be
    def set_entrances(self, input:tuple):
        for i in range(0, len(input)):
            self.map[input[i][0]][input[i][1]].set_entrance((input[i][2], input[i][3]))

    #get all the entrances
    def get_entrances(self):
        entrance:tuple = ()
        for x in range(0, 5):
            for y in range(0, 5):
                for z in range(0, len(self.map[x][y].entrance)):
                    entrance += tuple([(x,y, self.map[x][y].entrance[z][1])])
        return entrance

    #get the entrances of a building
    def get_specified_entrance(self, place):
        entrance:tuple = ()
        for x in range(0, 5):
            for y in range(0, 5):
                for z in range(0, len(self.map[x][y].entrance)):
                    if (self.map[x][y].entrance[z][0] == place):
                        entrance += tuple([(x,y, self.map[x][y].entrance[z][1])])
        return entrance
    
    #set as occupied = there is one pipe there
    def occupy(self, x, y):
        self.map[x][y].set_occupied(True)

    #check if a place in occupied
    def get_occupied(self, x,y):
        return(self.map[x][y].occupied)

    #set a obstacle(s) in a place
    def set_obstacles(self, new_obstacles:tuple): #for multiple obstacle at once
        for i in range(0, len(new_obstacles)):
            self.map[new_obstacles[i][0]][new_obstacles[i][1]].set_obstacle(True)

    def set_obstacle(self, x, y):   #for just one obstacle at a time
        self.map[x][y].set_obstacle(True)
        if (self.map[3][3].obstacle == True and self.map[4][2].obstacle == True):
            self.map[4][4].set_obstacle(True)

    #get the place of all the obstacles
    def get_obstacles(self) -> tuple:
        obstacles : tuple = ()
        for x in range(0, 5):
            for y in range(0, 5):
                if (self.map[x][y].obstacle == True):
                    obstacles += tuple([(x, y)])
        return obstacles

    #get the squares that are arond a certain place
    def get_adjacent(self, place:tuple):
        squares:tuple = ()
        if (place[0] != 0):
            squares += tuple([(place[0] - 1, place[1])])
        if (place[0] != 4):
            squares += tuple([(place[0] + 1, place[1])])

        if (place[1] != 0):
            squares += tuple([(place[0], place[1] - 1)])
        if (place[1] != 4):
            squares += tuple([(place[0], place[1] + 1)])
        return squares
    
    #removes the obstacles
    def remove_obstacles(self, places:tuple):
        obstacles:tuple = self.get_obstacles()
        return self.remove_blocks(places, obstacles)

    def remove_blocks(self, normal:tuple, blocks:tuple):
        new_list:tuple = ()
        for j in range(0, len(normal)):
            put = False
            for i in range(0, len(blocks)):
                if (blocks[i][0] == normal[j][0] and blocks[i][1] == normal[j][1]):
                    put = False
                    break
                else:
                    put = True
            if (put == True):
                new_list += tuple([(normal[j])])

        return new_list

    #create a path based on the start and end location, taking in account the obstacles
    def make_path(self, start:tuple, end:tuple) -> tuple:
        complete_path : tuple = ()
        paths : tuple = (([[start]]))
        
        has_path = False

        while not(has_path):
            new_paths:tuple = ()
            # for cur_path in paths:
            #     print(cur_path)
            for p in range(0, len(paths)):
                
                path : tuple = tuple(paths[p])
                
                next_blocks = self.get_adjacent(path[len(path) -1])
                next_blocks = self.remove_obstacles(next_blocks)
                next_blocks = self.remove_blocks(next_blocks, tuple([path[len(path) -2]]))
                for b in range(0, len(next_blocks)):
                    block = next_blocks[b]
                    # print(block)
                    new_path:tuple = tuple(path)
                    new_path += (tuple([block]))
                    # print(new_path)
                    new_paths += tuple(([new_path]))

                    if (block == end):
                        complete_path = new_path
                        has_path = True
                        break

            paths = new_paths
        return complete_path
