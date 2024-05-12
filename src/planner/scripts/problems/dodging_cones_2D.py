import random

from problems import visualizer_2D as viz2D

class Dodging_Cones:
    def __init__(self):

        self.window_size = 1000
        window_size = self.window_size
        self.start = (50, 50)
        self.goal = (950, 800)
        num_obstacles = 100

        # Define the problem environment with different obstacle types
        self.obstacles = []
        for i in range(num_obstacles):
            x_pos = random.randint(int(window_size/10),int(window_size/10*9))
            y_pos = random.randint(0,int(window_size))
            radius = random.randint(int(window_size/100),int(window_size/100*5))
            self.obstacles.append( ('circle', (x_pos,y_pos,radius)) )

    def get_environment(self):
        """ Function to Return Environment Details """
        return self.window_size,self.obstacles,self.start,self.goal