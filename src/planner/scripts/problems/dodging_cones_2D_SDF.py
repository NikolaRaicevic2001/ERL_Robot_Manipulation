import random

class Dodging_Cones_SDF:
    def __init__(self):

        self.window_size = 1000
        window_size = self.window_size
        self.start = (50, 50)
        self.goal = (950, 800)
        num_obstacles = 20
        
        # Define the problem environment with circle_SDF
        self.obstacles = []
        for i in range(num_obstacles):
            x_pos = random.randint(int(window_size/10),int(window_size/10*9))
            y_pos = random.randint(0,int(window_size))
            radius = random.randint(int(window_size/100),int(window_size/100*5))
            self.obstacles.append( ('circle_SDF', (x_pos,y_pos,radius)) )

        # Define the problem environment with circle
        for i in range(0):
            x_pos = random.randint(int(window_size/10),int(window_size/10*9))
            y_pos = random.randint(0,int(window_size))
            radius = random.randint(int(window_size/100),int(window_size/100*5))
            self.obstacles.append( ('circle', (x_pos,y_pos,radius)) )


    def get_environment(self):
        """ Function to Return Environment Details """
        return self.window_size,self.obstacles,self.start,self.goal
    
