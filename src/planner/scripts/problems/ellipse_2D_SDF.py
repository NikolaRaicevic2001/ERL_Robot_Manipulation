import random

class Dodging_Ellipse_SDF:
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
            a = random.randint(int(window_size/100),int(window_size/100*5))
            b = random.randint(int(window_size/100),int(window_size/100*5))
            self.obstacles.append( ('ellipse_SDF', (x_pos,y_pos,a,b)) )

    def get_environment(self):
        """ Function to Return Environment Details """
        return self.window_size,self.obstacles,self.start,self.goal
    
