import random

class Dodging_Cones_SDF:
    def __init__(self):

        self.window_size = 1000
        window_size = self.window_size
        self.start = (50, 50)
        self.goal = (950, 800)
        num_obstacles = 20
        
        # # Define the problem environment with circle_SDF
        # self.obstacles = []
        # for i in range(num_obstacles):
        #     x_pos = random.randint(int(window_size/10),int(window_size/10*9))
        #     y_pos = random.randint(0,int(window_size))
        #     radius = random.randint(int(window_size/100),int(window_size/100*5))
        #     self.obstacles.append( ('circle_SDF', (x_pos,y_pos,radius)) )

        # # Define the problem environment with circle
        # for i in range(0):
        #     x_pos = random.randint(int(window_size/10),int(window_size/10*9))
        #     y_pos = random.randint(0,int(window_size))
        #     radius = random.randint(int(window_size/100),int(window_size/100*5))
        #     self.obstacles.append( ('circle', (x_pos,y_pos,radius)) )

        # Predefined Obstacles
        self.obstacles = [
            ('circle_SDF', (int(self.window_size / 10 * 2), int(self.window_size / 10 * 2), int(self.window_size / 200 * 10))),    # Obstacle 1
            ('circle_SDF', (int(self.window_size / 10 * 5), int(self.window_size / 10 * 2), int(self.window_size / 200 * 15))),    # Obstacle 2
            ('circle_SDF', (int(self.window_size / 10 * 8), int(self.window_size / 10 * 2), int(self.window_size / 200 * 20))),    # Obstacle 3
            ('circle_SDF', (int(self.window_size / 10 * 3), int(self.window_size / 10 * 5), int(self.window_size / 200 * 10))),    # Obstacle 4
            ('circle_SDF', (int(self.window_size / 10 * 6), int(self.window_size / 10 * 8), int(self.window_size / 200 * 30))),    # Obstacle 5
            ('circle_SDF', (int(self.window_size / 10 * 3), int(self.window_size / 10 * 2.5), int(self.window_size / 200 * 10))),  # Obstacle 6
            ('circle_SDF', (int(self.window_size / 10 * 7), int(self.window_size / 10 * 5.5), int(self.window_size / 200 * 15))),  # Obstacle 7
            ('circle_SDF', (int(self.window_size / 10 * 3.5), int(self.window_size / 10 * 3.2), int(self.window_size / 200 * 20))),# Obstacle 8
            ('circle_SDF', (int(self.window_size / 10 * 4), int(self.window_size / 10 * 7.7), int(self.window_size / 200 * 10))),  # Obstacle 9
            ('circle_SDF', (int(self.window_size / 10 * 7), int(self.window_size / 10 * 5.9), int(self.window_size / 200 * 30))),  # Obstacle 10
        ]


    def get_environment(self):
        """ Function to Return Environment Details """
        return self.window_size,self.obstacles,self.start,self.goal
    
