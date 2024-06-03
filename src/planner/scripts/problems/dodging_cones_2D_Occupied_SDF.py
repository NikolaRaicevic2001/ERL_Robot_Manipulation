import numpy as np
from scipy import ndimage
import random
import matplotlib.pyplot as plt

class Dodging_Cones_Occupied_SDF:
    def __init__(self):
        self.window_size = 1000
        self.start = (50, 50)
        self.goal = (950, 800)
        num_obstacles = 20
        self.SDF = []
        self.obstacles = []
        for i in range(num_obstacles):
            x_pos = random.randint(int(self.window_size/10), int(self.window_size/10*9))
            y_pos = random.randint(0, int(self.window_size))
            radius = random.randint(int(self.window_size/100), int(self.window_size/100*5))
            self.obstacles.append(('circle_Occupied', (x_pos, y_pos, radius)))

        self.obstacles = [
            ('circle_Occupied', (int(self.window_size / 10 * 2), int(self.window_size / 10 * 2), int(self.window_size / 200 * 10))),    # Obstacle 1
            ('circle_Occupied', (int(self.window_size / 10 * 5), int(self.window_size / 10 * 2), int(self.window_size / 200 * 15))),    # Obstacle 2
            ('circle_Occupied', (int(self.window_size / 10 * 8), int(self.window_size / 10 * 2), int(self.window_size / 200 * 20))),    # Obstacle 3
            ('circle_Occupied', (int(self.window_size / 10 * 3), int(self.window_size / 10 * 5), int(self.window_size / 200 * 10))),    # Obstacle 4
            ('circle_Occupied', (int(self.window_size / 10 * 6), int(self.window_size / 10 * 8), int(self.window_size / 200 * 30))),    # Obstacle 5
            ('circle_Occupied', (int(self.window_size / 10 * 3), int(self.window_size / 10 * 2.5), int(self.window_size / 200 * 10))),  # Obstacle 6
            ('circle_Occupied', (int(self.window_size / 10 * 7), int(self.window_size / 10 * 5.5), int(self.window_size / 200 * 15))),  # Obstacle 7
            ('circle_Occupied', (int(self.window_size / 10 * 3.5), int(self.window_size / 10 * 3.2), int(self.window_size / 200 * 20))),# Obstacle 8
            ('circle_Occupied', (int(self.window_size / 10 * 4), int(self.window_size / 10 * 7.7), int(self.window_size / 200 * 10))),  # Obstacle 9
            ('circle_Occupied', (int(self.window_size / 10 * 7), int(self.window_size / 10 * 5.9), int(self.window_size / 200 * 30))),  # Obstacle 10
        ]


    def get_environment(self):
        """ Function to Return Environment Details """
        return self.window_size,self.obstacles,self.start,self.goal


