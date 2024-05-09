import random

from problems import visualizer_2D as viz2D

class Dodging_Cones:
    def __init__(self):

        window_size = 1000
        num_obstacles = 100

        # Define the problem environment with different obstacle types
        obstacles = []
        for i in range(num_obstacles):
            x_pos = random.randint(int(window_size/10),int(window_size/10*9))
            y_pos = random.randint(0,int(window_size))
            radius = random.randint(int(window_size/100),int(window_size/100*5))
            obstacles.append( ('circle', (x_pos,y_pos,radius)) )
            # print(obstacles[i])

        start = (50, 50)
        goal = (950, 800)
        env = viz2D.Environment(obstacles, start, goal)

        app = viz2D.PlannerApp(window_size, window_size, env)
        app.run()

        # new_start = (100,100)
        # new_goal = (200,200)

        # env.update_start_goal(new_start, new_goal)
