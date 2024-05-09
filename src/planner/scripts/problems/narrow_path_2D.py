from problems import visualizer_2D as viz2D
   
class Narrow_Path:
    def __init__(self):
        window_size = 1000
        # Define the problem environment with different obstacle types
        obstacles = [
            ('polygon', [(window_size/10, 0), (window_size/10, window_size/2), (window_size/2, window_size/2), (window_size/2, window_size/10*4), (window_size/10*9, window_size/10*4), (window_size/10*9,0)]),
            ('polygon', [(window_size/10, window_size), (window_size/10, window_size/10*5.5), (window_size/10*5.5, window_size/10*5.5), (window_size/10*5.5,window_size)]),
            ('polygon', [(window_size/10*5.5, window_size), (window_size/10*5.5, window_size/10*4.5), (window_size/10*9, window_size/10*4.5), (window_size/10*9,window_size)])
        ]

        start = (50, 450)
        goal = (950, 500)
        env = viz2D.Environment(obstacles, start, goal)

        app = viz2D.PlannerApp(window_size, window_size, env)
        app.run()
