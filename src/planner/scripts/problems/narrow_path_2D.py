from problems import visualizer_2D as viz2D
   
class Narrow_Path:
    def __init__(self):

        self.window_size = 1000
        window_size = self.window_size
        # Define the problem environment with different obstacle types
        self.obstacles = [
            ('polygon', [(window_size/10, 0), (window_size/10, window_size/2), (window_size/2, window_size/2), (window_size/2, window_size/10*4), (window_size/10*9, window_size/10*4), (window_size/10*9,0)]),
            ('polygon', [(window_size/10, window_size), (window_size/10, window_size/10*5.5), (window_size/10*5.5, window_size/10*5.5), (window_size/10*5.5,window_size)]),
            ('polygon', [(window_size/10*5.5, window_size), (window_size/10*5.5, window_size/10*4.5), (window_size/10*9, window_size/10*4.5), (window_size/10*9,window_size)])
        ]

        self.start = (50, 450)
        self.goal = (950, 500)

    def get_environment(self):
        """ Function to Return Environment Details """
        return self.window_size,self.obstacles,self.start,self.goal

