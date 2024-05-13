import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

from math import cos, sin, radians, pi

class Environment:
    def __init__(self, obstacles, start, goal, nodes):
        self.obstacles = obstacles  
        self.start = start 
        self.goal = goal   
        self.nodes = nodes

    def draw(self):# Draw obstacles
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        for obs_type, vertices in self.obstacles:
            if obs_type == 'polygon':
                glBegin(GL_POLYGON)
                glColor3f(0, 0, 1)  # Blue color for obstacles
                for vertex in vertices:
                    glVertex2f(*vertex)
                glEnd()
            elif obs_type == 'circle':
                glBegin(GL_TRIANGLE_FAN)
                glColor3f(0, 0, 1)  # Blue color for obstacles
                x, y, radius = vertices
                for angle in range(0, 361, 10):  # Circle approximation
                    glVertex2f(x + radius * cos(radians(angle)), y + radius * sin(radians(angle)))
                glEnd()
            elif obs_type == 'circle_SDF':
                num_segments = 100
                x, y, radius = vertices
                outer_radius = radius + 50

                glBegin(GL_TRIANGLE_FAN)
                glColor3f(1, 0, 0)  # Blue color for obstacles
                for angle in range(0, 361, 10):  # Circle approximation
                    glVertex2f(x + radius * cos(radians(angle)), y + radius * sin(radians(angle)))
                glEnd()

                # Draw the circle with gradient
                for i in range(num_segments):
                    theta = 2.0 * pi * i / num_segments
                    next_theta = 2.0 * pi * (i + 1) / num_segments
                    glBegin(GL_TRIANGLE_STRIP)
                    # Center of circle, full color intensity
                    glColor4f(1, 1, 0, 1)  # Yellow at the center
                    glVertex2f(x + radius * cos(theta), y + radius * sin(theta))
                    glVertex2f(x + radius * cos(next_theta), y + radius * sin(next_theta))

                    # Outer edge of fade effect
                    glColor4f(0, 1, 0, 0)  # Green fading to transparent
                    glVertex2f(x + outer_radius * cos(theta), y + outer_radius * sin(theta))
                    glVertex2f(x + outer_radius * cos(next_theta), y + outer_radius * sin(next_theta))
                    glEnd()
            elif obs_type == 'ellipse_SDF':
                num_segments = 100
                x, y, a, b = vertices
                outer_scale = 50

                # Draw the ellipse using triangle strips with gradient extending beyond the radii
                outer_a = a + outer_scale  # Extend the horizontal radius for gradient
                outer_b = b + outer_scale  # Extend the vertical radius for gradient
            
                glBegin(GL_TRIANGLE_STRIP)
                for i in range(num_segments + 1):  # +1 to close the loop
                    theta = 2.0 * pi * i / num_segments
                    # Inner edge of ellipse
                    glColor4f(1, 1, 0, 1)  # Yellow at the ellipse's edge
                    glVertex2f(x + a * cos(theta), y + b * sin(theta))
                    # Outer edge of gradient
                    glColor4f(0, 1, 0, 0)  # Green fading to transparent
                    glVertex2f(x + outer_a * cos(theta), y + outer_b * sin(theta))
                glEnd()

        # Draw RRT nodes and paths
        glColor3f(0, 0, 0)  # Black color for RRT paths
        glLineWidth(2)
        glBegin(GL_LINES)
        for node in self.nodes:
            if node.parent:
                glVertex2f(node.parent.x, node.parent.y)
                glVertex2f(node.x, node.y)
        glEnd()

        # Draw start and goal
        glPointSize(10)
        glBegin(GL_POINTS)
        glColor3f(0, 1, 0)  # Green color for start
        glVertex2f(*self.start)
        glColor3f(1, 0, 0)  # Red color for goal
        glVertex2f(*self.goal)
        glEnd()

    def update_start_goal(self, new_start=None, new_goal=None):
        if new_start:
            self.start = new_start
        if new_goal:
            self.goal = new_goal

class PlannerApp:
    def __init__(self, width, height, environment):
        self.width = width
        self.height = height
        self.environment = environment
        pygame.init()
        pygame.display.set_mode((width, height), DOUBLEBUF | OPENGL)
        gluOrtho2D(0, width, 0, height)
        self.mouse_down = False
        glClearColor(1, 1, 1, 1) 

    def draw_grid(self, color=(1, 1, 1), grid_size=20):
        """Draw a grid to help visualize the plane."""
        glBegin(GL_LINES)
        glColor3fv(color)
        # Draw horizontal lines
        for y in range(0, self.height + grid_size, grid_size):
            glVertex2f(0, y)
            glVertex2f(self.width, y)
        # Draw vertical lines
        for x in range(0, self.width + grid_size, grid_size):
            glVertex2f(x, 0)
            glVertex2f(x, self.height)
        glEnd()

    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == MOUSEBUTTONDOWN:
                    self.mouse_down = True
                    if event.button == 1:  # Left click to set start
                        x, y = pygame.mouse.get_pos()
                        self.environment.update_start_goal(new_start=(x, self.height - y))
                    elif event.button == 3:  # Right click to set goal
                        x, y = pygame.mouse.get_pos()
                        self.environment.update_start_goal(new_goal=(x, self.height - y))
                elif event.type == MOUSEBUTTONUP:
                    self.mouse_down = False

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            self.draw_grid()
            self.environment.draw()
            pygame.display.flip()
        pygame.quit()
