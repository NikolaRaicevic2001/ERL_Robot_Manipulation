import sys
import numpy as np
from scipy import ndimage
from math import cos, sin, radians, pi

import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

from planners import RRT

class Environment:
    def __init__(self, obstacles, start, goal, nodes, width, height, grid_resolution = 10):
        self.obstacles = obstacles  
        self.start = start 
        self.goal = goal   
        self.nodes = nodes
        self.width = width
        self.height = height
        self.grid_resolution = grid_resolution
        self.SDF_Map = np.full((height // grid_resolution, width // grid_resolution), float('inf'))
        self.grid = np.zeros((height // grid_resolution, width // grid_resolution), dtype=np.uint8)
        self.SDF_Map_Occupied = np.full((height // grid_resolution, width // grid_resolution), float('inf'))

    def draw(self):# Draw obstacles
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        # Occupied Obstacles
        for obs_type, params in self.obstacles:
            if obs_type == 'circle_Occupied':
                x_center, y_center, radius = params
                # Ensure grid indices are within bounds and calculate the correct indices
                for i in range(max(0, y_center - radius), min(self.height, y_center + radius)):
                    for j in range(max(0, x_center - radius), min(self.width, x_center + radius)):
                        if (i - y_center) ** 2 + (j - x_center) ** 2 <= radius ** 2:
                            self.grid[i // self.grid_resolution, j // self.grid_resolution] = 1

        if np.any(self.grid):
            dist_to_obstacle = ndimage.distance_transform_edt(1 - self.grid)
            dist_from_obstacle = ndimage.distance_transform_edt(self.grid)
            self.SDF_Map_Occupied = dist_to_obstacle - dist_from_obstacle

        # Draw SDF Map from Distance Transform
        if np.any(self.SDF_Map_Occupied < float('inf')):
            glBegin(GL_QUADS)
            max_distance = np.max(self.SDF_Map_Occupied)
            for grid_y in range(self.SDF_Map_Occupied.shape[0]):
                for grid_x in range(self.SDF_Map_Occupied.shape[1]):
                    x = grid_x * self.grid_resolution
                    y = grid_y * self.grid_resolution
                    distance = self.SDF_Map_Occupied[grid_y, grid_x]
                    normalized_distance = distance / max_distance
                    glColor3f(1.0 - normalized_distance, normalized_distance, 0)
                    glVertex2f(x, y)
                    glVertex2f(x + self.grid_resolution, y)
                    glVertex2f(x + self.grid_resolution, y + self.grid_resolution)
                    glVertex2f(x, y + self.grid_resolution)
            glEnd()

        # SDF Obstacles
        for obs_type, vertices in self.obstacles:
            if obs_type == 'circle_SDF':
                for grid_y in range(self.SDF_Map.shape[0]):
                    for grid_x in range(self.SDF_Map.shape[1]):
                        x = grid_x * self.grid_resolution + self.grid_resolution // 2
                        y = grid_y * self.grid_resolution + self.grid_resolution // 2
                        distance = RRT.sdCircle((x,y), vertices)
                        self.SDF_Map[grid_y,grid_x] = min(self.SDF_Map[grid_y,grid_x],distance)

            elif obs_type == 'ellipse_SDF':
                for grid_y in range(self.SDF_Map.shape[0]):
                    for grid_x in range(self.SDF_Map.shape[1]):
                        x = grid_x * self.grid_resolution + self.grid_resolution // 2
                        y = grid_y * self.grid_resolution + self.grid_resolution // 2
                        distance = RRT.sdEllipse((x,y), vertices)
                        self.SDF_Map[grid_y,grid_x] = min(self.SDF_Map[grid_y,grid_x],distance)

        # Draw SDF Map Min Distance
        if np.any(self.SDF_Map < float('inf')):
            glBegin(GL_QUADS)
            max_distance = np.max(np.abs(self.SDF_Map))
            for grid_y in range(self.SDF_Map.shape[0]):
                for grid_x in range(self.SDF_Map.shape[1]):
                    x = grid_x * self.grid_resolution
                    y = grid_y * self.grid_resolution
                    distance = self.SDF_Map[grid_y, grid_x]
                    normalized_distance = distance / max_distance
                    if distance < 0:
                        glColor3f(1.0, normalized_distance, normalized_distance)
                    else:
                        glColor3f(1.0 - normalized_distance, normalized_distance, 0.0)

                    # Draw a quad for each grid cell
                    glVertex2f(x, y)
                    glVertex2f(x + self.grid_resolution, y)
                    glVertex2f(x + self.grid_resolution, y + self.grid_resolution)
                    glVertex2f(x, y + self.grid_resolution)
            glEnd()


        # Obstacles
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
    def __init__(self, width, height, environment, grid_size=10):
        self.width = width
        self.height = height
        self.grid_size = grid_size
        self.environment = environment
        pygame.init()
        pygame.display.set_mode((width, height), DOUBLEBUF | OPENGL)
        gluOrtho2D(0, width, 0, height)
        glClearColor(1, 1, 1, 1) 
        self.mouse_down = False

    def draw_grid(self, color=(0.5, 0.5, 0.5), grid_size=10):
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
            self.draw_grid(grid_size = self.grid_size)
            self.environment.draw()
            pygame.display.flip()
        pygame.quit()

