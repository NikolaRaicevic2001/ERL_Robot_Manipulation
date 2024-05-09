import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np

class Environment3D:
    def __init__(self, obstacles, start, goal):
        self.obstacles = obstacles
        self.start = np.array(start)
        self.goal = np.array(goal)

    def draw(self):
        for shape, vertices in self.obstacles:
            if shape == 'cube':
                self.draw_cube(vertices)
            elif shape == 'pyramid':
                self.draw_pyramid(vertices)
        
        glColor3f(1, 0, 0)  # Red for start
        self.draw_sphere(self.start, 0.1)
        glColor3f(0, 1, 0)  # Green for goal
        self.draw_sphere(self.goal, 0.1)

    def draw_cube(self, vertices):
        glBegin(GL_QUADS)
        for surface in vertices:
            for vertex in surface:
                glVertex3fv(vertex)
        glEnd()

    def draw_pyramid(self, vertices):
        glBegin(GL_TRIANGLES)
        for vertex in vertices:
            glVertex3fv(vertex)
        glEnd()

    def draw_sphere(self, position, radius):
        quadric = gluNewQuadric()
        glPushMatrix()
        glTranslatef(*position)
        gluSphere(quadric, radius, 32, 32)
        glPopMatrix()

class Viewer3D:
    def __init__(self, width, height, environment):
        self.environment = environment
        pygame.init()
        pygame.display.set_mode((width, height), DOUBLEBUF | OPENGL)
        gluPerspective(45, (width / height), 0.1, 50.0)
        glTranslatef(0.0, 0.0, -5)

    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            self.handle_keys()
            self.environment.draw()
            pygame.display.flip()
        pygame.quit()

    def handle_keys(self):
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]:
            glRotatef(0.5, 0, 0.5, 0)
        if keys[pygame.K_RIGHT]:
            glRotatef(-0.5, 0, 0.5, 0)
        if keys[pygame.K_UP]:
            glRotatef(0.5, 0.5, 0, 0)
        if keys[pygame.K_DOWN]:
            glRotatef(-0.5, 0.5, 0, 0)
        if keys[pygame.K_w]:  # Move camera forward
            glTranslatef(0, 0, 0.5)
        if keys[pygame.K_s]:  # Move camera backward
            glTranslatef(0, 0, -0.5)
        if keys[pygame.K_a]:  # Zoom in
            glScalef(1.1, 1.1, 1.1)
        if keys[pygame.K_d]:  # Zoom out
            glScalef(0.9, 0.9, 0.9)

if __name__ == "__main__":
    # Define the problem environment with obstacles
    obstacles = [
        ('cube', [
            [[1, 1, 1], [1, 1, -1], [1, -1, -1], [1, -1, 1]],
            [[-1, 1, -1], [-1, 1, 1], [-1, -1, 1], [-1, -1, -1]],
            [[1, 1, -1], [-1, 1, -1], [-1, -1, -1], [1, -1, -1]],
            [[-1, 1, 1], [1, 1, 1], [1, -1, 1], [-1, -1, 1]],
            [[1, 1, 1], [-1, 1, 1], [-1, 1, -1], [1, 1, -1]],
            [[1, -1, -1], [-1, -1, -1], [-1, -1, 1], [1, -1, 1]]
        ]),
        ('pyramid', [
            [0, 1, 0], [-1, -1, 1], [1, -1, 1],  # Front face
            [0, 1, 0], [1, -1, 1], [1, -1, -1],  # Right face
            [0, 1, 0], [1, -1, -1], [-1, -1, -1],  # Back face
            [0, 1, 0], [-1, -1, -1], [-1, -1, 1]  # Left face
        ])
    ]
    start = (0, 0, 0)
    goal = (2, 2, 2)
    env = Environment3D(obstacles, start, goal)
    viewer = Viewer3D(1000, 1000, env)
    viewer.run()
