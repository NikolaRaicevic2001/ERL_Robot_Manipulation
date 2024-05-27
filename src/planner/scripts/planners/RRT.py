import numpy as np
import random
from problems import visualizer_2D as viz2D

from scipy.spatial import KDTree

class RRTNode:
    def __init__(self, x, y, parent=None, cost = 0):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost
        self.children = []

    def set_parent(self, parent, cost):
        self.parent = parent
        self.cost = cost
        parent.children.append(self)

    @staticmethod
    def from_tuple(data):
        x, y, px, py = data
        parent = RRTNode(px, py) if px is not None and py is not None else None
        return RRTNode(x, y, parent)

def distance(node1, node2):
    return np.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

def point_in_polygon(point, polygon):
    """ Determine if the point (x, y) is inside the given polygon (ray-casting algorithm)"""
    x, y = point
    inside = False
    n = len(polygon)                
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def point_in_circle(point, circle):
    """Check if a point (x, y) is inside a circle defined by (center_x, center_y, radius)."""
    px, py = point
    cx, cy, radius = circle
    return (px - cx) ** 2 + (py - cy) ** 2 <= radius ** 2

def sdCircle(point, circle_SDF):
    """ Signed Distance Function for a circle. """
    px, py = point
    cx, cy, radius = circle_SDF
    distance_to_center = np.sqrt((px - cx) ** 2 + (py - cy) ** 2)
    return distance_to_center - radius # The signed distance from the point to the circle.


def sdEllipse(p, ellipse_SDF):
    x_pos, y_pos, a, b = ellipse_SDF  # Unpacking the ellipse center and semi-axes
    p = np.abs([p[0] - x_pos, p[1] - y_pos])  # Translate point to ellipse-centered coordinates

    if p[0] > p[1]:
        p = p[::-1]
        a, b = b, a  # Swap axes if needed after reordering point coordinates

    l = b**2 - a**2
    m = a * p[0] / l
    m2 = m**2
    n = b * p[1] / l
    n2 = n**2
    c = (m2 + n2 - 1.0) / 3.0
    c3 = c**3
    q = c3 + m2 * n2 * 2.0
    d = c3 + m2 * n2
    g = m + m * n2

    if d < 0.0:
        h = np.arccos(q / c3) / 3.0
        s = np.cos(h)
        t = np.sin(h) * np.sqrt(3.0)
        rx = np.sqrt(-c * (s + t + 2.0) + m2)
        ry = np.sqrt(-c * (s - t + 2.0) + m2)
        co = (ry + np.sign(l) * rx + np.abs(g) / (rx * ry) - m) / 2.0
    else:
        h = 2.0 * m * n * np.sqrt(d)
        s = np.sign(q + h) * np.power(np.abs(q + h), 1.0 / 3.0)
        u = np.sign(q - h) * np.power(np.abs(q - h), 1.0 / 3.0)
        rx = -s - u - c * 4.0 + 2.0 * m2
        ry = (s - u) * np.sqrt(3.0)
        rm = np.sqrt(rx**2 + ry**2)
        co = (ry / np.sqrt(rm - rx) + 2.0 * g / rm - m) / 2.0

    r = [a * co, b * np.sqrt(1.0 - co**2)]
    return np.linalg.norm(r - p) * np.sign(p[1] - r[1])

def is_collision_free(node, obstacles):
    """ Check if a node does not collide with any obstacles """
    x, y = node.x, node.y
    for obs_type, shape in obstacles:
        if obs_type == 'polygon' and point_in_polygon((x, y), shape):
            return False
        elif obs_type == 'circle' and point_in_circle((x, y), shape):
            return False
        elif obs_type == 'circle_SDF' and sdCircle((x, y), shape) < 50:
            return False
        elif obs_type == 'ellipse_SDF' and sdEllipse((x, y), shape) < 50:
            return False
    return True

######################################################
#################### RRT Algorith ####################
######################################################

class RRT:
    def __init__(self, start_pos, goal_pos, obstacles, goal_bias=0.05, step_size=20, max_iterations=1000):
        self.start = RRTNode(start_pos[0], start_pos[1])
        self.goal = RRTNode(goal_pos[0], goal_pos[1])
        self.obstacles = obstacles
        self.nodes = [self.start]
        self.goal_bias = goal_bias
        self.step_size = step_size
        self.max_iterations = max_iterations

    def find_nearest(self, point):
        nearest = None
        min_dist = float('inf')
        for node in self.nodes:
            d = distance(node, point)
            if d < min_dist:
                nearest = node
                min_dist = d
        return nearest

    def is_safe(self, point):
        # This method should include the logic to check collisions
        return is_collision_free(point, self.obstacles)

    def extend_tree(self):
        for i in range(self.max_iterations):
            if random.random() < self.goal_bias:
                sample = self.goal
            else:
                sample = RRTNode(random.uniform(0, 1000), random.uniform(0, 1000))

            nearest = self.find_nearest(sample)
            dir_x = sample.x - nearest.x
            dir_y = sample.y - nearest.y
            dist = np.sqrt(dir_x ** 2 + dir_y ** 2)
            step_x = nearest.x + (dir_x / dist) * self.step_size
            step_y = nearest.y + (dir_y / dist) * self.step_size
            new_node = RRTNode(step_x, step_y, nearest)

            if self.is_safe(new_node):
                self.nodes.append(new_node)
                if distance(new_node, self.goal) <= self.step_size:
                    self.goal.parent = new_node
                    self.nodes.append(self.goal)
                    return True
        return False
    
    def get_nodes(self):
        self.extend_tree() 
        return self.nodes  

######################################################
############### RRT Star Algorith ####################
######################################################

class RRT_Star:
    def __init__(self, start, goal, obstacles, bounds, step_size=0.1, goal_bias=0.1, max_iterations=1000):
        self.start = RRTNode(start[0], start[1])
        self.goal = RRTNode(goal[0], goal[1])
        self.obstacles = obstacles
        self.bounds = bounds
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.max_iters = max_iterations
        self.nodes = [self.start]
        self.kdtree = KDTree([(self.start.x, self.start.y)])
        self.goal_region = 0.1

    def sample(self):
        if random.random() < self.goal_bias:
            return (self.goal.x, self.goal.y)
        else:
            x = random.uniform(self.bounds[0], self.bounds[1])
            y = random.uniform(self.bounds[2], self.bounds[3])
            return (x, y)

    def nearest(self, point):
        distances, indexes = self.kdtree.query([point], k=1)
        return self.nodes[indexes[0]]

    def steer(self, from_node, to_point):
        dist = distance(from_node, RRTNode(to_point[0], to_point[1]))
        if dist < self.step_size:
            return RRTNode(to_point[0], to_point[1], parent=from_node)
        ratio = self.step_size / dist
        new_x = from_node.x + ratio * (to_point[0] - from_node.x)
        new_y = from_node.y + ratio * (to_point[1] - from_node.y)
        return RRTNode(new_x, new_y, parent=from_node)

    def plan(self):
        for i in range(self.max_iters):
            rand_point = self.sample()
            nearest_node = self.nearest(rand_point)
            new_node = self.steer(nearest_node, rand_point)

            if is_collision_free(new_node, self.obstacles):
                new_node.set_parent(nearest_node, nearest_node.cost + distance(nearest_node, new_node))
                self.nodes.append(new_node)
                self.kdtree = KDTree([(node.x, node.y) for node in self.nodes])
                self.rewire(new_node)

                if distance(new_node, self.goal) <= self.goal_region:
                    print("Goal reached")
                    self.goal.set_parent(new_node, new_node.cost + distance(new_node, self.goal))
                    return self.extract_path(self.goal)
            else:
                # print(f"Collision detected at: ({new_node.x}, {new_node.y})")
                pass
        print("Maximum iterations reached without finding a path.")
        return None


    def extract_path(self, end_node):
        path = []
        current_node = end_node
        while current_node.parent is not None:
            path.append((current_node.x, current_node.y))
            current_node = current_node.parent
        path.append((self.start.x, self.start.y))
        return path[::-1]

    def rewire(self, new_node):
        N = len(self.nodes)
        rad = 1.414 * (np.log(N) / N) ** (0.5)
        neighbors = [node for node in self.nodes if distance(node, new_node) < rad and node != new_node]

        for node in neighbors:
            new_cost = new_node.cost + distance(new_node, node)
            if new_cost < node.cost and is_collision_free(node, self.obstacles):
                node.set_parent(new_node, new_cost)

    def get_nodes(self):
        self.plan()
        return self.nodes
