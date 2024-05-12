import numpy as np
import random
from problems import visualizer_2D as viz2D

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

def is_collision_free(node, obstacles):
    """Check if a node does not collide with any obstacles:
    For polygons: ('polygon', [(x1, y1), (x2, y2), ...])
    For circles: ('circle', (center_x, center_y, radius))"""
    x, y = node.x, node.y
    for obs_type, shape in obstacles:
        if obs_type == 'polygon' and point_in_polygon((x, y), shape):
            return False
        elif obs_type == 'circle' and point_in_circle((x, y), shape):
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
        self.goal_region = 0.1

    def distance_hypot(self, p1, p2):
        return np.hypot(p2.x - p1.x, p2.y - p1.y)

    def sample(self):
        if random.random() < self.goal_bias:
            return self.goal
        else:
            return RRTNode(random.uniform(self.bounds[0], self.bounds[1]), random.uniform(self.bounds[2], self.bounds[3]))

    def nearest(self, point):
        return min(self.nodes, key=lambda node: self.distance_hypot(node, point))

    def steer(self, from_node, to_node):
        dist = self.distance_hypot(from_node, to_node)
        if dist < self.step_size:
            return to_node
        ratio = self.step_size / dist
        new_x = from_node.x + ratio * (to_node.x - from_node.x)
        new_y = from_node.y + ratio * (to_node.y - from_node.y)
        return RRTNode(new_x, new_y)

    def is_safe(self, point):
        # This method should include the logic to check collisions
        return is_collision_free(point, self.obstacles)

    def plan(self):
        for i in range(self.max_iters):
            random_node = self.sample()
            nearest_node = self.nearest(random_node)
            new_node = self.steer(nearest_node, random_node)
            if self.is_safe(new_node):
                new_node.set_parent(nearest_node, nearest_node.cost + self.distance_hypot(nearest_node, new_node))
                self.nodes.append(new_node)
                self.rewire(new_node)  # Rewire the tree with respect to the new node
                if self.distance_hypot(new_node, self.goal) <= self.goal_region:
                    self.goal.set_parent(new_node, new_node.cost + self.distance_hypot(new_node, self.goal))
                    return self.extract_path(self.goal)
                
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
        # Define a search radius based on problem dimension and tree cardinality
        search_radius = self.step_size * 5  # Example radius, should be adjusted based on your specific needs
        # Find nearby nodes within the search radius
        nearby_nodes = [node for node in self.nodes if self.distance_hypot(node, new_node) < search_radius and node != new_node]
        for node in nearby_nodes:
            new_cost = new_node.cost + self.distance_hypot(new_node, node)
            if new_cost < node.cost:  # Found a better path to this node via new_node
                if self.is_safe(node):  # Make sure path from new_node to node is free of obstacles
                    node.set_parent(new_node, new_cost)

        # Check if new_node provides a better path to any of its non-direct children
        for node in nearby_nodes:
            potential_cost = new_node.cost + self.distance_hypot(new_node, node)
            if potential_cost < node.cost and self.is_safe(node):
                node.set_parent(new_node, potential_cost)
                self.rewire(node)  # Recursively rewire the tree if the path cost improved

    
    def get_nodes(self):
        self.plan() 
        return self.nodes  
