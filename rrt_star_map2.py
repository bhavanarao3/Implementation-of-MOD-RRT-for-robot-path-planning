import pygame
import random
import math
import time

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("RRT* Path Planning")
clock = pygame.time.Clock()

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 255, 255)

# Obstacle and Map Settings
START = (50, 550)
GOAL = (750, 50)
NODE_RADIUS = 5
STEP_SIZE = 20
OBSTACLES = [
    ("circle", 600, 400, 100),
    ("rect", 0, 200, 400, 50),
    ("rect", 0, 400, 400, 50),
    ("rect", 0, 100, 400, 50),
    ("circle", 600, 100, 100)
]


# Node class for the RRT* Tree
class Node:
    def __init__(self, point, parent=None):
        self.point = point
        self.parent = parent
        self.cost = 0

# Draw obstacles on the map
def draw_map():
    screen.fill(WHITE)
    pygame.draw.circle(screen, RED, START, NODE_RADIUS)
    pygame.draw.circle(screen, RED, GOAL, NODE_RADIUS)
    for shape, *args in OBSTACLES:
        if shape == "circle":
            pygame.draw.circle(screen, BLACK, args[:2], args[2])
        elif shape == "rect":
            pygame.draw.rect(screen, BLACK, args)

# Check if a point is within an obstacle

def in_obstacle(point):
    x, y = point
    for shape, *args in OBSTACLES:
        if shape == "circle":
            ox, oy, r = args
            if distance((ox, oy), (x, y)) <= r:
                return True
        elif shape == "rect":
            ox, oy, w, h = args
            if ox <= x <= ox + w and oy <= y <= oy + h:
                return True
    return False

def in_circle(point, circle_args, clearance):
    cx, cy, r = circle_args
    distance_to_center = math.sqrt((point[0] - cx) ** 2 + (point[1] - cy) ** 2)
    return distance_to_center <= r + clearance + NODE_RADIUS

def in_rect(point, rect_args, clearance):
    x, y, w, h = rect_args
    return x <= point[0] <= x + w and y <= point[1] <= y + h


# Extend the RRT* Tree with clearance around obstacles
def extend_rrt_star(nodes, goal):
    nodes_explored = 0
    clearance = 20  # Set the clearance distance
    while True:
        rand_point = (random.randint(0, 800), random.randint(0, 600))
        nearest_node = min(nodes, key=lambda node: distance(node.point, rand_point))
        angle = math.atan2(rand_point[1] - nearest_node.point[1], rand_point[0] - nearest_node.point[0])
        new_point = (nearest_node.point[0] + STEP_SIZE * math.cos(angle), nearest_node.point[1] + STEP_SIZE * math.sin(angle))

        # Check if the new point is within the clearance distance of any obstacle
        if not any(
            (shape == "rect" and in_rect(new_point, args, clearance)) or
            (shape == "circle" and in_circle(new_point, args, clearance))
            for shape, *args in OBSTACLES
        ):
            if not in_obstacle(new_point):
                new_node = Node(new_point, nearest_node)
                new_node.cost = nearest_node.cost + distance(nearest_node.point, new_point)
                near_nodes = find_near_nodes(nodes, new_node, 50)
                min_cost_node = nearest_node
                min_cost = nearest_node.cost + distance(nearest_node.point, new_node.point)
                for near_node in near_nodes:
                    if not in_obstacle(new_node.point):
                        new_cost = near_node.cost + distance(near_node.point, new_node.point)
                        if new_cost < min_cost:
                            min_cost = new_cost
                            min_cost_node = near_node
                new_node.parent = min_cost_node
                nodes.append(new_node)
                for near_node in near_nodes:
                    if not in_obstacle(new_node.point):
                        new_cost = new_node.cost + distance(near_node.point, new_node.point)
                        if new_cost < near_node.cost:
                            near_node.parent = new_node
                            near_node.cost = new_cost
                nodes_explored += 1
                break

    return nodes, new_node, nodes_explored
# Find nodes in the neighborhood
def find_near_nodes(nodes, new_node, radius):
    near_nodes = []
    for node in nodes:
        if distance(node.point, new_node.point) <= radius:
            near_nodes.append(node)
    return near_nodes

# Calculate distance between two points
def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

# Visualization
def draw_tree(nodes):
    for node in nodes:
        if node.parent:
            pygame.draw.line(screen, GREEN, node.point, node.parent.point, 2)
        pygame.draw.circle(screen, BLUE, (int(node.point[0]), int(node.point[1])), NODE_RADIUS)

# Draw final path
def draw_final_path(nodes, goal):
    path = []
    current_node = nodes[-1]  # Last added node
    while current_node:
        path.append(current_node.point)
        current_node = current_node.parent

    for i in range(len(path) - 1):
        pygame.draw.line(screen, RED, path[i], path[i + 1], 2)
    return path

# Main Loop
def main():
    start = time.time()
    nodes = [Node(START)]
    nodes_explored = 0

    running = True
    while running:
        draw_map()
        draw_tree(nodes)

        nodes, new_node, explored = extend_rrt_star(nodes, GOAL)
        nodes_explored += explored

        if new_node and distance(new_node.point, GOAL) < STEP_SIZE:
            path = draw_final_path(nodes, GOAL)
            pygame.display.flip()
            end = time.time()
            execution = end - start
            print("Execution time: ", execution)
            print("Nodes Explored: ", nodes_explored)
            print("Number of Nodes in the Path: ", len(path))
            time.sleep(10)
            break

        pygame.display.flip()
        clock.tick(30)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

if __name__ == '__main__':
    main()
