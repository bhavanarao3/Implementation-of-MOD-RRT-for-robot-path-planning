import pygame
import random
import math
import time
import numpy as np

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("MOD-RRT* Path Planning")
clock = pygame.time.Clock()

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 255, 255)
PURPLE = (128, 0, 128)
YELLOW = (255, 165, 0)

# Obstacle and Map Settings
STATIC_OBSTACLES = [(300, 200, 100, 100), (500, 400, 100, 100), (100, 300, 50, 50)]
DYNAMIC_OBSTACLES = []
START = (50, 50)
GOAL = (700, 520)
NODE_RADIUS = 5
STEP_SIZE = 20

# Node class for the RRT* Tree
class Node:
    def __init__(self, point, parent=None):
        self.point = point
        self.parent = parent
        self.cost = 0

# Draw obstacles on the map
def draw_map():
    screen.fill(WHITE)
    for obstacle in STATIC_OBSTACLES:
        pygame.draw.rect(screen, BLACK, obstacle)
    for obstacle in DYNAMIC_OBSTACLES:
        if not any(rect_overlap(obstacle, static_obstacle) for static_obstacle in STATIC_OBSTACLES):
            pygame.draw.rect(screen, PURPLE, obstacle)

# Check if two rectangles overlap
def rect_overlap(rect1, rect2):
    x1, y1, w1, h1 = rect1
    x2, y2, w2, h2 = rect2
    return x1 < x2 + w2 and x1 + w1 > x2 and y1 < y2 + h2 and y1 + h1 > y2

# Check if a point is within a clearance distance of an obstacle
def in_obstacle(point, clearance=10):
    x, y = point
    for obstacle in STATIC_OBSTACLES + DYNAMIC_OBSTACLES:
        if obstacle[0] - clearance <= x <= obstacle[0] + obstacle[2] + clearance and \
           obstacle[1] - clearance <= y <= obstacle[1] + obstacle[3] + clearance:
            return True
    return False


# Extend the RRT* Tree with obstacle avoidance and Pareto dominance
def mod_rrt_star(nodes, goal):
    while True:
        rand_point = (random.randint(0, 800), random.randint(0, 600))
        nearest_node = min(nodes, key=lambda node: distance(node.point, rand_point))

        # Calculate cost and distance to goal for the new point
        angle = math.atan2(rand_point[1] - nearest_node.point[1], rand_point[0] - nearest_node.point[0])
        new_point = (nearest_node.point[0] + STEP_SIZE * math.cos(angle), nearest_node.point[1] + STEP_SIZE * math.sin(angle))
        new_cost = nearest_node.cost + distance(nearest_node.point, new_point)
        new_distance_to_goal = distance(new_point, GOAL)

        # Check for collision with static obstacles
        if not in_obstacle(new_point) and not intersects_dynamic_obstacle(nearest_node.point, new_point):
            # Check for Pareto dominance
            dominated = False
            for node in nodes:
                if node.cost <= new_cost and distance(node.point, GOAL) <= new_distance_to_goal:
                    dominated = True
                    break
            if not dominated:
                new_node = Node(new_point, nearest_node)
                new_node.cost = new_cost
                nodes.append(new_node)
                break

    return nodes, new_node

# Check if the line segment between two points intersects with any dynamic obstacle
def intersects_dynamic_obstacle(point1, point2):
    for obstacle in DYNAMIC_OBSTACLES:
        if line_intersects_rect(point1, point2, obstacle):
            return True
    return False

# Check if a line segment intersects with a rectangle
def line_intersects_rect(point1, point2, rect):
    x1, y1 = point1
    x2, y2 = point2
    x, y, w, h = rect

    left, right = min(x1, x2), max(x1, x2)
    top, bottom = min(y1, y2), max(y1, y2)

    if x <= left <= x + w and y <= top <= y + h:
        return True
    if x <= right <= x + w and y <= bottom <= y + h:
        return True
    if left <= x <= right and top <= y <= bottom:
        return True
    if left <= x + w <= right and top <= y + h <= bottom:
        return True

    return False

# Calculate distance between two points
def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

# Simplify the path by removing unnecessary points
def simplify_path(path):
    simplified_path = [path[0]]
    for i in range(1, len(path) - 1):
        if not line_intersects_rect(simplified_path[-1], path[i + 1], DYNAMIC_OBSTACLES[0]):  # Corrected function call
            simplified_path.append(path[i])
    simplified_path.append(path[-1])
    return simplified_path


# Interpolate curves between points using spline interpolation
def interpolate_path(path):
    interpolated_path = []
    for i in range(len(path) - 1):
        interpolated_path.extend(interpolate_segment(path[i], path[i + 1]))
    return interpolated_path

# Interpolate a curve between two points using cubic spline interpolation
def interpolate_segment(point1, point2):
    t = np.linspace(0, 1, 10)
    x = (1 - t) * point1[0] + t * point2[0]
    y = (1 - t) * point1[1] + t * point2[1]
    return list(zip(x, y))

def smooth_path(path):
    smoothed_path = [path[0]]  # Start with the first point

    for i in range(1, len(path) - 1):
        # Check if a straight line between path[i] and any other point in the path intersects any obstacles
        intersects = False
        for j in range(i+1, len(path)):
            if line_intersects_obstacles(path[i], path[j]):
                intersects = True
                break

        if not intersects:
            smoothed_path.append(path[i])  # Add the current point to the smoothed path

    smoothed_path.append(path[-1])  # Add the last point

    return smoothed_path


# Check if a line segment between two points intersects with any obstacles
def line_intersects_obstacles(point1, point2):
    for obstacle in STATIC_OBSTACLES + DYNAMIC_OBSTACLES:
        if line_intersects_rect(point1, point2, obstacle):
            return True
    return False

# Visualization
def draw_tree(nodes):
    for node in nodes:
        if node.parent:
            pygame.draw.line(screen, GREEN, node.point, node.parent.point, 2)
        pygame.draw.circle(screen, BLUE, (int(node.point[0]), int(node.point[1])), NODE_RADIUS)

# Draw final path with specified color
def draw_final_path(nodes, goal, color):
    path = []
    current_node = nodes[-1]  # Last added node
    while current_node:
        path.append(current_node.point)
        current_node = current_node.parent

    for i in range(len(path) - 1):
        pygame.draw.line(screen, color, path[i], path[i + 1], 2)

# Draw the smoothed path on the screen with specified color
def draw_smoothed_path(path, color):
    for i in range(len(path) - 1):
        pygame.draw.line(screen, color, path[i], path[i + 1], 2)

# Check if the path intersects with dynamic obstacles
def path_intersects_obstacle(nodes, goal):
    path = []
    current_node = nodes[-1]  # Last added node
    while current_node:
        path.append(current_node.point)
        current_node = current_node.parent

    for i in range(len(path) - 1):
        if intersects_dynamic_obstacle(path[i], path[i + 1]):
            return True
    return False

# Main Loop
def main():
    start_time = time.time()
    nodes = [Node(START)]
    dynamic_obstacles_added = False  # Flag to indicate if dynamic obstacles are added

    running = True
    while running:
        draw_map()
        draw_tree(nodes)

        if not dynamic_obstacles_added:
            if random.random() < 0.05 and len(DYNAMIC_OBSTACLES) < 5:  # Simulate detection of dynamic obstacles
                new_obstacle = (
                    random.randint(100, 700), random.randint(100, 500), random.randint(20, 80), random.randint(20, 80))
                DYNAMIC_OBSTACLES.append(new_obstacle)
                # Remove nodes that collide with the new obstacle
                nodes = [node for node in nodes if not in_obstacle(node.point)]

                # Check if dynamic obstacles are added
                if len(DYNAMIC_OBSTACLES) == 5:
                    dynamic_obstacles_added = True
        else:
            nodes, new_node = mod_rrt_star(nodes, GOAL)
            if new_node and distance(new_node.point, GOAL) < STEP_SIZE:
                # Draw final path
                draw_final_path(nodes, GOAL, RED)
                pygame.display.flip()
                time.sleep(5)

                # Path smoothing
                path = []
                current_node = nodes[-1]  # Last added node
                while current_node:
                    path.append(current_node.point)
                    current_node = current_node.parent


                execution_time = time.time() - start_time
                print("Execution Time:", execution_time, "seconds")

                # Calculate and print the total number of nodes explored
                total_nodes_explored = len(nodes)
                print("Total Nodes Explored:", total_nodes_explored)

                break

        pygame.display.flip()
        clock.tick(30)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

if __name__ == '__main__':
    main()
