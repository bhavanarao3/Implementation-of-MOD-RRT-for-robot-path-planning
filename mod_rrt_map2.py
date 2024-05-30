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
STATIC_OBSTACLES = [
    ("circle", 600, 400, 100),
    ("rect", 0, 200, 400, 50),
    ("rect", 0, 400, 400, 50),
    ("rect", 0, 100, 400, 50),
    ("circle", 600, 100, 100)
]
DYNAMIC_OBSTACLES = []
START = (50, 550)
GOAL = (750, 50)
NODE_RADIUS = 5
STEP_SIZE = 20


# Node class for the RRT* Tree
class Node:
    def __init__(self, point, parent=None):
        self.point = point
        self.parent = parent
        self.cost = 0

def draw_map():
    screen.fill(WHITE)
    pygame.draw.circle(screen, RED, START, NODE_RADIUS)
    pygame.draw.circle(screen, RED, GOAL, NODE_RADIUS)

    # Add dynamic obstacles if needed
    if len(DYNAMIC_OBSTACLES) < 3 and len(DYNAMIC_OBSTACLES) < 1:
        if random.random() < 0.05:
            if random.choice([True, False]):  # Randomly choose between circle and rect
                # Generate circular dynamic obstacle
                new_obstacle = ("circle", random.randint(100, 700), random.randint(100, 500), random.randint(20, 40))
            else:
                # Generate rectangular dynamic obstacle
                new_obstacle = ("rect", random.randint(100, 700), random.randint(100, 500), random.randint(20, 80),
                                random.randint(20, 80))

            # Check if the new obstacle overlaps with any existing obstacles
            overlaps = False
            for obs in STATIC_OBSTACLES + DYNAMIC_OBSTACLES:
                if obs[0] == "circle" and new_obstacle[0] == "circle":
                    if circle_overlap(obs[1:], new_obstacle[1:]):
                        overlaps = True
                        break
                elif obs[0] == "rect" and new_obstacle[0] == "rect":
                    if rect_overlap(obs[1:], new_obstacle[1:]):
                        overlaps = True
                        break
                elif obs[0] == "circle" and new_obstacle[0] == "rect":
                    if circle_rect_overlap(obs[1:], new_obstacle[1:]):
                        overlaps = True
                        break
                elif obs[0] == "rect" and new_obstacle[0] == "circle":
                    if circle_rect_overlap(new_obstacle[1:], obs[1:]):
                        overlaps = True
                        break

            # Add the new obstacle if it doesn't overlap with any existing obstacles
            if not overlaps:
                DYNAMIC_OBSTACLES.append(new_obstacle)

    # Draw static obstacles
    for static_obs in STATIC_OBSTACLES:
        if static_obs[0] == "circle":
            _, x1, y1, r1 = static_obs
            pygame.draw.circle(screen, BLACK, (x1, y1), r1)
        elif static_obs[0] == "rect":
            _, x1, y1, w1, h1 = static_obs
            pygame.draw.rect(screen, BLACK, (x1, y1, w1, h1))

    # Draw dynamic obstacles
    for obs in DYNAMIC_OBSTACLES:
        if obs[0] == "circle":
            _, x, y, r = obs
            pygame.draw.circle(screen, PURPLE, (x, y), r)
        elif obs[0] == "rect":
            _, x, y, w, h = obs
            pygame.draw.rect(screen, PURPLE, (x, y, w, h))


# Check if a circle and a rectangle overlap
def circle_rect_overlap(circle, rect):
    cx, cy, cr = circle
    rx, ry, rw, rh = rect
    closest_x = max(rx, min(cx, rx + rw))
    closest_y = max(ry, min(cy, ry + rh))
    distance_squared = (cx - closest_x)**2 + (cy - closest_y)**2
    return distance_squared < cr**2

def rect_overlap(rect1, rect2):
    x1, y1, w1, h1 = rect1
    x2, y2, w2, h2 = rect2
    return x1 < x2 + w2 and x1 + w1 > x2 and y1 < y2 + h2 and y1 + h1 > y2

def circle_overlap(circle1, circle2):
    x1, y1, r1 = circle1
    x2, y2, r2 = circle2
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2) < r1 + r2



# Check if a point is within an obstacle
def in_obstacle(point, clearance=20):
    x, y = point
    for obs in STATIC_OBSTACLES:
        if obs[0] == "circle":
            _, ox, oy, r = obs
            if distance((ox, oy), (x, y)) <= r + clearance:
                return True
        elif obs[0] == "rect":
            _, ox, oy, w, h = obs
            if ox - clearance <= x <= ox + w + clearance and oy - clearance <= y <= oy + h + clearance:
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
    for obs in DYNAMIC_OBSTACLES:
        if obs[0] == "circle":
            _, ox, oy, r = obs
            if line_intersects_circle(point1, point2, (ox, oy), r):
                return True
        elif obs[0] == "rect":
            _, ox, oy, w, h = obs
            if line_intersects_rect(point1, point2, (ox, oy, w, h)):
                return True
    return False


def line_intersects_circle(point1, point2, center, radius):
    x1, y1 = point1
    x2, y2 = point2
    cx, cy = center

    # Vector from point1 to point2
    dx, dy = x2 - x1, y2 - y1

    # Vector from point1 to the circle's center
    fx, fy = cx - x1, cy - y1

    # Dot product
    dot = fx * dx + fy * dy

    # Nearest point on the line to the circle center
    t = dot / (dx * dx + dy * dy)

    # Clamp t to lie within the line segment
    t = max(0, min(1, t))

    nearest_x, nearest_y = x1 + t * dx, y1 + t * dy

    return distance((nearest_x, nearest_y), center) <= radius


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


def line_intersects_obstacle(point1, point2):
    for obs in STATIC_OBSTACLES + DYNAMIC_OBSTACLES:
        if obs[0] == "circle":
            _, ox, oy, r = obs
            if line_intersects_circle(point1, point2, (ox, oy), r):
                return True
        elif obs[0] == "rect":
            _, ox, oy, w, h = obs
            if line_intersects_rect(point1, point2, (ox, oy, w, h)):
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


# Smooth the path by optimizing the positions of the points
def smooth_path(path):
    smoothed_path = [path[0]]  # Start with the first point

    for i in range(1, len(path) - 1):
        # Calculate the angle between the start and end points and the current point
        angle1 = math.atan2(path[-1][1] - path[0][1], path[-1][0] - path[0][0])
        angle2 = math.atan2(path[i][1] - path[0][1], path[i][0] - path[0][0])

        # Check if the current point significantly deviates from the straight line
        if abs(angle1 - angle2) > math.pi / 30:  # Adjust the threshold angle as needed
            smoothed_path.append(path[i])  # Add the current point to the smoothed path

    smoothed_path.append(path[-1])  # Add the last point

    return smoothed_path


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
    running = True
    dynamic_obstacles_added = False

    while running:
        draw_map()
        draw_tree(nodes)

        if not dynamic_obstacles_added:
            # Check if dynamic obstacles are added
            if len(DYNAMIC_OBSTACLES) == 1:
                dynamic_obstacles_added = True
        else:
            # Extend the RRT* tree and check for goal
            nodes, new_node = mod_rrt_star(nodes, GOAL)
            if new_node and distance(new_node.point, GOAL) < STEP_SIZE:
                if path_intersects_obstacle(nodes, GOAL):
                    nodes = [Node(START)]
                else:
                    draw_final_path(nodes, GOAL, RED)
                    pygame.display.flip()
                    time.sleep(5)

                    # Path smoothing
                    path = []
                    current_node = nodes[-1]
                    while current_node:
                        path.append(current_node.point)
                        current_node = current_node.parent


                    execution_time = time.time() - start_time
                    print("Execution Time:", execution_time, "seconds")

                    # Calculate and print the total number of nodes explored
                    total_nodes_explored = len(nodes)
                    print("Total Nodes Explored:", total_nodes_explored)

                    # Calculate and print the number of nodes in the path
                    total_nodes_in_path = len(path)
                    print("Number of Nodes in the Path:", total_nodes_in_path)
                    break

        pygame.display.flip()
        clock.tick(30)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()


if __name__ == '__main__':
    main()
