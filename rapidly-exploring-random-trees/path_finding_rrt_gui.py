"""
This version doesn't cap the edge distance and always try to connect to the goal
Yolo
"""

import pygame
import math
import random
from collections import deque

# --- CONFIG ---
WIDTH, HEIGHT = 800, 800
ROWS, COLS = 40, 40
CELL_SIZE = WIDTH // COLS
RADIUS = 1
GOAL_RADIUS = 10
STEP_SIZE = 3
RRT_ITERATIONS = 1000

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
GREEN = (0, 255, 0)   # Start
RED = (255, 0, 0)     # Goal

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("ESDF Grid Pathfinding")

# --- GRID DATA ---
grid = [[0 for _ in range(COLS)] for _ in range(ROWS)]  # 0=free, 1=obstacle
esdf = [[0 for _ in range(COLS)] for _ in range(ROWS)]

start = None
goal = None
node_list = []
tree = []

def compute_esdf():
    """Compute ESDF (distance to nearest obstacle) using BFS/Dijkstra with 8-connectivity."""
    global esdf
    esdf = [[math.inf for _ in range(COLS)] for _ in range(ROWS)]
    from heapq import heappush, heappop

    pq = []  # priority queue (distance, row, col)

    # Initialize queue with obstacle cells
    for r in range(ROWS):
        for c in range(COLS):
            if grid[r][c] == 1:  # obstacle
                esdf[r][c] = 0
                heappush(pq, (0, r, c))

    # Dijkstra (since diagonal costs are √2)
    directions = [
        (1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),  # straight moves
        (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))  # diagonal moves
    ]

    while pq:
        dist, r, c = heappop(pq)
        if dist > esdf[r][c]:
            continue  # already found shorter

        for dr, dc, cost in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < ROWS and 0 <= nc < COLS:
                new_dist = dist + cost
                if new_dist < esdf[nr][nc]:
                    esdf[nr][nc] = new_dist
                    heappush(pq, (new_dist, nr, nc))


def draw_grid():
    # Background grid
    for r in range(ROWS):
        for c in range(COLS):
            x, y = c * CELL_SIZE, r * CELL_SIZE
            color = BLACK if grid[r][c] == 1 else WHITE
            pygame.draw.rect(screen, color, (x, y, CELL_SIZE, CELL_SIZE))
            pygame.draw.rect(screen, GRAY, (x, y, CELL_SIZE, CELL_SIZE), 1)

    # Start & goal
    if start:
        r, c = start
        pygame.draw.rect(screen, GREEN, (c*CELL_SIZE, r*CELL_SIZE, CELL_SIZE, CELL_SIZE))
    if goal:
        r, c = goal
        pygame.draw.rect(screen, RED, (c*CELL_SIZE, r*CELL_SIZE, CELL_SIZE, CELL_SIZE))


    # Edges (red lines between parent and node)
    if len(tree) > 1:
        for node in node_list:
            r, c = node
            center = cell_center(r, c)
            pygame.draw.circle(screen, (0, 0, 255), center, CELL_SIZE // 3)

        for entry in tree:
            if entry["parent"] is not None:
                # * means to unpack the tuple into 2 values
                start_pos = cell_center(*entry["parent"])
                end_pos   = cell_center(*entry["pos"])
                pygame.draw.line(screen, (255, 0, 0), start_pos, end_pos, 2)


def get_cell_from_mouse(pos):
    x, y = pos
    c, r = x // CELL_SIZE, y // CELL_SIZE
    return r, c

def cell_center(row, col):
    x = col * CELL_SIZE + CELL_SIZE // 2
    y = row * CELL_SIZE + CELL_SIZE // 2
    return (x, y)

def grid_to_world(cell):
    r, c = cell
    return (c + 0.5, r + 0.5)

def distance_between_nodes(node1, node2):
    return math.sqrt((node1[0]-node2[0])**2 + (node1[1]-node2[1])**2)

def find_nearest_node(node_list, new_node):
    min_distance = math.inf
    nearest_node = None
    for k in node_list:
        d = distance_between_nodes(k, new_node)
        if d < min_distance:
            min_distance = d
            nearest_node = k

    return nearest_node
    


def grid_line(p0, p1):
    """
    Return a list of all grid cells (row, col) that the line segment
    from p0 to p1 passes through. 
    p0 and p1 are floats in (row, col) space.
    Each grid cell is 1x1 with integer boundaries.
    """

    r0, c0 = p0   # floats (row, col)
    r1, c1 = p1

    cells = []

    # Convert to integer grid indices
    r, c = int(math.floor(r0)), int(math.floor(c0))
    end_r, end_c = int(math.floor(r1)), int(math.floor(c1))

    dr = r1 - r0
    dc = c1 - c0

    # Step direction
    step_r = 1 if dr > 0 else -1
    step_c = 1 if dc > 0 else -1

    # Avoid division by zero
    t_delta_r = abs(1.0 / dr) if dr != 0 else math.inf
    t_delta_c = abs(1.0 / dc) if dc != 0 else math.inf

    # Initial t_max values
    if dr > 0:
        t_max_r = ((r + 1) - r0) / dr
    else:
        t_max_r = (r0 - r) / -dr if dr < 0 else math.inf

    if dc > 0:
        t_max_c = ((c + 1) - c0) / dc
    else:
        t_max_c = (c0 - c) / -dc if dc < 0 else math.inf

    # Traverse the grid
    while True:
        cells.append((r, c))  # (row, col)

        if r == end_r and c == end_c:
            break

        if t_max_r < t_max_c:
            r += step_r
            t_max_r += t_delta_r
        else:
            c += step_c
            t_max_c += t_delta_c

    return cells



def collision(cell_list):
    """
    Takes a list of cells we pass through and returns True if collision is imminent
    """

    for k in cell_list:
        if RADIUS > esdf[k[0]][k[1]] - math.sqrt(2):
            return True

    return False

def run_rrt(esdf):
    if not start or not goal:
        print("Start or goal not set")
        return

    current_node = start
    for i in range(RRT_ITERATIONS):
                
        random_x = random.uniform(0,ROWS)
        random_y = random.uniform(0, COLS)
        random_point = (random_x, random_y)

        nearest_node = find_nearest_node(node_list, random_point)

        delta_x = random_x-nearest_node[0]
        delta_y = random_y-nearest_node[1]
        node_distance = distance_between_nodes(random_point, nearest_node)
        
        new_node_x = nearest_node[0] + ((delta_x/node_distance)*STEP_SIZE)
        new_node_y = nearest_node[1] + ((delta_y/node_distance) *STEP_SIZE)
        new_node = (new_node_x, new_node_y)

        
        cells_on_line = grid_line(nearest_node, new_node)
        

        if node_distance <= STEP_SIZE:
            cells_on_line = grid_line(nearest_node, random_point)
            if not collision(cells_on_line):
                tree.append({"pos": random_point, "parent": nearest_node})
                node_list.append(random_point)
                current_node = random_point

        elif not collision(cells_on_line):
            tree.append({"pos": new_node, "parent": nearest_node})
            node_list.append(new_node)
            current_node = new_node

        if distance_between_nodes(current_node, goal) <= GOAL_RADIUS:
            cells_current_node_to_goal = grid_line(current_node, goal)
            if not collision(cells_current_node_to_goal):
                tree.append({"pos": goal, "parent": current_node})
                node_list.append(goal)
                return




# --- MAIN LOOP ---
running = True
placing = False
removing = False

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.MOUSEBUTTONDOWN:
            r, c = get_cell_from_mouse(pygame.mouse.get_pos())
            if event.button == 1:  # left click
                if pygame.key.get_mods() & pygame.KMOD_SHIFT:
                    start = (r, c)
                    node_list.append(start)
                    tree.append({"pos": start, "parent": None})

                elif pygame.key.get_mods() & pygame.KMOD_CTRL:
                    goal = (r, c)
                else:
                    grid[r][c] = 1
                    placing = True
                    compute_esdf()
            elif event.button == 3:  # right click to erase
                grid[r][c] = 0
                removing = True
                compute_esdf()

        elif event.type == pygame.MOUSEBUTTONUP:
            placing = False
            removing = False

        elif event.type == pygame.MOUSEMOTION:
            if placing or removing:
                r, c = get_cell_from_mouse(pygame.mouse.get_pos())
                if placing:
                    grid[r][c] = 1
                elif removing:
                    grid[r][c] = 0
                compute_esdf()

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN:  # Enter key
                run_rrt(esdf)

    screen.fill(WHITE)
    draw_grid()
    pygame.display.flip()

pygame.quit()
