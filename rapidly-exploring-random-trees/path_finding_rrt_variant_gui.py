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

def grid_line(p0, p1):
    """
    Return a list of all grid cells (row, col) that the line segment
    from p0 to p1 passes through. Each point is a tuple (x, y).
    Assumes each grid cell is 1x1, with integer boundaries.
    """
    x0, y0 = p0
    x1, y1 = p1
    cells = []

    # Convert start/end to integer grid coordinates
    x, y = int(math.floor(x0)), int(math.floor(y0))
    end_x, end_y = int(math.floor(x1)), int(math.floor(y1))

    dx = x1 - x0
    dy = y1 - y0

    # Step direction
    step_x = 1 if dx > 0 else -1
    step_y = 1 if dy > 0 else -1

    # Avoid division by zero
    t_delta_x = abs(1.0 / dx) if dx != 0 else math.inf
    t_delta_y = abs(1.0 / dy) if dy != 0 else math.inf

    # Initial t_max values
    if dx > 0:
        t_max_x = ((x + 1) - x0) / dx
    else:
        t_max_x = (x0 - x) / -dx if dx < 0 else math.inf

    if dy > 0:
        t_max_y = ((y + 1) - y0) / dy
    else:
        t_max_y = (y0 - y) / -dy if dy < 0 else math.inf

    # Traverse the grid
    while True:
        cells.append((y, x))  # note: (row, col) = (y, x)

        if x == end_x and y == end_y:
            break

        if t_max_x < t_max_y:
            x += step_x
            t_max_x += t_delta_x
        else:
            y += step_y
            t_max_y += t_delta_y

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

        random_x = random.randint(0,ROWS-1)
        random_y = random.uniform(0, COLS-1)
        new_node = (random_x, random_y)


        cells_current_node_to_goal = grid_line(grid_to_world(current_node), grid_to_world(goal))

        if not collision(cells_current_node_to_goal):
            tree.append({"pos": goal, "parent": current_node})
            node_list.append(goal)
            return

        cells_on_line = grid_line(grid_to_world(current_node), grid_to_world(new_node))
        if new_node not in node_list and not collision(cells_on_line):
            tree.append({"pos": new_node, "parent": current_node})
            node_list.append(new_node)
            current_node = new_node





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
