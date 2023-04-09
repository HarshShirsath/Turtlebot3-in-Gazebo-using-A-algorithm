#!/usr/bin/env python3
import pygame
import numpy as np
import math
import time
import tkinter as tk
from tkinter import *
from tkinter import messagebox
import heapq
import os
import argparse

pygame.init()

WINDOW_WIDTH, WINDOW_HEIGHT = 1500, 500

screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Astar Algorithm Simulation")

MAP = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH), dtype=np.int8)
MAP_FILENAME = "map.txt"
# Define the colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 191, 255)
BLACK = (0, 0, 0)
GRAY = (50, 50, 50)
LIGHT_GRAY = (100, 100, 100)
YELLOW = (255, 255, 0)

#################################################
# AStar Algorithm
#################################################
class AStar:
    def __init__(self, screen, start, end, start_orientation, end_orientation, MAP, rpm1 = 50, rpm2 = 100):
        self.screen = screen
        self.start_node = start
        self.end_node = end
        self.start_orientation = start_orientation
        self.end_orientation = end_orientation
        self.euclidean_threshold = 10
        self.theta_threshold = 30
        self.time = 2
        self.L = 40
        self.R = 0.33 
        self.dt = 0.1
        self.MAP = MAP
        self.RPM1 = rpm1
        self.RPM2 = rpm2
    
    def turtlebot(self, curr_node, curr_orient, rpm_left, rpm_right):
        positions = [curr_node]
        x, y = curr_node
        theta = math.radians(curr_orient)

        for _ in range(int(self.time / self.dt)):
            dtheta = (rpm_right - rpm_left) * self.R / self.L * self.dt
            theta += dtheta
            dx = 0.5 * self.R * (rpm_left + rpm_right) * math.cos(theta) * self.dt
            dy = 0.5 * self.R * (rpm_left + rpm_right) * math.sin(theta) * self.dt
            x, y = x + dx, y + dy
            positions.append((x, y))

        final_orient = math.degrees(theta)
        return positions, final_orient

    def generate_successors(self, curr_node, curr_orient):
        actions = [
            (0, self.RPM1),
            (self.RPM1, 0),
            (self.RPM1, self.RPM1),
            (0, self.RPM2),
            (self.RPM2, 0),
            (self.RPM2, self.RPM2),
            (self.RPM1, self.RPM2),
            (self.RPM2, self.RPM1)
        ]

        all_neighbors = []

        for action in actions:
            rpm_left, rpm_right = action
            positions, final_orient = self.turtlebot(curr_node, curr_orient, rpm_left, rpm_right)
            x, y = positions[-1]
            n = ((int(x), int(y)), final_orient, positions, (rpm_left, rpm_right))

            if n is not None and n[0][0] >= 0 and n[0][0] < self.MAP.shape[1] and n[0][1] >= 0 and n[0][1] < self.MAP.shape[0] and self.MAP[n[0][1]][n[0][0]] != -1:
                all_neighbors.append(n)

        return all_neighbors


    def plot_vectors(self, parent_node, child_node):
        positions = child_node[2]
        
        for i in range(len(positions) - 1):
            pygame.draw.line(self.screen, BLACK, positions[i], positions[i+1], 2)

        end_position = child_node[0]
        dx = end_position[0] - parent_node[0]
        dy = end_position[1] - parent_node[1]
        angle = math.atan2(dy, dx)

        arrow_length = 4
        arrow_width = 3
        arrow_points = [
            (end_position[0] - arrow_length * math.cos(angle) + arrow_width * math.sin(angle), end_position[1] - arrow_length * math.sin(angle) - arrow_width * math.cos(angle)),
            (end_position[0], end_position[1]),
            (end_position[0] - arrow_length * math.cos(angle) - arrow_width * math.sin(angle), end_position[1] - arrow_length * math.sin(angle) + arrow_width * math.cos(angle))
        ]
        pygame.draw.polygon(self.screen, RED, arrow_points)
        pygame.display.flip()

    def angle_difference(self, angle1, angle2):
        diff = abs(angle1 - angle2)
        if diff > 180:
            diff = 360 - diff
        return diff

    def action_cost(self, rpm_values):
        if rpm_values in [(0, self.RPM1), (self.RPM1, 0), (0, self.RPM2), (self.RPM2, 0)]:
            return 1.4
        elif rpm_values in [(self.RPM1, self.RPM2), (self.RPM1, self.RPM1)]:
            return 1.2
        elif rpm_values in [(self.RPM1, self.RPM1), (self.RPM2, self.RPM2)]:
            return 1
        else:
            return 1 

    def heuristic_cost(self, node1, rpm_values):
        action_cost = self.action_cost(rpm_values)

        cost_to_end_node = math.sqrt((self.end_node[0] - node1[0])**2 + (self.end_node[1] - node1[1])**2)
        return action_cost + cost_to_end_node

    def is_goal_reached(self, current_node, curr_orient):
        if math.sqrt((self.end_node[0] - current_node[0])**2 + (self.end_node[1] - current_node[1])**2) <= self.euclidean_threshold:
            return True
        return False

    def is_close_to_explored(self, node, explored, threshold=3):
        for exp_node, _, _ in explored.keys(): 
            if math.sqrt((exp_node[0] - node[0])**2 + (exp_node[1] - node[1])**2) <= threshold:
                return True
        return False
    
    def generate_path(self, parents, final_orient, final_node, final_rpm):
        print("Generating Path")
        path = []
        state = (final_node, final_orient, final_rpm)
        while state is not None:
            path.append((state[0], round(state[1]) % 360, state[2]))
            state = parents.get(state)
        path.reverse()
        return path

    def explore(self):
        start_heuristic = self.heuristic_cost(self.start_node, (0,0))
        open_set = [(start_heuristic, (self.start_orientation, self.start_node, (0, 0)))]
        parent_nodes = {}
        closed_set = {}
        explored = {}
        neighbors = self.generate_successors(self.start_node, self.start_orientation)
        if neighbors is None:
            return parent_nodes, None
        else:
            while open_set:
                curr_cost, current_node = heapq.heappop(open_set)

                if self.is_goal_reached(current_node[1], current_node[0]):
                    return self.generate_path(explored, current_node[0], current_node[1], current_node[2])

                neighbors = self.generate_successors(current_node[1], current_node[0])
                
                for neighbor in neighbors:
                    n_cost = curr_cost + self.heuristic_cost(neighbor[0], neighbor[3])

                    for i, (f, n) in enumerate(open_set):
                        if f > n_cost:
                            del open_set[i]
                            break
                        
                    if neighbor[0] not in closed_set or n_cost < closed_set[neighbor[0]]:
                        if not self.is_close_to_explored(neighbor[0], explored):
                            closed_set[neighbor[0]] = n_cost
                            heapq.heappush(open_set, (n_cost, (neighbor[1], neighbor[0], neighbor[3])))  
                            parent_nodes[(neighbor[0], neighbor[3])] = (current_node[1], current_node[2])  
                            explored[(neighbor[0], neighbor[1], neighbor[3])] = (current_node[1], current_node[0], current_node[2])  
                            self.plot_vectors(current_node[1], neighbor)

        return parent_nodes, None

    
#################################################
# Visualization
#################################################
def draw_shapes(clearance = 30):
    screen.fill(LIGHT_GRAY)

    # Draw the shapes
    rect1 = pygame.draw.rect(screen, RED, (375, 0, 38, 312))
    rect2 = pygame.draw.rect(screen, RED, (625, 188, 38, 312))

    center = (1000, 225)
    radius = 125
    pygame.draw.circle(screen, RED, center, radius)
    
    font = pygame.font.Font('freesansbold.ttf', 8)
    direc_center = (60, 430)
    arrow_end = [((direc_center[0]-25,direc_center[1]),"180"),((direc_center[0]+25,direc_center[1]),"0"),((direc_center[0],direc_center[1]-25),"270"),((direc_center[0],direc_center[1]+25),"90")]
    for arrow, text in arrow_end:
        text_surface = font.render(text, True, WHITE, LIGHT_GRAY)
        text_rect = text_surface.get_rect()
        
        dx = arrow[0] - direc_center[0]
        dy = arrow[1] - direc_center[1]
        angle = math.atan2(dy, dx)

        pygame.draw.line(screen, BLACK, direc_center, arrow, 2)

        arrow_length = 4
        arrow_width = 3
        arrow_points = [
            (arrow[0] - arrow_length * math.cos(angle) + arrow_width * math.sin(angle), arrow[1] - arrow_length * math.sin(angle) - arrow_width * math.cos(angle)),
            arrow,
            (arrow[0] - arrow_length * math.cos(angle) - arrow_width * math.sin(angle), arrow[1] - arrow_length * math.sin(angle) + arrow_width * math.cos(angle))
        ]
        pygame.draw.polygon(screen, BLACK, arrow_points)
        
        text_rect.centerx = arrow_points[0][0] + arrow_width * math.sin(angle)
        text_rect.centery = arrow_points[0][1] - arrow_width * math.cos(angle)
        
        screen.blit(text_surface, text_rect)

    
    pygame.display.update()
    MAP = create_map(screen, clearance)

    for i in range(MAP.shape[0]):
        for j in range(MAP.shape[1]):
            if MAP[i][j] == -1:
                screen.set_at((j, i), WHITE)

    rect1 = pygame.draw.rect(screen, RED, (375, 0, 38, 312))
    rect2 = pygame.draw.rect(screen, RED, (625, 188, 38, 312))

    center = (1000, 225)
    radius = 125
    pygame.draw.circle(screen, RED, center, radius)

    pygame.display.update()
    return screen


def create_map(screen, clearance):
    global MAP
    if os.path.isfile(MAP_FILENAME):
        with open(MAP_FILENAME, 'r') as f:
            MAP = [list(map(int, line.strip().split(','))) for line in f.readlines()]
        print("Map loaded from file:", MAP_FILENAME)
    else:
        for i in range(0, WINDOW_HEIGHT):
            for j in range(0, WINDOW_WIDTH):
                if screen.get_at((j, i)) == RED:
                    for ii in range(i - clearance, i + clearance):
                        for jj in range(j - clearance, j + clearance):
                            if ii >= 0 and ii < WINDOW_HEIGHT and jj >= 0 and jj < WINDOW_WIDTH:
                                MAP[ii][jj] = -1

        for i in range(0, WINDOW_HEIGHT):
            for j in range(0, WINDOW_WIDTH):
                if i < 25 or i >= WINDOW_HEIGHT - 25 or j < 25 or j >= WINDOW_WIDTH - 25:
                    MAP[i][j] = -1

        with open(MAP_FILENAME, 'w') as f:
            f.write('\n'.join(','.join(str(cell) for cell in row) for row in MAP))
        print("Map saved to file:", MAP_FILENAME)
    MAP = np.array(MAP)
    return MAP

def select_box(screen, start_pos, end_pos):
    selected_box = start_pos
    pygame.draw.circle(screen, BLUE, selected_box, 10)
    end_x = selected_box[0] + 50
    end_y = selected_box[1]
    pygame.draw.line(screen, RED, selected_box, (end_x, end_y), 2)
    pygame.display.update()

def draw_points(point_list):
    radius = 5

    points = [node[0] for node in point_list]

    for i, point in enumerate(points):
        pygame.draw.circle(screen, BLUE, point, radius)
        if i > 0:
            pygame.draw.line(screen, YELLOW, points[i-1], point)

    pygame.display.update()

def save_path(path, orient, filename='path.txt'):
    with open(filename, 'w') as f:
        for node in path:
            position, orient, rpm = node
            x, y = position
            rpm1, rpm2 = rpm
            if y >= 250:
                y = -((y - 250) * 4) / 1000
            elif y < 250:
                y = ((250 - y) * 4) / 1000
            if x >= 125:
                x = ((x - 125) * 4) / 1000
            elif x < 125:
                x = -((125 - x) * 4) / 1000
            f.write(f'{x},{y}\n')
    print(f'Path saved to {filename}')

def parse_arguments():
    parser = argparse.ArgumentParser(description="A* Algorithm with Turtlebot")
    parser.add_argument("--RPM1", type=int, default=50, help="RPM for the first wheel (default: 50)")
    parser.add_argument("--RPM2", type=int, default=100, help="RPM for the second wheel (default: 100)")
    
    return parser.parse_args()

def main():
    while True:
        screen = draw_shapes()
        start_node, st_orientation = (125,250),0
        select_box(screen, start_node, st_orientation)
        end_node, end_orientation = (1375,250),0
        select_box(screen, end_node, end_orientation)
        if st_orientation < 0:
            st_orientation = 360 + st_orientation
        if end_orientation < 0:
            end_orientation = 360 + end_orientation
        print(st_orientation,end_orientation)
        args = parse_arguments()
        if MAP[start_node[1],start_node[0]] == -1 or MAP[end_node[1],end_node[0]] == -1 or start_node == end_node:
            tk.Tk().wm_withdraw()
            messagebox.showinfo("Try Again! Start node and End node are same")
            break
        else:
            start_time = (time.process_time())
            print(f"Start node: {start_node} and End node: {end_node}")
            # astar = AStar(screen, start_node, end_node, st_orientation, end_orientation, MAP, args.RPM1, args.RPM2)
            astar = AStar(screen, (125,250), (1375,250), 0, end_orientation, MAP, args.RPM1, args.RPM2)
            path = astar.explore()
            if path is not None:
                print(f"Path: {path}")
                screen = draw_shapes()
                draw_points(path)
                save_path(path, st_orientation)
            else:
                tk.Tk().wm_withdraw()
                messagebox.showinfo("Try Again! Could not Find Path")
            print(f"Time taken to process: {(time.process_time()-start_time)}")
            break

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

if __name__ == "__main__":
    main()