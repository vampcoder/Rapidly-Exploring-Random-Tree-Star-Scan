from kdTree import kdTree
from kdTree import node
import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2


XDIM = 640
YDIM = 480
WINSIZE = [XDIM, YDIM]
EPSILON = 7.0
NUMNODES = 5000
dim = 2

class RRTAlgorithm:
    def __init__(self, source, goal, nodes): #initial and destination coordinates and number of nodes
        self .start(source, goal, nodes)

    def start(self, source, goal, nodes):
        RRTree = node(source, [], None, True)
        Points = kdTree(None, None, 0, source, RRTree)
        current = source

        while not self.check(current, goal):
            rand = [random.random() * 640.0, random.random() * 480.0]
            ret = Points.search(rand, 100000000000000, None)
            nearest_neighbour = ret[1]
            new_point = self.step_from_to(nearest_neighbour, rand)
            nde = node(new_point, [], ret[2], True)
            ret[2].add_child(nde)
            Points.insert(new_point, dim, nde)

    def check(self, point , goal): # checking if currently added node is at goal or not
        if point[0] > goal[0]-5 and point[0] < goal[0]+5 and point[1] > goal[1]-5 and point[1] < goal[1]+5:
            return True
        return False

    def dist(p1, p2): #returns euclid's distance between points p1 and p2
        return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

    def step_from_to(self,p1, p2): #returns point with at most epsilon distance from nearest neighbour in the direction of randomly generated point
        if self.dist(p1, p2) < EPSILON:
            return p2
        else:
            theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
            return [p1[0] + EPSILON * cos(theta), p1[1] + EPSILON * sin(theta)]

def main():
    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('RRT brute force')
    white = 255, 240, 200
    black = 20, 20, 40
    screen.fill(black)
