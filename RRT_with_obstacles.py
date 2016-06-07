from kdTree import kdTree
from kdTree import node
import sys, random, math, pygame, time
from pygame.locals import *
from math import sqrt, cos, sin, atan2

XDIM = 640
YDIM = 480
WINSIZE = [XDIM, YDIM]
EPSILON = 10.0
dim = 2
p = 5

class RRTAlgorithm:
    def __init__(self, source, goal, nodes):  # initial and destination coordinates and number of nodes
        self.start(source, goal, nodes)

    def generateRandom(self):
        return [random.random() * 640.0, random.random() * 480.0]

    def getPoints(self, goal):
        if 100.0 * random.random() < p:
            return goal
        else:
            return self.generateRandom()

    def drawObstacles(self, screen):
        pygame.draw.circle(screen, (255, 255, 255), (320,240), 50)

    def checkObstacles(self, pnt, screen):
        pixelarray = pygame.PixelArray(screen)
        pnt = [int(pnt[0]), int(pnt[1])]
        if pixelarray[pnt[0], pnt[1]] == screen.map_rgb((255, 255, 255)):
            #print pixelarray[pnt[1], pnt[0]]
            return True
        return False

    def start(self, source, goal, nodes):
        pygame.init()
        screen = pygame.display.set_mode(WINSIZE)
        pygame.display.set_caption('RRT brute force')
        white = 100, 100, 100
        black = 20, 20, 40
        bright = 255, 255, 255
        screen.fill(black)
        self.drawObstacles(screen)

        RRTree = node(source, [], None, True)  # actual RRTree
        Points = kdTree(None, None, 0, source, RRTree,
                        None)  # for storing generated points to increase the search complexity
        current = source

        while not self.check(current, goal):
            rand = self.getPoints(goal)
            ret = Points.search(rand, 100000000000000000, None, None)
            nearest_neighbour = ret[1]
            new_point = self.step_from_to(nearest_neighbour, rand)

            while self.checkObstacles(new_point, screen):
                rand = self.getPoints(goal)
                ret = Points.search(rand, 100000000000000000, None, None)
                nearest_neighbour = ret[1]
                new_point = self.step_from_to(nearest_neighbour, rand)

            # print rand, nearest_neighbour, new_point
            # time.sleep(0.1)
            nde = node(new_point, [], ret[2], True)
            ret[2].add_child(nde)
            Points.insert(new_point, dim, nde)
            current = new_point
            pnt = [int(new_point[0]), int(new_point[1])]
            pygame.draw.line(screen, white, nearest_neighbour, new_point)
            # pygame.draw.circle(screen, (255, 0, 0), pnt, 1)
            pygame.display.update()
            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    sys.exit("Leaving .")

        ret = Points.search(current, 100000000000000000000, None, None)
        nde = ret[2]

        while nde.parent != None:
            pygame.draw.line(screen, bright, nde.point, nde.parent.point)
            pygame.display.update()
            nde = nde.parent
            time.sleep(0.1)
            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    sys.exit("Leaving.")

    def check(self, point, goal):  # checking if currently added node is at goal or not
        if point[0] > goal[0] - 5 and point[0] < goal[0] + 5 and point[1] > goal[1] - 5 and point[1] < goal[1] + 5:
            return True
        return False

    def dist(self, p1, p2):  # returns euclid's distance between points p1 and p2
        return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

    def step_from_to(self, p1,
                     p2):  # returns point with at most epsilon distance from nearest neighbour in the direction of randomly generated point
        if self.dist(p1, p2) < EPSILON:
            return p2
        else:
            theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
            return [p1[0] + EPSILON * cos(theta), p1[1] + EPSILON * sin(theta)]

def main():
    tree = RRTAlgorithm([500, 400], [5, 5], 2000)

main()