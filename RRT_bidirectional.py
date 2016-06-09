from kdTree import kdTree
from kdTree import node
import sys, random, math, pygame, time
from pygame.locals import *
from math import sqrt, cos, sin, atan2

XDIM = 640
YDIM = 480
WINSIZE = [XDIM, YDIM]
EPSILON = 5.0
dim = 2
p = 1

class RRTBidirectionalAlgorithm:
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
        #self.drawObstacles(screen)

        RRTree1 = node(source, [], None, True)  # first RRTree from source
        RRTree2 = node(goal, [], None, True) #second RRTree from goal
        Points1 = kdTree(None, None, 0, source, RRTree1,
                        None)  # for storing generated points in RRTree1 to increase the search complexity
        Points2 = kdTree(None, None, 0, goal, RRTree2,
                         None)  # for storing generated points in RRTree2 to increase the search complexity

        #Points1.print_tree()

        #Points2.print_tree()
        for i in range(10000):

            #Doing work for first tree
            rand = self.getPoints(goal)
            ret = Points1.search(rand, 100000000000000000, None, None)
            nearest_neighbour = ret[1]
            new_point = self.step_from_to(nearest_neighbour, rand)
            while self.checkObstacles(new_point, screen):
                rand = self.getPoints(goal)
                ret = Points1.search(rand, 100000000000000000, None, None)
                nearest_neighbour = ret[1]
                new_point = self.step_from_to(nearest_neighbour, rand)

            nde = node(new_point, [], ret[2], True)
            ret[2].add_child(nde)
            Points1.insert(new_point, dim, nde)

            pnt = [int(new_point[0]), int(new_point[1])]
            pygame.draw.line(screen, white, nearest_neighbour, new_point)
            # pygame.draw.circle(screen, (255, 0, 0), pnt, 1)
            pygame.display.update()

            #print 'a'

            #repeating same thing for second tree but not towards random point but towards the point generated in last tree i.e, new_point
            ret = Points2.search(new_point, 100000000000000000, None, None)
            nearest_neighbour = ret[1]
            current = self.step_from_to(nearest_neighbour, new_point)

            nde = node(current, [], ret[2], True)
            ret[2].add_child(nde)
            Points2.insert(current, dim, nde)
            #print 'b'

            pnt = [int(current[0]), int(current[1])]
            pygame.draw.line(screen, white, nearest_neighbour, current)

            #print 'points ', new_point, current
            # pygame.draw.circle(screen, (255, 0, 0), pnt, 1)
            pygame.display.update()
            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    sys.exit("Leaving .")
            #Points1.print_tree()
            #Points2.print_tree()
            if current == new_point:
                #print 'c'
                break

            temp1 = Points1
            temp2 = RRTree1
            Points1 = Points2
            RRTree1 = RRTree2
            Points2 = temp1
            RRTree2 = temp2

        ret1 = Points1.search(current, 100000000000000000000, None, None)
        nde1 = ret1[2]
        ret2 = Points2.search(current, 100000000000000000000, None, None)
        nde2 = ret2[2]

        while nde1.parent != None:
            pygame.draw.line(screen, bright, nde1.point, nde1.parent.point)
            #print nde1.point
            pygame.display.update()
            nde1 = nde1.parent
            time.sleep(0.1)
            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    sys.exit("Leaving.")

        while nde2.parent != None:
            pygame.draw.line(screen, bright, nde2.point, nde2.parent.point)
            #print nde2.point
            pygame.display.update()
            nde2 = nde2.parent
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

    def step_from_to(self, p1, p2):  # returns point with at most epsilon distance from nearest neighbour in the direction of randomly generated point
        if self.dist(p1, p2) < EPSILON:
            return p2
        else:
            theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
            return [p1[0] + EPSILON * cos(theta), p1[1] + EPSILON * sin(theta)]

def main():
    tree = RRTBidirectionalAlgorithm([500, 400], [5, 5], 2000)

main()