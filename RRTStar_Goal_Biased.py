from kdTree import kdTree
from kdTree import node
import sys, random, pygame ,time
from pygame.locals import *
from math import sqrt,cos,sin,atan2, hypot
import kdtree
import numpy as np

XDIM = 640
YDIM = 480
WINSIZE = [XDIM, YDIM]
EPSILON = 50
NUMNODES = 5000
dim = 2
RADIUS = 20
p = 2

class RRTAlgorithm(object):
    def __init__(self, source, goal, nodes): #initial and destination coordinates and number of nodes
        print source, goal
        self.costMap = [[10000000000 for x in range(YDIM)] for x in range(XDIM)]
        self.path = None
        self.goalNode = None
        self.start(source, goal, nodes)

    def addConnection1(self, Points, rand, screen):

        ret = Points.search(rand, 100000000000000, None, None, None, None, None)
        nearest_neighbour = ret[1]
        new_point = self.step_from_to(nearest_neighbour, rand)
        nde = node(new_point, [], ret[2], True, ret[2].cost + hypot(new_point[0] - ret[1][0], new_point[1] - ret[1][1]))
        ret[2].add_child(nde)
        Points.insert(new_point, dim, nde)
        current = new_point
        new_point = [int(new_point[0]), int(new_point[1])]
        pygame.draw.line(screen, (100, 100, 100), nearest_neighbour, new_point)
        pygame.display.update()
        return new_point
        pass

    def generatePoints(self, Pointmap):
        prob = 100.0*random.random()
        if prob < p:
             return self.goal
        else:
            rand = [int(random.random() * 640.0), int(random.random() * 480.0)]
            while Pointmap[rand[0]][rand[1]] == 1:
                rand = [int(random.random() * 640.0), int(random.random() * 480.0)]
            return rand
    def sortdist(self, n):
        return n[1]

    def updateSubtreCost(self, nde):
        for i in nde.children:
            if self.costMap[i.point[0]][ i.point[1]] < self.costMap[nde.point[0]][ nde.point[1]] + hypot(i.point[0]-nde.point[0], i.point[1]-nde.point[1]):
                print "yes"
            self.costMap[i.point[0]][ i.point[1]] = self.costMap[nde.point[0]][ nde.point[1]] + hypot(i.point[0]-nde.point[0], i.point[1]-nde.point[1])
            self.updateSubtreCost(i)

    def createNewLink(self, childlink, parentlink, screen):

        oldParent = childlink.parent
        oldParent.children.remove(childlink)
        childlink.parent = parentlink
        parentlink.children.append(childlink)
        childlink.propogateCost()
        #self.updateSubtreCost(childlink)

        pygame.draw.line(screen, (20, 20, 40), oldParent.point, childlink.point)
        pygame.draw.line(screen, (100, 100, 100), parentlink.point, childlink.point)

    def findKNN(self, point, Pointmap):
        start = [int(point[0]-RADIUS), int(point[1]-RADIUS)]
        end = [int(point[0] + RADIUS), int(point[1] + RADIUS)]
        if start[0] < 0:
            start[0] = 0
        if start[1] < 0:
            start[1] = 0
        if end[0] >= XDIM:
            end[0] = XDIM-1
        if end[1] >= YDIM:
            end[1] = YDIM-1
        ret = []

        for i in range(start[0], end[0]):
            for j in range(start[1], end[1]):
                if Pointmap[i][j] == 1:
                    ret.append(([i, j], hypot(point[0]-i, point[1]-j)))
        return ret
        pass

    def addConnections(self, Points, Pointmap, new_point, screen, source):
        ret = Points.search(new_point, 1000000000000000000, None, None, None, None, None)
        nodes = []

        nodes = Points.searchNN(new_point, RADIUS)
        #print len(nodes)
        flag = False
        for i in nodes:
            if ret[1] == i[0]:
                flag = True
                break
        if not flag:
            nodes.append((ret[1], ret[0]))

        sorted(nodes, key = self.sortdist)

        nn = nodes[0][0]
        #print 'nn ', nn
        cost = []
        for i in nodes:
            cost.append(Points.search(i[0], 100000000000000000000, None, None, None, None, None)[2].cost)
        mincost = 10000000000000000000000000
        for i in range(len(nodes)):
            nn1 = nodes[i][0]
            if cost[i] + nodes[i][1] < mincost:
                mincost = cost[i] + nodes[i][1]
                nn = nn1
        pnt = nn
        ret  = Points.search(pnt, 100000000000000, None, None, None, None, None)
        nearest_neighbour = ret[1]
        new_point = self.step_from_to(nearest_neighbour, new_point)
        new_point = [int(new_point[0]), int(new_point[1])]
        Pointmap[new_point[0]][new_point[1]] = 1
        pygame.draw.line(screen, (100, 100, 100), nn, new_point)
        pygame.display.update()
        c = mincost
        nde = node(new_point, [], ret[2], True, c)
        ret[2].add_child(nde)
        Points.insert(new_point, 2, nde)

        '''

        Update Other links

        '''

        flag = False
        # if self.goalNode !=None:
        #     flag = True
        if self.path != None:
            for i in nodes:
                pnt1 = i[0]
                if pnt != pnt1 and pnt1 in self.path:
                    flag = True
        if flag:
            nde1 = self.goalNode
            while nde1.parent != None:
                pygame.draw.line(screen, (100, 100, 100), nde1.point, nde1.parent.point)
                nde1 = nde1.parent
            #print "rubbed"
            pygame.display.update()
            #time.sleep(10)

        for i in range(len(nodes)):
            pnt = nodes[i][0]
            if pnt != nearest_neighbour and pnt != source:
                if cost[i] > c + hypot(pnt[0]-new_point[0], pnt[1]-new_point[1]) and hypot(pnt[0]-new_point[0], pnt[1]-new_point[1]) < EPSILON:
                    child = Points.search(pnt, 10000000000000000000000000, None, None, None, None, None)[2]
                    child.cost = c + hypot(pnt[0] - new_point[0], pnt[1] - new_point[1])
                    self.createNewLink(child, nde, screen)

        if flag:
            nodes = Points.searchNN(self.goal, 10)
            sorted(nodes, key=self.sortdist)
            pnt = nodes[0][0]
            self.path = []
            nde = self.goalNode = Points.search(pnt, 1000000000000000000, None, None, None, None, None)[2]
            while nde.parent != None:
                pygame.draw.line(screen, (255, 255, 255), nde.point, nde.parent.point)
                self.path.append(nde.point)
                #print "point ", nde.point
                nde = nde.parent
            pygame.display.update()
        return new_point

    def start(self, source, goal, nodes):
        self.source = source
        self.goal = goal
        pygame.init()
        screen = pygame.display.set_mode(WINSIZE)
        pygame.display.set_caption('RRT star using KdTrees')
        white = 100, 100, 100
        black = 20, 20, 40
        bright = 255, 255, 255
        screen.fill(black)

        RRTree = node(source, [], None, True) #actual RRTree
        Points = kdTree(None, None, 0, source, RRTree) #for storing generated points to increase the search complexity
        NearestNeighbourTree = kdtree.create([source]) # for storing points same as Points, using it for finding points in a range
        current = source
        Pointmap = [[0 for i in range (YDIM)] for i in range(XDIM)]
        Pointmap[source[0]][source[1]] = 1

        count  = 0

        while not self.check(current, goal):

            rand = self.generatePoints(Pointmap)
            current = self.addConnections(Points, Pointmap, rand, screen, source)
            count = count + 1
            #current = self.addConnection1(Points, rand, screen)

            pygame.display.update()
            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    sys.exit("Leaving .")

        ret = Points.search(current, 100000000000000000000, None, None, None, None, None)
        nde = ret[2]
        self.goalNode = nde
        path = []
        while nde.parent != None:
            print nde.point, nde.cost
            path.append(nde.point)
            pygame.draw.line(screen, bright, nde.point, nde.parent.point)
            pygame.display.update()
            nde = nde.parent
            if nde.parent == nde:
                break
            time.sleep(0.05)
            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    sys.exit("Leaving.")
        print 'count', count
        self.path = path
        for i in range(10000-count):
            rand = self.generatePoints(Pointmap)
            current = self.addConnections(Points, Pointmap, rand, screen, source)
            pygame.display.update()
            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    sys.exit("Leaving.")

    def printchildren(self, nde):
        print nde.point, nde.cost
        for i in nde.children:
            print i.point, i.cost

    def check(self, point , goal): # checking if currently added node is at goal or not
        if point[0] > goal[0]-5 and point[0] < goal[0]+5 and point[1] > goal[1]-5 and point[1] < goal[1]+5:
            return True
        return False

    def dist(self, p1, p2): #returns euclid's distance between points p1 and p2
        return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

    def step_from_to(self,p1, p2): #returns point with at most epsilon distance from nearest neighbour in the direction of randomly generated point
        if self.dist(p1, p2) < EPSILON:
            return p2
        else:
            theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
            return [p1[0] + EPSILON * cos(theta), p1[1] + EPSILON * sin(theta)]

def main():
    tree = RRTAlgorithm([635, 475], [20, 40], 2000)

main()