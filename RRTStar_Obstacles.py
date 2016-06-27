from kdTree import kdTree
from kdTree import node
import sys, random, pygame ,time
from pygame.locals import *
from math import sqrt,cos,sin,atan2, hypot
import kdtree
import numpy as np
import cv2

import sys
sys.setrecursionlimit(1500)

XDIM ,YDIM = 0, 0
WINSIZE = [XDIM, YDIM]
EPSILON = 10
NUMNODES = 5000
dim = 2
RADIUS = 20
p = 2
flag = False

class RRTAlgorithm(object):
    def __init__(self): #initial and destination coordinates and number of nodes
        self.getSourceAndGoal()
        global XDIM, YDIM
        XDIM, YDIM, z = img.shape
        print XDIM, YDIM
        self.costMap = [[10000000000 for x in range(YDIM)] for x in range(XDIM)]
        self.path = None
        self.goalNode = None
        self.start()

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

    def checkBoundaries(self, point):
        x, y, z = img.shape
        if point[0] >= 0 and point[1] >= 0 and point[0] < y and point[1] < x:
            return True
        return False

    def checkforObstacles(self, p1, p2):
        if img[p1[1]][p1[0]][0] == 255:
            return True
        else:
            return False

    def generatePoints(self):
        x, y, z = img.shape
        prob = 100.0*random.random()
        if prob < p:
             return self.goal
        else:
            rand = [int(random.random() * (y-1)*1.0), int(random.random() * (x-1)*1.0)]

            # while self.checkforObstacles(rand):
            #     rand = [int(random.random() * (y-1)*1.0), int(random.random() * (x-1)*1.0)]
            return rand

    def sortdist(self, n):
        return n[1]

    def updateSubtreCost(self, nde):
        for i in nde.children:
            if self.costMap[i.point[0]][ i.point[1]] < self.costMap[nde.point[0]][ nde.point[1]] + hypot(i.point[0]-nde.point[0], i.point[1]-nde.point[1]):
                print "yes"
            self.costMap[i.point[0]][ i.point[1]] = self.costMap[nde.point[0]][ nde.point[1]] + hypot(i.point[0]-nde.point[0], i.point[1]-nde.point[1])
            self.updateSubtreCost(i)

    def createNewLink(self, childlink, parentlink):

        pnt = childlink.point
        while True:
            childlink.propogateCost()
            oldParent = childlink.parent
            oldParent.children.remove(childlink)
            childlink.parent = parentlink
            parentlink.children.append(childlink)
            if oldParent.cost > childlink.cost + self.dist(oldParent.point, childlink.point):
                oldParent.cost = childlink.cost + self.dist(oldParent.point, childlink.point)
                if pnt == childlink.point:
                    cv2.line(img, tuple(parentlink.point), tuple(childlink.point), (100, 100, 100), 1)
                parentlink = childlink
                childlink = oldParent

            else:
                cv2.line(img, tuple(oldParent.point), tuple(childlink.point), (0, 0, 0), 1)
                break

    def dist(self, p1, p2):
        return hypot(p1[0]-p2[0], p1[1]-p2[1])

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

    def addConnections(self, Points, source):
        new_point = self.generatePoints()
        # print new_point
        ret = Points.search(new_point, 1000000000000000000, None, None, None, None, None)
        nodes = []

        nodes = Points.searchNN(new_point, RADIUS)
        # print len(nodes)
        flag = False
        for i in nodes:
            if ret[1] == i[0]:
                flag = True
                break
        if not flag:
            nodes.append((ret[1], ret[0]))

        sorted(nodes, key=self.sortdist)

        nn = nodes[0][0]
        # print 'nn ', nn
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
        ret = Points.search(pnt, 100000000000000, None, None, None, None, None)
        nearest_neighbour = ret[1]
        new_point = self.step_from_to(nearest_neighbour, new_point)

        new_point = [int(new_point[0]), int(new_point[1])]

        while self.checkforObstacles(new_point, nearest_neighbour):
            new_point = self.generatePoints()
            #print new_point
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
            #Pointmap[new_point[0]][new_point[1]] = 1
        cv2.line(img, tuple(nn), tuple(new_point), (100, 100, 100), 1)
        c = mincost
        nde = node(new_point, [], ret[2], True, c)
        ret[2].add_child(nde)
        Points.insert(new_point, 2, nde)

        '''

        Update Other links

        '''

        flag = False
        if self.goalNode !=None:
            flag = True
            if self.path != None:
                for i in nodes:
                    pnt1 = i[0]
                    if pnt != pnt1 and pnt1 in self.path:
                        flag = True
        if flag:
            nde1 = self.goalNode
            while nde1.parent != None:
                cv2.line(img, tuple(nde1.point), tuple(nde1.parent.point), (100, 100, 100), 1)
                nde1 = nde1.parent
            #print "rubbed"
            #time.sleep(10)
            #cv2.imshow('rubbed', img)
            #cv2.waitKey(0)

        for i in range(len(nodes)):
            pnt = nodes[i][0]
            if pnt != nearest_neighbour and pnt != source:
                if cost[i] > c + hypot(pnt[0]-new_point[0], pnt[1]-new_point[1]) and hypot(pnt[0]-new_point[0], pnt[1]-new_point[1]) < EPSILON:
                    child = Points.search(pnt, 10000000000000000000000000, None, None, None, None, None)[2]
                    child.cost = c + hypot(pnt[0] - new_point[0], pnt[1] - new_point[1])
                    self.createNewLink(child, nde)

        if flag:
            nodes = Points.searchNN(self.goal, 10)
            sorted(nodes, key=self.sortdist)
            pnt = nodes[0][0]
            self.path = []
            nde = self.goalNode = Points.search(pnt, 1000000000000000000, None, None, None, None, None)[2]
            while nde.parent != None:
                cv2.line(img, tuple(nde.point), tuple(nde.parent.point), (200, 200,200), 1)
                self.path.append(nde.point)
                #print "point ", nde.point
                nde = nde.parent
        return new_point

    def start(self):
        white = 100, 100, 100
        black = 20, 20, 40
        bright = 255, 255, 255

        RRTree = node(self.source, [], None, True) #actual RRTree
        Points = kdTree(None, None, 0, self.source, RRTree) #for storing generated points to increase the search complexity
        NearestNeighbourTree = kdtree.create([self.source]) # for storing points same as Points, using it for finding points in a range
        current = self.source
        #Pointmap = [[0 for i in range (YDIM)] for i in range(XDIM)]
        #Pointmap[self.source[0]][self.source[1]] = 1

        count  = 0

        while not self.check(current, self.goal):

            current = self.addConnections(Points, self.source)
            cv2.imshow('image', img)
            k = cv2.waitKey(1)
            count = count + 1
            #current = self.addConnection1(Points, rand, screen)

        ret = Points.search(current, 100000000000000000000, None, None, None, None, None)
        nde = ret[2]
        self.goalNode = nde
        path = []
        while nde.parent != None:
            print nde.point, nde.cost
            path.append(nde.point)
            cv2.line(img, tuple(nde.point), tuple(nde.parent.point), (200, 200,200))
            cv2.imshow('image', img)
            k = cv2.waitKey(1)
            nde = nde.parent

            if nde.parent == nde:
                break
        print 'count', count
        self.path = path
        for i in range(1000000-count):
            current = self.addConnections(Points, self.source)
            cv2.imshow('image', img)
            k = cv2.waitKey(1)
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

    def draw_circle(self, event, x, y, flags, param):
            global flag

            if event == cv2.EVENT_LBUTTONDBLCLK:
                # cv2.circle(img, (x, y), 100, (255, 0, 0), -1)
                if not flag:
                    self.source = x, y
                    print self.source
                    flag = True
                else:
                    self.goal = x, y
                    print self.goal

    def getSourceAndGoal(self):
            cv2.namedWindow('image')
            cv2.setMouseCallback('image', self.draw_circle)
            cv2.imshow('image', img)
            cv2.waitKey(0)

    def step_from_to(self,p1, p2): #returns point with at most epsilon distance from nearest neighbour in the direction of randomly generated point
        if self.dist(p1, p2) < EPSILON:
            return p2
        else:
            theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
            return [p1[0] + EPSILON * cos(theta), p1[1] + EPSILON * sin(theta)]

def main():

    tree = RRTAlgorithm()
img = cv2.imread('Images\obstacle.png')
main()