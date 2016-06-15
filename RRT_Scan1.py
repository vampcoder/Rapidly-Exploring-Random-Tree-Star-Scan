import cv2
from math import pi, sin, cos, hypot, atan2
import numpy as np
from kdTree import kdTree
from kdTree import node
import random
import copy
from Queue import PriorityQueue
import time

#global Declarations
sx, sy = -1, -1
dx, dy = -1, -1
flag = False
EPSILON = 20.0
NUMNODES = 5000
dim = 2
threshold = 4 #breaking condition of RRT loop

class pnt(object): #for heuristic implemantation
    def __init__(self, x, y, nde, goal):
        self.x = x
        self.y = y
        self.nde = nde
        self.goal = goal

    def findHeuristic(self):
        return hypot(self.x - self.goal[0], self.y - self.goal[1])

    def __cmp__(self, other):
        return cmp(self.nde.cost+ self.findHeuristic(), other.nde.cost+other.findHeuristic())

class RRTmodifiedAlgo():

    def __init__(self):
        self.getSourceAndGoal()
        global sx, sy, dx, dy
        self.source = [sx, sy]
        self.goal = [dx, dy]
        self.RRTree = node(self.source, [], None, True)  # actual RRTree
        self.Points = kdTree(None, None, 0, self.source, self.RRTree, None)  # for storing generated points to increase the search complexity
        self.black = None
        self.gray = None
        self.path = [self.source]
        self.current = self.source  #current position of a robot in map
        self.queue = PriorityQueue()
        self.turn = 0
        self.goalFound = False
        self.startProcessing()

    def dist(self, p1, p2):
        return hypot(p1[0]-p2[0], p1[1]-p2[1])

    def checkBondaries(self, p, img):
        rx, ry, rz = img.shape
        if p[0] < 0 or p[1] < 0 or p[0] >= ry or p[1] >= rx:
            return False
        return True

    def check_same(self, p1, p2):
        if int(p1[0]) <= int(p2[0])+1 and int(p1[1]) <= int(p2[1])+1 and int(p1[0]) >= int(p2[0])-1 and int(p1[1]) >= int(p2[1])-1:
            return True
        return False

    def check_for_black(self, p1, p2):  #check if a point is in black region or not by checking if edge joining it cuts any obstacle region
        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
        t1 = p1
        i = 0
        pnt = (int(t1[0]), int(t1[1]))
        i = 0
        #cv2.circle(self.img, pnt, 2, (0, 255, 255), 1)
        #cv2.imshow('image', self.img)
        #cv2.waitKey(0)
        #print "entered Black"
        while not self.check_same(t1, p2):
            t1 = [p1[0] + i * cos(theta), p1[1] + i * sin(theta)]
            if not self.checkBondaries(t1, self.img):
                return True
            #print t1
            if self.img[int(t1[1])][int(t1[0])][0] == 255:
        #        print "left black with true"
                return True
            i = i + 1
        #print "left black with false"
        return False

    def check_for_gray(self, p2): #Check if a point is in gray region or not by checking its reachability from source point
        p1 = self.current
        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
        t1 = p1
        pnt = (t1[0], t1[1])
        i = 0
        #cv2.circle(self.img, pnt, 2, (0, 255, 255), 1)
        #cv2.imshow('image', self.img)
        #cv2.waitKey(0)
        while not self.check_same(t1, p2):
            t1 = [int(p1[0] + i * cos(theta)), int(p1[1] + i * sin(theta))]
            # if self.turn == 1:
            #     cv2.circle(self.img, tuple(t1), 1, (0, 255, 0), 1)
            #     cv2.waitKey(1)
            if not self.checkBondaries(t1, self.img):
                return True
            if self.img[int(t1[1])][int(t1[0])][0] == 255:
                return True
            i = i + 1
        return False

    def checkInsideBlack(self, nn):
        if self.black == None:
            return False
        ret = self.black.search(nn, 1000000000000000000000, None, None)
        # if self.turn == 1:
        #     print ret[0]
        if ret[0] < 1:
            return True
        return False

    def printString(self, str):
        if self.turn == 1:
            print str

    def growRRT(self):
        print "Queue Empty ? ", self.queue.empty()
        print 'Current ', self.current
        count = 0
        X, Y, Z = img.shape
        cv2.imshow('self image', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        img1 = self.img
        cv2.imshow('image1', img1)
        cv2.waitKey(1)

        while not self.goalFound and count < 10:

            rand = [int(random.random() * Y * 1.0), int(random.random() * X * 1.0)]
            ret = self.Points.search(rand, 100000000000000, None, None)
            nearest_neighbour = ret[1]
            new_point = self.step_from_to(nearest_neighbour, rand)

            if not self.check_for_black(nearest_neighbour, new_point):
                #self.printString("not a black node")
                if not self.checkInsideBlack(nearest_neighbour):
                    if not self.check_for_gray(new_point):
                        self.printString("not a gray node even")
                        cv2.line(img1, tuple(nearest_neighbour), tuple(new_point), (0, 0, 255), 1)
                        cv2.imshow('image1', img1)
                        cv2.waitKey(1)
                        nde = node(new_point, [], ret[2], True)
                        if self.checkIfPathExist(new_point):
                            print "goal Found"

                            self.goalFound = True
                        ret[2].add_child(nde)
                        self.Points.insert(new_point, dim, nde)
                        if self.dist(new_point, nearest_neighbour) <= threshold:
                            count = count + 1
                    else:
                        #self.printString("gray node")
                        if self.gray == None:
                            self.gray = kdTree(None, None, 0, nearest_neighbour, ret[2], None)
                        ret = self.gray.search(nearest_neighbour, 10000000000000000000, None, None)
                        if ret[0] != 0:
                            self.gray.insert(nearest_neighbour, 2, ret[2])
                            self.queue.put(pnt(nearest_neighbour[0], nearest_neighbour[1], ret[2], self.goal))
            else:
                #self.printString("black node")
                if self.black == None:
                    self.black = kdTree(None, None, 0, nearest_neighbour, ret[2], None)
                ret = self.black.search(nearest_neighbour, 100000000000000000, None, None)
                if ret[0] != 0:
                    self.black.insert(nearest_neighbour, 2, ret[2])

        self.turn = self.turn + 1

    def step_from_to(self, p1, p2):  # returns point with at most epsilon distance from nearest neighbour in the direction of randomly generated point
        if self.dist(p1, p2) < EPSILON:
            return p2
        else:
            theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
            return [int(p1[0] + EPSILON * cos(theta)), int(p1[1] + EPSILON * sin(theta))]

    def findNearestObstacle(self, Img, x, y, theta):
        #print theta
        rx, ry, rz = Img.shape
        theta = pi*theta/180
        step = 20
        while x < rx and y < ry and x >= 0 and y >= 0:
            if Img[int(x)][int(y)][0] == 255:
                break
            else:
                x = x + step*sin(theta)
                y = y + step*cos(theta)

        if x >= rx or y >= ry or x < 0 or y < 0:
            while x >= rx or y >= ry or x < 0 or y < 0:
                x = x - sin(theta)
                y = y - cos(theta)
            return x, y

        while Img[int(x)][int(y)][0] == 255:
            x = x-sin(theta)
            y = y-cos(theta)

        return x, y

    def markVisibleArea(self, originalImg, visibleImg, x, y):
        lx, ly = -200, -200 #last coordinates
        for i in range(361):
            nx, ny = self.findNearestObstacle(originalImg, x, y, i)
            nx = int(nx)
            ny = int(ny)
            visibleImg[nx][ny] = (255, 255, 255)
            if i != 0 and hypot(ny-ly, nx-lx) < 70:
                cv2.line(visibleImg, (ny, nx), (ly, lx), (255, 255, 255), 3)

            lx, ly = nx, ny

        cv2.imshow('image', visibleImg)
        cv2.imshow('original', originalImg)
        cv2.waitKey(0)
        self.img = visibleImg

    def draw_circle(self, event, x, y, flags, param):
        global sx, sy, dx, dy, flag

        if event==cv2.EVENT_LBUTTONDBLCLK:
            #cv2.circle(img, (x, y), 100, (255, 0, 0), -1)
            if not flag:
                sx, sy = x, y
                print sx, sy
                flag = True
            else:
                dx, dy = x, y
                print dx, dy

    def getSourceAndGoal(self):
        cv2.namedWindow('image')
        cv2.setMouseCallback('image', self.draw_circle)
        cv2.imshow('image', img)
        cv2.waitKey(0)

    def checkIfPathExist(self, p): # Checks if direct path has been found using RRT only
        if p[0] < self.goal[0] + 5 and p[1] < self.goal[1] + 5 and p[0] > self.goal[0] - 5 and p[1] > self.goal[1] - 5:
            return True
        return False

    def check_goal(self): # Ckecks if robot has reached the goal or not
        if self.current[0] < self.goal[0] + 2 and self.current[1] < self.goal[1] + 2 and self.current[0] > self.goal[0]-2 and self.current[1] > self.goal[1]-2:
            return True
        return False

    def walk_step(self):
        flag = False
        p = None
        while not flag:
            p = self.queue.get()
            ret = self.black.search([p.x, p.y], 100000000000000000, None, None)
            if ret[0] == 0:
                continue
            flag = True

        print "heuristic ", p.nde.point
        cv2.circle(self.img, tuple(p.nde.point), 3, (0, 255, 0), 3)
        cv2.circle(self.img, tuple(self.goal), 3, (0, 255, 0), 3)
        cv2.imshow('image1', self.img)
        cv2.waitKey(0)

        nde = p.nde
        while nde.parent.parent != None:
            nde = nde.parent
        cv2.line(self.img, tuple(nde.point), tuple(self.current), (255, 255, 0), 1)
        self.current = nde.point

        nde.cost = 0
        nde.updateCost()

    def startProcessing(self):
        arr = np.zeros(img.shape)
        self.img = arr
        while not self.check_goal() and not self.goalFound:
            self.markVisibleArea(img, self.img, self.current[1], self.current[0])
            print "visible marked"
            self.growRRT()
            print "Tree has been grown"
            self.walk_step()
            print "Step has been taken"
            time.sleep(5)
            cv2.destroyAllWindows()

img = cv2.imread('Images/obstacle1.png')

start = RRTmodifiedAlgo()

cv2.destroyAllWindows()