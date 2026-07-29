import cv2
from math import pi, sin, cos, hypot, atan2
import numpy as np
from kdTree import kdTree
from kdTree import node
import random
import copy
import time
import queue as Q

#global Declarations
sx, sy = -1, -1
dx, dy = -1, -1
flag = False
EPSILON = 20.0
NUMNODES = 5000
dim = 2
threshold = 4 #breaking condition of RRT loop
p = 5

img = cv2.imread('Images/obstacle.png')

class RRTmodifiedAlgo():

    def __init__(self):
        self.getSourceAndGoal()
        global sx, sy, dx, dy
        self.source = [sx, sy]
        self.goal = [dx, dy]
        self.RRTree = node(self.source, [], None, True)  # Permanent RRTree
        self.Points = kdTree(None, None, 0, self.source, self.RRTree)  # for storing generated points to increase the search complexity, Currently storing points of normal RRT
        self.tempPoints = None # currently storing points of Goal biased which is being generated to form RRT complete and stores those extra points in kdTree data structure.
        self.leafNodes = [] #storing all the nodes
        self.path = [self.source]
        self.current = self.source  #current position of a robot in map
        self.turn = 0
        self.goalFound = False
        self.extraPoints = []
        self.steps = []
        self.startProcessing()

    def dist(self, p1, p2):
        return hypot(p1[0]-p2[0], p1[1]-p2[1])

    def checkBondaries(self, p, img):
        rx, ry= img.shape
        if p[0] < 0 or p[1] < 0 or p[0] >= rx or p[1] >= ry:
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
        while not self.check_same(t1, p2):
            t1 = [p1[0] + i * cos(theta), p1[1] + i * sin(theta)]
            if not self.checkBondaries(t1, self.img):
                return True
            if self.img[int(t1[0])][int(t1[1])] == 255:
                return True
            i = i + 1
        return False

    def check_for_gray(self, p2): #Check if a point is in gray region or not by checking its reachability from source point
        if self.img[p2[0]][p2[1]] == 100:
            return False
        else:
            return True

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
            print(str)

    def storeleaves(self, rrtnode): #for storing leaf nodes
        #print rrtnode.children
        if len(rrtnode.children) == 0:
            self.leafNodes.append(rrtnode)
        for i in rrtnode.children:
            self.storeleaves(i)

    def generateGoalBiasPoints(self):
        x = random.random()*100
        X,Y = self.img.shape
        if x > 70:
            return [int(random.random() * (X-1) * 1.0), int(random.random() * (Y-1) * 1.0)]
        else:
            return self.goal

    def checkIfGoalFound(self, p): #checks if goal has been reached by temporary extended goal biased RRT
        if p[0]< self.goal[0] + 2 and p[0] > self.goal[0]-2 and p[1] < self.goal[1]+2 and p[1] > self.goal[1]-2:
            return True
        return False

    def goalBiastempRRT(self): #grow tree with goal bias-ness

        while True:
            rand = self.generateGoalBiasPoints()
            ret = self.Points.search(rand, 100000000000000, None, None, None, None, None)
            ret1 = ret
            if self.tempPoints != None:
                ret1 = self.tempPoints.search(rand, 100000000000000, None, None, None, None, None)
            if ret[0] > ret1[0]:
                ret = ret1
            nearest_neighbour = ret[1]
            new_point = self.step_from_to(nearest_neighbour, rand)
            if new_point[0] == nearest_neighbour[0] and new_point[1] == nearest_neighbour[1]:
                print("same point")
                continue

            if not self.check_for_black(nearest_neighbour, new_point):
                nde = node(new_point, [], ret[2], True)
                ret[2].add_child(nde)
                self.leafNodes.append((ret[2], nde))
                if self.tempPoints == None:
                    self.tempPoints = kdTree(None, None, 0, new_point, nde)
                else:
                    self.tempPoints.insert(new_point, dim, nde)
                self.extraPoints.append(new_point)
                if self.checkIfGoalFound(new_point):
                    while nde.parent.point != self.current:
                        nde = nde.parent
                    nde1 = nde.parent
                    nde.parent = None
                    nde.children.append(nde1)
                    nde1.children.remove(nde)
                    nde1.parent = nde
                    cv2.line(self.img, tuple(reversed(self.current)), tuple(reversed(nde.point)), 200 , 1)
                    self.steps.append([self.current, nde.point])
                    self.current = nde.point
                    break
                cv2.line(self.tempimg, tuple(reversed(nearest_neighbour)), tuple(reversed(new_point)), 200, 1)
                cv2.circle(self.tempimg, tuple(reversed(self.goal)), 3, 200 , 3)
                cv2.imshow('image2', self.tempimg)
                k = cv2.waitKey(1)
                if k == 27:
                    exit()

    def removegeneratedLeafNodes(self):
        for rrtnode in self.leafNodes:
            pnt = rrtnode[0].point
            ret = self.Points.search(pnt, 100000000000000, None, None, None, None, None)
            if ret[0] < 1:
                rrtnode[0].children.remove(rrtnode[1])
                rrtnode[1].parent = None

    def showCurrentTree(self, rrtnode):
        for i in rrtnode.children:
            cv2.line(self.img1, tuple(rrtnode.point), tuple(i.point), (0, 0, 255), 1)
            self.showCurrentTree(i)

    def checkIfRemoved(self):
        for i in self.extraPoints:
            ret = self.Points.search(i, 1000000000000000000, None, None, None, None, None)
            #print ret[0], ret[1], i

    def generatePoints(self):
        x = random.random() * 100
        X, Y = self.img.shape
        if x > p:
            return [int(random.random() * (X-1) * 1.0), int(random.random() * (Y-1) * 1.0)]
        else:
            return self.goal

    def normalRRT(self):
        count = 0
        X, Y = img.shape[:2]
        self.tempimg = copy.copy(self.treeimage)
        while not self.goalFound and count < 10:
            rand = self.generatePoints()
            if not self.checkBondaries(rand, self.img):
                continue
            ret = self.Points.search(rand, 100000000000000, None, None, None, None, None)
            nearest_neighbour = ret[1]
            new_point = self.step_from_to(nearest_neighbour, rand)

            if not self.check_for_black(nearest_neighbour, new_point):
                if not self.check_for_gray(new_point):
                    #print new_point
                    nde = node(new_point, [], ret[2], True)
                    ret[2].add_child(nde)
                    self.Points.insert(new_point, dim, nde)
                    if self.dist(new_point, nearest_neighbour) <= threshold:
                        count = count + 1

                    cv2.line(self.treeimage, tuple(reversed(nearest_neighbour)), tuple(reversed(new_point)), (0, 0, 255), 1)
                    cv2.imshow('image1', self.treeimage)
                    cv2.line(self.tempimg, tuple(reversed(nearest_neighbour)), tuple(reversed(new_point)), (0, 0, 255), 1)
                    cv2.imshow('image2', self.tempimg)
                    k = cv2.waitKey(1)

                    if k == 27:
                        exit()
                    if self.checkIfGoalFound(new_point):
                        self.goalFound = True
                        break

    def recPrint(self, point):
        if point.left != None:
            self.recPrint(point.left)
        nde = point.nde
        if nde.parent != None:
            cv2.line(self.treeimage, tuple(reversed(nde.point)), tuple(reversed(nde.parent.point)), 0, 1)
        if point.right != None:
            self.recPrint(point.right)


    def printWholeTree(self):
        self.treeimage = copy.copy(self.img)
        self.recPrint(self.Points)
        for i in self.steps:
            cv2.line(self.treeimage, tuple(reversed(i[0])), tuple(reversed(i[1])), 200, 1)
        # cv2.imshow('new_tree', self.treeimage)
        # cv2.waitKey(1)

    def growRRT(self):
        self.printWholeTree()
        self.normalRRT()
        if self.goalFound:
            return
        print(len(self.leafNodes))
        self.goalBiastempRRT()
        self.removegeneratedLeafNodes()

        self.tempPoints = None
        self.leafNodes = []

    def step_from_to(self, p1, p2):  # returns point with at most epsilon distance from nearest neighbour in the direction of randomly generated point
        if self.dist(p1, p2) < EPSILON:
            return p2
        else:
            theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
            return [int(p1[0] + EPSILON * cos(theta)), int(p1[1] + EPSILON * sin(theta))]

    def findNearestObstacle(self, Img, x, y, theta):
        #print theta
        rx, ry, rz = Img.shape
        #print rx, ry
        theta = pi*theta/180.0
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

        return x+sin(theta), y + cos(theta)

    def markVisibleArea(self, originalImg):
        visibleImg = np.zeros(self.img.shape, np.uint8)
        x, y = self.current[0], self.current[1]
        lx, ly = -200, -200 #last coordinates
        points = []
        for i in range(1083):
            nx, ny = self.findNearestObstacle(originalImg, x, y, i/3)
            #print nx, ny
            nx = int(nx)
            ny = int(ny)
            points.append((ny, nx))
            if i != 0:
                cv2.line(visibleImg, (ny, nx), (ly, lx), 100, 1)
            lx, ly = nx, ny
        h, w = visibleImg.shape

        mask = np.zeros((h+2, w+2), np.uint8)
        cv2.floodFill(visibleImg, mask, (y, x), 100)
        for i in points:
            cv2.circle(visibleImg, i, 3, 255, 6)

        self.img = cv2.bitwise_or(self.img, visibleImg)

    def draw_circle(self, event, x, y, flags, param):
        global sx, sy, dx, dy, flag

        if event==cv2.EVENT_LBUTTONDBLCLK:
            #cv2.circle(img, (x, y), 100, (255, 0, 0), -1)
            if not flag:
                sx, sy = y, x
                print(sx, sy)
                flag = True
            else:
                dx, dy = y, x
                print(dx, dy)

    def getSourceAndGoal(self):
        cv2.namedWindow('image')
        cv2.setMouseCallback('image', self.draw_circle)
        cv2.imshow('image', img)
        cv2.waitKey(1)

    def checkIfPathExist(self, p): # Checks if direct path has been found using RRT only
        if p[0] < self.goal[0] + 5 and p[1] < self.goal[1] + 5 and p[0] > self.goal[0] - 5 and p[1] > self.goal[1] - 5:
            return True
        return False

    def check_goal(self): # Ckecks if robot has reached the goal or not
        if self.current[0] < self.goal[0] + 2 and self.current[1] < self.goal[1] + 2 and self.current[0] > self.goal[0]-2 and self.current[1] > self.goal[1]-2:
            return True
        return False

    def startProcessing(self):
        arr = np.zeros(img.shape[:2], np.uint8)
        self.img = arr
        self.treeimage = np.zeros(img.shape[:2], np.uint8)
        #self.markVisibleArea(img)
        while not self.check_goal() and not self.goalFound:
            self.markVisibleArea(img)
            # cv2.imshow('image', self.img)
            # k = cv2.waitKey(1)
            print("visible marked")
            self.growRRT()
            print("Tree has been grown")
        print("goal Reached")

start = RRTmodifiedAlgo()

cv2.destroyAllWindows()
