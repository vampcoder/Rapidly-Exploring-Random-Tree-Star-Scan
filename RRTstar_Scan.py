import cv2
from math import pi, sin, cos, hypot, atan2
import numpy as np
from kdTree import kdTree
from kdTree import node
import random
import copy
import time

#global Declarations
sx, sy = -1, -1
dx, dy = -1, -1
flag = False
EPSILON = 10.0
NUMNODES = 5000
dim = 2
threshold = 4 #breaking condition of RRT loop
p = 5
RADIUS = 12

class RRTmodifiedAlgo():

    def __init__(self):
        self.getSourceAndGoal()
        global sx, sy, dx, dy
        self.source = [sx, sy]
        self.goal = [dx, dy]
        self.RRTree = node(self.source, [], None, True)  # Permanent RRTree
        self.Points = kdTree(None, None, 0, self.source, self.RRTree)  # for storing generated points to increase the search complexity, Currently storing points of normal RRT
        self.tempPoints = None # currently storing points of Goal biased which is being generated to form RRT complete and stores those extra points in kdTree data structure.
        self.leafNodes = [] #storing all the nodes fron which nodes in new RRTree generated
        self.path = None  # path from source to goal
        self.current = self.source  #current position of a robot in map
        self.turn = 0
        self.goalFound = False
        self.goalNode = None
        self.extraPoints = []
        self.ArchivedTree = None
        self.startProcessing()

    def sortdist(self, n):
            return n[1]

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
                        cv2.line(self.img, tuple(parentlink.point), tuple(childlink.point), (100, 100, 100), 1)
                        cv2.line(self.tempimg, tuple(parentlink.point), tuple(childlink.point), (100, 100, 100), 1)
                    parentlink = childlink
                    childlink = oldParent

                else:
                    cv2.line(self.img, tuple(oldParent.point), tuple(childlink.point), (0, 0, 0), 1)
                    cv2.line(self.tempimg, tuple(oldParent.point), tuple(childlink.point), (0, 0, 0), 1)
                    break

    def dist(self, p1, p2):
            return hypot(p1[0] - p2[0], p1[1] - p2[1])

    def addConnections(self):
            source = self.current
            c = 0
            flag1 = False
            cost = []
            nodes = []
            pnt = None
            new_point = None
            nearest_neighbour = None
            ret = None

            while not flag1:
                #finding the nearest point to generated point
                new_point = self.generatePoints()
                ret = self.Points.search(new_point, 1000000000000000000, None, None, None, None, None)
                nearest_neighbour = ret[1]
                new_point = self.step_from_to(nearest_neighbour, new_point)
                new_point = [int(new_point[0]), int(new_point[1])]
                if not self.check_for_black(nearest_neighbour, new_point):
                    if not self.check_for_gray(new_point):
                        flag1 = True
                        break

            nos = self.Points.searchNN(new_point, RADIUS)
            # print len(nos)
            cost = 100000000000
            parent = None
            nodes = []
            for i in nos:
                ret = self.Points.search(i[0], 1000000000000000000000, None, None, None, None, None)
                if ret[2].cost + self.dist(new_point, ret[1]) < cost:
                    cost = ret[2].cost + self.dist(new_point, ret[1])
                    parent = ret[2]

                nodes.append(ret)

            cv2.line(self.img, tuple(parent.point), tuple(new_point), (100, 100, 100), 1)
            cv2.line(self.tempimg, tuple(parent.point), tuple(new_point), (100, 100, 100), 1)

            nde = node(new_point, [], parent, True, cost)
            parent.add_child(nde)
            self.Points.insert(new_point, 2, nde)

            '''
            Update Other links
            '''

            flag = False
            if self.goalNode != None:
                flag = True

            if flag:
                nde1 = self.goalNode
                while nde1.parent != None:
                    cv2.line(self.img, tuple(nde1.point), tuple(nde1.parent.point), (100, 100, 100), 1)
                    cv2.line(self.tempimg, tuple(nde1.point), tuple(nde1.parent.point), (100, 100, 100), 1)
                    nde1 = nde1.parent

            for i in nodes:
                if i[1] != parent.point:
                    if i[2].cost > self.dist(i[1], new_point) + cost:
                        i[2].cost = self.dist(i[1], new_point) + cost
                        self.createNewLink(i[2], nde)

            if flag:
                nodes = self.Points.searchNN(self.goal, 10)
                sorted(nodes, key=self.sortdist)
                pnt = nodes[0][0]
                self.path = []
                nde = self.goalNode = self.Points.search(pnt, 1000000000000000000, None, None, None, None, None)[2]
                while nde.parent != None:
                    cv2.line(self.img, tuple(nde.point), tuple(nde.parent.point), (200, 200, 200), 1)
                    cv2.line(self.tempimg, tuple(nde.point), tuple(nde.parent.point), (200, 200, 200), 1)
                    self.path.append(nde.point)
                    nde = nde.parent
            return new_point, parent.point

    def checkforObstacles(self, p1, p2):
        if img[p1[1]][p1[0]][0] == 255 or img[int((p1[1] + p2[1]) / 2)][int((p1[0] + p2[0]) / 2)][0] == 255:
            return True
        else:
            return False

    def checkBoundaries(self, p, img):
        rx, ry, rz = img.shape
        if p[0] < 0 or p[1] < 0 or p[0] >= ry or p[1] >= rx:
            return False
        return True

    def check_same(self, p1, p2):
        if int(p1[0]) <= int(p2[0])+1 and int(p1[1]) <= int(p2[1])+1 and int(p1[0]) >= int(p2[0])-1 and int(p1[1]) >= int(p2[1])-1:
            return True
        return False

    def check_for_black(self, p1, p2):  # check if a point is in black region or not by checking if edge joining it cuts any obstacle region
        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
        t1 = p1
        i = 0
        pnt = (int(t1[0]), int(t1[1]))
        i = 0
        while not self.check_same(t1, p2):
            t1 = [p1[0] + i * cos(theta), p1[1] + i * sin(theta)]
            if not self.checkBoundaries(t1, self.img):
                return True
            if self.img[int(t1[1])][int(t1[0])][0] == 255:
                return True
            i = i + 1
        return False

    def check_for_gray(self, p2): #Check if a point is in gray region or not by checking its reachability from source point
        p1 = self.current
        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
        t1 = p1
        pnt = (t1[0], t1[1])
        i = 0

        while not self.check_same(t1, p2):
            t1 = [int(p1[0] + i * cos(theta)), int(p1[1] + i * sin(theta))]
            if not self.checkBoundaries(t1, img):
                return True
            if img[int(t1[1])][int(t1[0])][0] == 255:
                return True
            i = i + 1
        return False

    def printString(self, str):
        if self.turn == 1:
            print str

    def storeleaves(self, rrtnode): #for storing leaf nodes
        #print rrtnode.children
        if len(rrtnode.children) == 0:
            self.leafNodes.append(rrtnode)
        for i in rrtnode.children:
            self.storeleaves(i)

    def generateGoalBiasPoints(self):
        x = random.random()*100
        X,Y,Z = self.img.shape
        if x > 70:
            return [int(random.random() * Y * 1.0), int(random.random() * X * 1.0)]
        else:
            return self.goal

    def checkIfGoalFound(self, p): #checks if goal has been reached by temporary extended goal biased RRT
        if p[0]< self.goal[0] + 2 and p[0] > self.goal[0]-2 and p[1] < self.goal[1]+2 and p[1] > self.goal[1]-2:
            return True
        return False

    def goalBiastempRRT(self): #grow tree with goal biasness

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
                print "same point"
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
                    cv2.line(self.img, tuple(self.current), tuple(nde.point), (0, 255, 255), 1)
                    self.current = nde.point
                    break
                cv2.line(self.tempimg, tuple(nearest_neighbour), tuple(new_point), (0, 255, 255), 1)
                cv2.circle(self.tempimg, tuple(self.goal), 3, (0, 0, 255), 3)
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

    def generatePoints(self):
        x = random.random() * 100
        X, Y, Z = self.img.shape
        if x > p:
            return [int(random.random() * Y * 1.0), int(random.random() * X * 1.0)]
        else:
            return self.goal

    def normalRRTstar(self):
        count = 0
        X, Y, Z = img.shape
        self.tempimg = copy.copy(self.img)
        while not self.goalFound and count < 10:
            # rand = [int(random.random() * Y * 1.0), int(random.random() * X * 1.0)]
            new_point, nearest_neighbour = self.addConnections()

            if self.dist(new_point, nearest_neighbour) <= threshold:
                count = count + 1

            cv2.imshow('image1', self.img)
            cv2.imshow('image2', self.tempimg)
            k = cv2.waitKey(1)
            if k == 27:
                exit()

            if self.checkIfGoalFound(new_point):
                self.goalFound = True
                break

    def growRRT(self):

        self.normalRRTstar()
        return
        if self.goalFound:
            return
        #self.storeleaves(self.RRTree)
        print len(self.leafNodes)
        self.goalBiastempRRT()
        self.removegeneratedLeafNodes()
        self.showCurrentTree(self.RRTree)
        self.tempPoints = None
        self.leafNodes = []
        #cv2.imshow('reduced tree', self.img1)
        #cv2.waitKey(1)

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

        return x+sin(theta), y + cos(theta)

    def markVisibleArea(self, originalImg, visibleImg, x, y):
        lx, ly = -200, -200 #last coordinates
        for i in range(361):
            nx, ny = self.findNearestObstacle(originalImg, x, y, i)
            nx = int(nx)
            ny = int(ny)
            #cv2. circle(visibleImg, (ny, nx), 2, (255, 255, 255), 2)
            visibleImg[nx][ny] = (255, 255, 255)

            if i != 0:
                theta = atan2(ly-ny, lx-nx)
                cx, cy = nx, ny
                j = 0
                if self.dist([nx, ny], [lx, ly]) < 5:
                    while not(cx == lx and cy == ly) and originalImg[int(cx)][int(cy)][0] == 255:
                        visibleImg[int(cx)][int(cy)] = (255, 255, 255)
                        cx = nx + j*cos(theta)
                        cy = ny + j*sin(theta)
                        j = j+1
                    visibleImg[int(cx)][int(cy)] = (255, 255, 255)
                    cx, cy = lx, ly
                    j = 0
                    while not(cx == nx and cy == ny) and originalImg[int(cx)][int(cy)][0] == 255:
                        visibleImg[int(cx)][int(cy)] = (255, 255, 255)
                        cx = lx - j * cos(theta)
                        cy = ly - j * sin(theta)
                        j = j + 1
                    visibleImg[int(cx)][int(cy)] = (255, 255, 255)
                    visibleImg[int(nx)][int(ny)] = (255, 255, 255)
                    visibleImg[int(lx)][int(ly)] = (255, 255, 255)
            lx, ly = nx, ny

        self.img1 = copy.copy(visibleImg)
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

    def startProcessing(self):
        arr = np.zeros(img.shape)
        self.img = arr
        while not self.check_goal() and not self.goalFound:
            self.markVisibleArea(img, self.img, self.current[1], self.current[0])
            print "visible marked"
            self.growRRT()
            print "Tree has been grown"

        print "goal Reached"

img = cv2.imread('Images/obstacle.png')

start = RRTmodifiedAlgo()

cv2.destroyAllWindows()