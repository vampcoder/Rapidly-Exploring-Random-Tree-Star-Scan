import math

class node:
    def __init__(self, point, children, parent, actual, cost = 0): # node of RRT
        self.children = children # list of all children as a sigle node may have multiple number of nodes
        self.point = point
        self.actual = actual
        self.cost = cost
        self.parent = parent

    def add_child(self, node1):
        self.children.append(node1)

    def propogateCost(self):
        if self in self.children:
            self.children.remove(self)
        for i in self.children:
            i.cost = self.cost + math.hypot(i.point[0]-self.point[0], i.point[1]-self.point[1])
            i.propogateCost()

class kdTree(object):
    def __init__(self, left, right, axis, point, nde):
        self.left = left
        self.right = right
        self.axis = axis
        self.point = point
        self.nde  = nde

    def insert(self, point, dim, nde): #for inserting a node in tree
        tp = self.point
        tx = self.axis
        if tp[tx] > point[tx]:
            if self.left == None:
                self.left = kdTree(None, None, (tx+1)%dim, point, nde)
            else:
                self.left.insert(point, dim, nde)
        else:
            if self.right == None:
                self.right = kdTree(None, None, (tx+1)%dim, point, nde)
            else:
                self.right.insert(point, dim, nde)

    def dist(self, point):
        total = 0
        for i in range(len(point)):
            total =  total + (self.point[i]-point[i])*(self.point[i]-point[i])
        total = math.sqrt(total)
        return total

    def search(self, point, dist, refp, nde, kdlink, kdlinkparent, selfparent): # for searching nearest neighbour
        axis = self.axis
        if self.left == None and self.right == None:
            #w = abs(self.point[axis]-point[axis])
            w = self.dist(point)
            ret = []
            if w < dist:
                ret.append(w)
                ret.append(self.point)
                ret.append(self.nde)
                ret.append(self)
                ret.append(selfparent)
                return ret
            else:
                ret.append(dist)
                ret.append(refp)
                ret.append(nde)
                ret.append(kdlink)
                ret.append(kdlinkparent)
                return ret

        else:
            d = self.dist(point)
            #d = abs(self.point[axis]-point[axis])
            if d < dist:
                dist = d
                refp = self.point
                nde = self.nde
                kdlink = self
                kdlinkparent = selfparent
            if point[axis] <= self.point[axis]:
                if point[axis]-dist <= self.point[axis] and self.left != None:
                    ret = self.left.search(point, dist, refp, nde, kdlink, kdlinkparent, self)
                    dist = ret[0]
                    refp = ret[1]
                    nde = ret[2]
                    kdlink = ret[3]
                    kdlinkparent = ret[4]
                if point[axis]+dist > self.point[axis] and self.right != None:
                    return self.right.search(point, dist, refp, nde, kdlink, kdlinkparent, self)
            else:
                if point[axis] + dist > self.point[axis] and self.right != None:
                    ret = self.right.search(point, dist, refp, nde, kdlink, kdlinkparent, self)
                    dist = ret[0]
                    refp = ret[1]
                    nde = ret[2]
                    kdlink = ret[3]
                    kdlinkparent = ret[4]
                if point[axis] - dist <= self.point[axis] and self.left != None:
                    return self.left.search(point, dist, refp, nde, kdlink, kdlinkparent, self)
            ret = []
            ret.append(dist)
            ret.append(refp)
            ret.append(nde)
            ret.append(kdlink)
            ret.append(kdlinkparent)
            return ret

    def searchNN(self, point, dist):  # for searching nearest neighbour in radius dist
        axis = self.axis
        if self.left == None and self.right == None:
            # w = abs(self.point[axis]-point[axis])
            w = self.dist(point)
            ret = []
            if w < dist:
                temp = [self.point, w]
                ret.append(temp)
                return ret
            else:
                return ret

        else:
            d = self.dist(point)
            # d = abs(self.point[axis]-point[axis])
            sol = []
            if d < dist:
                temp = [self.point, d]
                sol.append(temp)
            if point[axis] <= self.point[axis]:
                if point[axis] - dist <= self.point[axis] and self.left != None:
                    ret = self.left.searchNN(point, dist)
                    sol = sol + ret
                if point[axis] + dist > self.point[axis] and self.right != None:
                    ret = self.right.searchNN(point, dist)
                    sol = sol + ret
                    return sol
            else:
                if point[axis] + dist > self.point[axis] and self.right != None:
                    ret = self.right.searchNN(point, dist)
                    sol = sol+ret
                if point[axis] - dist <= self.point[axis] and self.left != None:
                    ret =  self.left.searchNN(point, dist)
                    sol = sol + ret
                    return sol
            return sol

    def findMin(self, d, dim, parent): # for finding minimum node in subtree rooted at nde
        #print 'traverse', self.point
        if self.right == None and self.left == None:
            return (self, parent)

        if self.axis == d: #condition that current node's division is equal to searched node'd division
            if self.left == None:
                return self, parent
            else:
                return self.left.findMin(d, dim, self)

        #Search both direction : left and right
        temp1 = None
        temp2 = None
        if self.left == None:
            po = [100000000000 for i in range(dim)]
            temp1 = kdTree(None, None, d, po, None), None
        else:
            temp1 = self.left.findMin(d, dim, self)

        if self.right == None:
            po = [100000000000 for i in range(dim)]
            temp2 = kdTree(None, None, d, po, None), None
        else:
            temp2 = self.right.findMin(d, dim, self)

        temp = self, parent
        x = min(temp[0].point[d], min(temp1[0].point[d], temp2[0].point[d]))
        if x == temp[0].point[d]:
            return temp
        elif x == temp1[0].point[d]:
            return temp1
        else:
            return temp2

    def set_none(self, node):
        pass

    def check_leaf(self):
        if self.right == None and self.left == None:
            return True
        return False

    def deleteNode(self, parent): #for deleting a node nde
        if self.check_leaf():
            if parent != None:
                if parent.right == self:
                    parent.right = None
                else:
                    parent.left = None
            del self
            return

        if self.right != None:
            mini = self.right.findMin(self.axis, dim, self)
            self.point = mini[0].point
            self.nde = mini[0].nde
            self.right.deleteNode(self)

        elif self.left != None:
            mini = self.left.findMin(self.axis, dim, self)
            self.point = mini[0].point
            self.nde = mini[0].nde
            self.left.deleteNode(self)
            self.right = self.left
            self.left = None

    def print_tree(self):
        if self.left != None:
            self.left.print_tree()
        print self.point, self.axis
        if self.right != None:
            self.right.print_tree()

dim = 2
def main():
    print 'Enter Points'
    n = int(input()) # number of points
    global dim
    temp = int(input()) #number of dimensions

    root = None

    for i in range(n):
        point = []
        for j in range(dim):
            point.append(int(input()))
        if root == None:
            root = kdTree(None, None, 0, point, None)
        else:
            root.insert(point, dim, None)

    root.print_tree()

    '''
    print "Enter a point to search"
    point = []
    for i in range(dim):
        point.append(int(input()))

    ret = root.search(point, 1000000000000, None, None)
    print "Nearest Neighbour "
    print ret
    '''
    print " "

    #ret = root.findMin(0, dim)
    #print ret.point, ret.axis
    #ret.deleteNode()
    #ret = root.findMin(1, dim)

    #print ret.point, ret.axis
    #ret = root.findMin(1, dim, None)
    ret = root.search([51, 75], 100000000000000000, None, None, None, None, None)
    print " "
    print ret[1]
    print " "
    ret[3].deleteNode(ret[4])
    #ret.deleteNode()
    root.print_tree()

if __name__ == "__main__" :
    main()