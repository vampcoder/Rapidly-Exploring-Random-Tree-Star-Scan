import math

class node:
    def __init__(self, point, children, parent, actual): # node of RRT
        self.children = children # list of all children as a sigle node may have multiple number of nodes
        self.point = point
        self.parent = parent
        self.actual = actual

    def add_child(self, node1):
        self.children.append(node1)

class kdTree:
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

    def search(self, point, dist, refp, nde): # for searching nearest neighbour
        axis = self.axis
        if self.left == None and self.right == None:
            w = abs(self.point[axis]-point[axis])
            ret = []
            if w < dist:
                ret.append(w)
                ret.append(self.point)
                ret.append(self.nde)
                return ret
            else:
                ret.append(dist)
                ret.append(refp)
                ret.append(nde)
                return ret

        else:
            d = abs(self.point[axis]-point[axis])
            if d < dist:
                dist = d
                refp = self.point
                nde = self.nde
            if point[axis] <= self.point[axis]:
                if point[axis]-dist <= self.point[axis] and self.left != None:
                    ret = self.left.search(point, dist, refp, nde)
                    dist = ret[0]
                    refp = ret[1]
                    nde = ret[2]
                if point[axis]+dist > self.point[axis] and self.right != None:
                    return self.right.search(point, dist, refp, nde)
            else:
                if point[axis] + dist > self.point[axis] and self.right != None:
                    ret = self.right.search(point, dist, refp, nde)
                    dist = ret[0]
                    refp = ret[1]
                    nde = ret[2]
                if point[axis] - dist <= self.point[axis] and self.left != None:
                    return self.left.search(point, dist, refp, nde)
            ret = []
            ret.append(dist)
            ret.append(refp)
            ret.append(nde)
            return ret

    def findMin(self, d, dim): # for finding minimum node in subtree rooted at nde
        print 'traverse', self.point
        if self.right == None and self.left == None:
            return self


        if self.axis == d: #condition that current node's division is equal to searched node'd division
            if self.left == None:
                return self
            else:
                return self.left.findMin(d, dim)

        #Search both direction : left and right
        temp1 = None
        temp2 = None
        if self.left == None:
            po = [100000000000 for i in range(dim)]
            temp1 = kdTree(None, None, d, po, None)
        else:
            temp1 = self.left.findMin(d, dim)

        if self.right == None:
            po = [100000000000 for i in range(dim)]
            temp2 = kdTree(None, None, d, po, None)
        else:
            temp2 = self.right.findMin(d, dim)

        temp = self
        x = min(temp.point[d], min(temp1.point[d], temp2.point[d]))
        if x == temp.point[d]:
            return temp
        elif x == temp1.point[d]:
            return temp1
        else:
            return temp2

    def deleteNode(self): #for deleting a node nde
        pass

    def print_tree(self):
        if self.left != None:
            self.left.print_tree()
        print self.point, self.axis
        if self.right != None:
            self.right.print_tree()


def main():
    print 'Enter Points'
    n = int(input()) # number of points
    dim = int(input()) #number of dimensions

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
    ret = root.findMin(0, dim)
    print ret.point, ret.axis

if __name__ == "__main__" :
    main()