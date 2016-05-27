import math

class kdTree:
    def __init__(self, left, right, axis, point):
        self.left = left
        self.right = right
        self.axis = axis
        self.point = point

    def insert(self, point, dim):
        tp = self.point
        tx = self.axis
        if tp[tx] > point[tx]:
            if self.left == None:
                self.left = kdTree(None, None, (tx+1)%dim, point)
            else:
                self.left.insert(point, dim)
        else:
            if self.right == None:
                self.right = kdTree(None, None, (tx+1)%dim, point)
            else:
                self.right.insert(point, dim)

    def search(self, point, dist, refp):
        axis = self.axis
        if self.left == None and self.right == None:
            w = abs(self.point[axis]-point[axis])
            ret = []
            if w < dist:
                ret.append(w)
                ret.append(self.point)
                return ret
            else:
                ret.append(dist)
                ret.append(refp)
                return ret

        else:
            d = abs(self.point[axis]-point[axis])
            if d < dist:
                dist = d
                refp = self.point
            if point[axis] <= self.point[axis]:
                if point[axis]-dist <= self.point[axis] and self.left != None:
                    ret = self.left.search(point, dist, refp)
                    dist = ret[0]
                    refp = ret[1]
                if point[axis]+dist > self.point[axis] and self.right != None:
                    return self.right.search(point, dist, refp)
            else:
                if point[axis] + dist > self.point[axis] and self.right != None:
                    ret = self.right.search(point, dist, refp)
                    dist = ret[0]
                    refp = ret[1]
                if point[axis] - dist <= self.point[axis] and self.left != None:
                    return self.left.search(point, dist, refp)
            ret = []
            ret.append(dist)
            ret.append(refp)
            return ret


    def print_tree(self):
        if self.left != None:
            self.left.print_tree()
        print self.point
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
            root = kdTree(None, None, 0, point)
        else:
            root.insert(point, dim)

    root.print_tree()

    print "Enter a point to search"
    point = []
    for i in range(dim):
        point.append(int(input()))

    ret = root.search(point, 1000000000000, None)
    print "Nearest Neighbour "
    print ret

if __name__ == "__main__" :
    main()