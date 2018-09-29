import cv2
import numpy as np

reducedimagebgr = cv2.imread('reduced.pgm')
reducedimge = cv2.cvtColor(reducedimagebgr, cv2.COLOR_BGR2GRAY)
maph, mapw = reducedimge.shape
parent = {}

#specify source and destination coordinates on the map here.

si = 13
sj = 20
di = 21
dj = 19


fvals = {}
gvals = {}
hvals = {}
ol = []
cl = []


def getCheapest():
    for point, cost in sorted(fvals.iteritems(), key=lambda (k,v): (v,k)):
        print point
        if point in ol:
            return point


def getfree(list):
    olist = []
    for (i,j) in list:
        if reducedimge[i][j] > 250 and ((i, j) not in cl):
            olist.append((i, j))
    return olist


def getH((i, j)):
    return (abs(di-i)+abs(dj-j))*10


def getG((i, j), p = None):
    global parent
    if p is None:
        pi, pj = parent[(i, j)]
    else:
        pi, pj = p
    if abs(pi-i) + abs(pj-j) > 1:
        cost = 14
    else:
        cost = 10
    if parent[(i, j)] == (si, sj):
        return cost
    else:
        return getG(parent[(i, j)])+cost


def getAdjacents(si, sj):
    if si==0 and sj==0:
        return getfree([(0,1), (1, 0), (1,1)])
    elif si == maph-1 and sj == mapw-1:
        return getfree([(maph-2, mapw-1), (maph-1, mapw-2), (maph-2, mapw-2)])
    elif si == 0 and sj == mapw-1:
        return getfree([(0, mapw-2), (1, mapw-1), (1, mapw-2)])
    elif si == maph-1 and sj == 0:
        return getfree([(maph-2, 0), (maph-2, 1), (maph-1, 1)])
    elif si == 0:
        return getfree([(0, sj-1), (0, sj+1), (si+1, sj+1), (si+1, sj), (si+1, sj-1)])
    elif si == maph-1:
        return  getfree([(si, sj-1), (si, sj+1), (si-1, sj), (si-1, sj-1), (si-1, sj+1)])
    elif sj == mapw-1:
        return getfree([(si, sj-1), (si+1, sj-1), (si-1, sj-1), (si+1, sj), (si-1, sj)])
    elif sj == 0:
        return getfree([(si, sj+1), (si, sj-1), (si, sj+1), (si+1, sj+1), (si-1, sj+1)])
    else:
        return getfree([(si, sj+1), (si, sj-1), (si+1, sj), (si-1, sj), (si+1, sj+1), (si-1, sj-1), (si+1, sj-1), (si-1, sj+1)])


def pathplan((si, sj), (di, dj)):
    global ol, cl, parent
    ol.append((si, sj))
    point = si, sj
    parent[point] = (si, sj)
    gvals[point] = getG(point)
    hvals[point] = getH(point)
    fvals[point] = gvals[point] + hvals[point]
    pathimage = reducedimagebgr.copy()
    i, j = getCheapest()
    while i != di or j!=dj:
        print i, j, "this is i, j"
        ol.remove((i, j))
        cl.append((i, j))
        neigh = getfree(getAdjacents(i, j))
        for point in neigh:
            if point in ol:
                ti, tj = point
                if abs(ti - i) + abs(tj - j) > 1:
                    cost = 14
                else:
                    cost = 10
                temp_g = gvals[(i, j)] + cost
                if gvals[point] > temp_g:
                    parent[point] = (i, j)
                    gvals[point] = temp_g
                    fvals[point] = hvals[point]+temp_g
            else:
                parent[point] = (i, j)
                gvals[point] = getG(point)
                hvals[point] = getH(point)
                fvals[point] = gvals[point] + hvals[point]
                ol.append(point)
        i, j = getCheapest()
    ri = di
    rj = dj
    path = []
    while ri !=si or rj != sj:
        path.append((ri, rj))
        ri, rj = parent[(ri, rj)]
    path.append((si, sj))
    path.reverse()
    for (i, j) in path:
        color = cv2.rectangle(pathimage, (j, i), (j, i), [255, 0, 0], -1)
    color = cv2.rectangle(color, (sj, si), (sj, si), [0, 0, 255], -1)
    color = cv2.rectangle(color, (dj, di), (dj, di), [0, 255, 0], -1)
    cv2.imwrite('path.pgm', color)
    print path
 
pathplan((si, sj), (di, dj))