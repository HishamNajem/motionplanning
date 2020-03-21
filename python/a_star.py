#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from mapinfo import MapInfo
from copy import deepcopy
import math
import numpy as np


class AStar(object):
    def __init__(self, start, end, map_info):
        self._s = start
        self._e = end
        self._map_info = map_info
        self._openset = dict()
        self._closeset = dict()

    def distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def neighbor_nodes(self, x):
        # for hybrid
        #dist = 0.15
        dist = 0.05
        plist = [(x[0] - dist, x[1] - dist), (x[0] - dist, x[1]), (x[0] - dist, x[1] + dist), (x[0], x[1] + dist),
                 (x[0] + dist, x[1] + dist), (x[0] + dist, x[1]), (x[0] + dist, x[1] - dist), (x[0], x[1] - dist)]
        for p in plist:
            #print(p)
            if not self._map_info.is_collision(point=p):
                yield p

    def reconstruct_path(self):
        pt = self._e
        path = []
        while pt:
            path.append(pt)
            pt = self._closeset[pt]['camefrom']
        return path[::-1]

    def run(self, display=False):
        h = self.distance(self._s, self._e)
        # print(h)
        self._openset[self._s] = {'g': 0, 'h': h, 'f': h, 'camefrom': None}
        #print(self._openset)
        while self._openset:
            x = min(self._openset, key=lambda key: self._openset[key]['f'])
            #print(x)
            self._closeset[x] = deepcopy(self._openset[x])
            del self._openset[x]
            if self.distance(x, self._e) < 0.1:
                if x != self._e:
                    self._closeset[self._e] = {'camefrom': x}
                return True
            if display:
                self._map_info.close = x
            for y in self.neighbor_nodes(x):
                #print(y)
                if y in self._closeset:
                    continue
		tentative_g_score = self._closeset[x]['g'] + self.distance(x, y)
                if y not in self._openset:
                    tentative_is_better = True
                elif tentative_g_score < self._openset[y]['g']:
                    tentative_is_better = True
                else:
                    tentative_is_better = False
                if tentative_is_better:
                    h = self.distance(y, self._e)
                    self._openset[y] = {
                        'g': tentative_g_score, 'h': h, 'f': tentative_g_score + h, 'camefrom': x}
                    if display:
                        self._map_info.open = y
        return False


if __name__ == "__main__":
    m = MapInfo(2, 1.115)
    m.show()
    m.start = (0.1, 0.9)
    m.end = (1.8, 1)
    m.obstacle = [(0.3, i) for i in np.linspace(0.5, 1.115, num=100)]
    #raw_input('enter to start ...')
    plan = AStar(m.start, m.end, m)
    if plan.run(display=False):
        m.path = plan.reconstruct_path()
    print (m.path)
    m.wait_close()
